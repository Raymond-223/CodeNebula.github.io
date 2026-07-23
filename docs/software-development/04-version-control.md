# 版本控制

版本控制是现代软件工程的基石。没有版本控制，协作开发将陷入混乱，历史追溯无从谈起，发布回退风险极高。本章从 Git 的底层对象模型出发，逐层深入到核心命令、分支策略、合并机制、协作工作流及高级主题，最后给出可直接落地的团队规范模板。读者可将本章作为日常 Git 操作的权威参考手册。

---

## 一、Git 工作原理

理解 Git 的原理是精通 Git 的前提。Git 从根本上区别于 SVN、CVS 等集中式版本控制系统——它是一套**内容寻址文件系统**，上层封装了版本控制接口。

### 1.1 对象模型（Object Model）

Git 的核心存储单元是四种对象，全部以 **SHA-1 哈希值** 作为标识：

| 对象类型 | 存储内容 | 用途 | 示例 |
|----------|---------|------|------|
| **blob** | 文件内容（二进制/文本） | 存储文件的每一个版本 | `e69de29bb2d1d6434b8b29ae775ad8c2e48c5391` |
| **tree** | 目录清单：文件名 → blob/tree 映射 | 存储目录结构 | `4d5fcadc293b348a3f1f4e1e78d5b9c3e7b5f5b2` |
| **commit** | 元数据：tree hash + parent(s) + author + message | 存储一次快照 | `a1b2c3d4e5f6a7b8c9d0e1f2a3b4c5d6e7f8a9b0` |
| **tag** | 指向 commit + 额外元数据（可含 GPG 签名） | 为发布版本打标签 | `v1.0.0` |

**关键理解**：Git 不存储文件变更（diff），而是存储**完整快照**。当文件未变化时，多个 commit 共享同一个 blob，不会重复存储。

```
commit (SHA-1)
  ├── tree
  │   ├── blob "README.md"
  │   ├── tree "src/"
  │   │   ├── blob "main.go"
  │   │   └── blob "utils.go"
  │   └── blob "go.mod"
  ├── parent → commit (上一次提交)
  ├── author: Alice <alice@example.com>
  └── message: "feat: add logging module"
```

### 1.2 SHA-1 哈希与内容寻址

每个对象的哈希值通过 SHA-1 对内容本身计算得出：`hash = SHA-1(object_type + " " + content_size + "\0" + content)`。这意味着：

- **相同内容 → 相同哈希**：两个文件内容一样，无论路径如何，共享同一个 blob
- **内容被修改 → 哈希完全改变**：任何字节变更都会产生全新的对象
- **完整性保证**：Git 可通过哈希值校验内容是否被篡改或损坏

实践意义：Git 仓库本质上是一个 **key-value 数据库**，key 是 SHA-1 哈希，value 是对象内容。你可以直接用 `git cat-file -p <hash>` 查看任意对象。

### 1.3 .git 目录结构

每个 Git 仓库根目录下的 `.git` 文件夹是 Git 的数据库和精神所在：

```
.git/
├── HEAD                     # 指向当前分支的引用文件
├── config                   # 仓库级配置（用户信息、远程仓库等）
├── description              # 仅用于 GitWeb 的仓库描述
├── index                    # 暂存区（staging area）的二进制快照
├── objects/                 # 对象数据库
│   ├── 01/                  # SHA-1 前缀为 01 的对象
│   │   └── 23456789...      # 实际对象文件（松散对象 loose object）
│   ├── pack/                # 打包后的对象（packfile，用于压缩和传输）
│   │   ├── pack-*.pack
│   │   └── pack-*.idx
│   └── info/                # 辅助信息
├── refs/                    # 引用（references）
│   ├── heads/               # 本地分支（每个文件对应一个分支）
│   │   ├── main
│   │   ├── develop
│   │   └── feature/login
│   ├── tags/                # 标签
│   │   └── v1.0.0
│   └── remotes/             # 远程分支
│       └── origin/
│           ├── main
│           └── develop
├── logs/                    # reflog 日志
│   └── HEAD
└── hooks/                   # 钩子脚本模板
    ├── pre-commit.sample
    ├── commit-msg.sample
    └── pre-push.sample
```

**理解提示**：`objects/` 是真正的数据仓库，`refs/` 是目录索引，`HEAD` 是你当前所在位置。

### 1.4 Git 引用（References）

引用是指向 commit 的指针，按用途分为：

| 引用类型 | 存储位置 | 说明 |
|----------|----------|------|
| **HEAD** | `.git/HEAD` | 当前检出位置，通常指向分支（`ref: refs/heads/main`），也可直接指向 commit（detached HEAD） |
| **分支（branch）** | `refs/heads/*` | 可变指针，随新 commit 自动前移 |
| **标签（tag）** | `refs/tags/*` | 不可变指针，通常用于发布版本标记 |
| **远程分支** | `refs/remotes/*` | 远程仓库分支的本地缓存，只读 |
| **ORIG_HEAD** | `.git/ORIG_HEAD` | 执行危险操作（merge、reset）前，Git 记录原 HEAD 位置用于恢复 |
| **FETCH_HEAD** | `.git/FETCH_HEAD` | `git fetch` 拉取的分支信息 |
| **MERGE_HEAD** | `.git/MERGE_HEAD` | 合并过程中被合并的分支 commit |

### 1.5 三棵树（Three Trees）

Git 的工作状态可以用"三棵树"模型来理解：

```
Working Directory       Index (Staging)          HEAD (Repository)
    │                        │                        │
    │     git add            │     git commit         │
    ├───────────────────────►├───────────────────────►│
    │                        │                        │
    │     git checkout ◄─────┼────────────────────────┤
    │                        │                        │
    │◄── git restore --worktree ──┤                    │
    │◄──── git restore --staged ──────────────────────┤
```

| 树 | 位置 | 内容 | 作用 |
|----|------|------|------|
| **工作目录 (Working Directory)** | 你的文件系统 | 实际文件 | 你编辑代码的地方 |
| **暂存区 (Index/Staging)** | `.git/index` | 下次要提交的文件清单 | 组装 commit 的中间区域 |
| **HEAD** | `.git/refs/heads/*` | 最后一次提交的快照 | 版本历史的"当前指针" |

`git status` 的本质就是比较这三棵树之间的差异：
- 工作目录 vs 暂存区 → "未暂存的变更"（红色）
- 暂存区 vs HEAD → "将要提交的变更"（绿色）

---

## 二、Git 核心命令

### 2.1 初始化与克隆

```bash
# 创建新仓库
git init                                    # 在当前目录初始化
git init --bare /path/to/repo.git           # 创建裸仓库（服务器端）

# 克隆已有仓库
git clone https://github.com/user/repo.git              # HTTPS 克隆
git clone git@github.com:user/repo.git                  # SSH 克隆
git clone --depth 1 https://github.com/user/repo.git    # 浅克隆（节省带宽）
git clone --branch develop https://github.com/user/repo.git  # 克隆特定分支
```

`--depth 1` 浅克隆只拉取最新 commit，适合 CI/CD 环境。需要全量历史时不用此参数。

### 2.2 日常操作

```bash
git add <file>              # 将文件加入暂存区
git add .                   # 暂存所有变更
git add -p                  # 交互式分段暂存（分块审查）

git commit -m "message"     # 提交暂存区
git commit -a -m "message"  # 暂存所有已跟踪文件并提交（跳过 git add）
git commit --amend          # 修改上一次提交（消息或内容）

git status                  # 查看当前工作状态
git status -s               # 简化输出

git log --oneline --graph   # 简洁的提交历史图
git log -p                  # 显示每次提交的 diff
git log --author="Alice"    # 按作者过滤
git log --since="2 weeks ago"  # 按时间过滤
git log --grep="fix"        # 按提交消息搜索

git diff                    # 工作目录 vs 暂存区
git diff --staged           # 暂存区 vs HEAD
git diff HEAD               # 工作目录 vs HEAD
git diff branchA..branchB   # 两个分支的差异
```

**`git add -p` 是专业开发者的习惯**——它让你分块审查变更，避免意外提交调试代码或临时修改。

### 2.3 分支管理

```bash
git branch                              # 列出本地分支
git branch -a                           # 列出所有分支（包括远程）
git branch feature/login                # 创建分支
git branch -d feature/login             # 删除已合并分支
git branch -D feature/login             # 强制删除未合并分支

git checkout feature/login              # 切换到分支（经典方式）
git switch feature/login                # 切换到分支（新版命令）
git switch -c feature/payment           # 创建并切换

git checkout -b feature/payment         # 创建并切换（经典方式）
git switch -                            # 切换到上一个分支

git merge feature/login                 # 将 feature/login 合并到当前分支
git rebase main                         # 将当前分支变基到 main
```

**重要**：`git switch` 和 `git restore` 是 Git 2.23+ 引入的"拆分化"命令——`switch` 只管分支切换，`restore` 只管文件恢复，职责更清晰，推荐逐渐迁移使用。

### 2.4 撤销操作

这是 Git 中最容易混淆也最重要的命令组。关键区别在于**针对历史还是针对工作区**以及**是否保留历史记录**：

| 命令 | 作用范围 | 是否修改历史 | 典型场景 |
|------|----------|-------------|---------|
| `git restore <file>` | 工作目录文件 | 否 | 丢弃未暂存的修改 |
| `git restore --staged <file>` | 暂存区文件 | 否 | 取消已暂存的文件 |
| `git reset --soft HEAD~1` | commit 历史 | 是 | 撤销提交但保留修改（重新提交） |
| `git reset --mixed HEAD~1` | commit 历史+暂存区 | 是 | 撤销提交并将修改放到工作目录 |
| `git reset --hard HEAD~1` | 全部 | 是 | 彻底丢弃最近一次提交及其修改 |
| `git revert <commit>` | 新增一个反向 commit | 否（追加） | 安全撤销已推送的提交 |
| `git commit --amend` | 最近一次提交 | 是 | 修改提交信息或加入遗漏文件 |

```bash
# 恢复工作目录文件
git restore README.md                    # 丢弃 README.md 的未暂存修改
git restore --source=HEAD~2 main.go     # 将 main.go 恢复到两个版本前

# 取消暂存
git restore --staged main.go            # 等价于旧的 git reset HEAD main.go

# 重置提交（本地未推送时使用）
git reset --soft HEAD~1                 # 撤销提交，修改保留在暂存区
git reset --mixed HEAD~1                # 撤销提交，修改保留在工作目录
git reset --hard HEAD~1                 # 彻底丢弃最近一次提交（不可恢复！）

# 安全撤销已推送的提交
git revert a1b2c3d                      # 创建一个新 commit 来逆转 a1b2c3d 的变更
git revert HEAD~3..HEAD                 # 逆转最近 3 次提交（从旧到新）
```

**黄金法则**：**已推送（push）到远程仓库的 commit，永远不要使用 reset 操作。** 使用 `git revert`，它通过追加新 commit 来安全地撤销变更，不会破坏协作成员的历史。`reset --hard` 只适合本地未推送的 commit。

### 2.5 git stash

当你在一个分支上工作到一半，需要切换到其他分支处理紧急事务时，stash 让你暂存当前工作状态：

```bash
git stash push -m "WIP: login form validation"   # 暂存当前修改
git stash list                                   # 查看 stash 列表
git stash pop                                    # 恢复最近的 stash 并删除记录
git stash apply stash@{2}                        # 恢复指定 stash 但不删除
git stash drop stash@{0}                         # 删除指定 stash
git stash clear                                  # 清空所有 stash
git stash branch feature/new-branch              # 从 stash 创建分支（避免冲突）
```

使用 `git stash -u` 或 `git stash --include-untracked` 可一并暂存未跟踪的文件。

### 2.6 git cherry-pick

将指定 commit 的变更应用到当前分支：

```bash
git cherry-pick a1b2c3d                     # 将 a1b2c3d 的变更应用到当前分支
git cherry-pick a1b2c3d..e5f6g7h           # 应用一系列 commit（不包括 a1b2c3d）
git cherry-pick -x a1b2c3d                 # 添加来源引用（符合合规要求）
```

**适用场景**：将一个修复 commit 从发布分支反向移植（backport）到其他维护分支，而非将一个完整分支合并。

### 2.7 git reflog（恢复利器）

`git reflog` 记录 HEAD 的所有移动历史——即使你做了 `--hard reset` 或丢失了某个 commit，只要它在 reflog 保留期内（默认 90 天），就能找回：

```bash
$ git reflog
a1b2c3d HEAD@{0}: reset: moving to HEAD~1
b2c3d4e HEAD@{1}: commit: feat: add user authentication
c3d4e5f HEAD@{2}: commit: fix: correct email validation
d4e5f6g HEAD@{3}: merge: feature/login into main
e5f6g7h HEAD@{4}: commit (initial): initial project setup
```

**恢复示例**：

```bash
# 场景：误操作 git reset --hard HEAD~2，丢失了两个重要 commit
# 找回方法：
git reflog                       # 找到丢失 commit 的哈希
git cherry-pick b2c3d4e          # 或直接 cherry-pick 恢复
# 或
git checkout -b recovery-branch b2c3d4e   # 从该 commit 创建新分支恢复
```

`git reflog` 是本地操作，**不会同步到远程仓库**，因此它是每个开发者最后的救命稻草。

### 2.8 git bisect（二分查找 Bug）

当你知道某个功能在旧版本还能用、新版本却坏了时，`git bisect` 通过二分搜索在所有 commit 中快速定位引入 Bug 的那一个：

```bash
git bisect start
git bisect bad                # 标记当前版本为坏
git bisect good v1.0.0        # 标记一个好版本
# Git 检出中间 commit，你测试后标记：
git bisect good               # 当前版本没问题
git bisect bad                # 当前版本有问题
# ...重复直到找到第一个坏了 commit
git bisect reset              # 结束 bisect
```

更高效的自动化形式：

```bash
git bisect start HEAD v1.0.0
git bisect run npm test       # 让 Git 自动运行测试脚本判断好坏
git bisect reset
```

### 2.9 git blame

逐行查看文件每一行的最后一次修改时间和作者：

```bash
git blame main.go
git blame -L 50,100 main.go   # 只看 50-100 行
git blame -w main.go          # 忽略空白修改
```

`git blame` 常用于代码审查时快速定位某行代码的归属，或排查可疑修改。

---

## 三、分支策略

分支策略是团队协作的"交通规则"。选择合适的策略直接影响发布效率、代码质量和团队协作体验。

### 3.1 Git Flow

Git Flow 是 2010 年由 Vincent Driessen 提出的经典模型，适合有固定发布周期的项目。

```
master ───●──────────●──────────●─────────
           \        / \        / \
            \      /   \      /   \
develop ─────●────●────●────●────●────●
              \      /          \      /
               \    /            \    /
feature/*       ●──●              ●──●
                      \          /
                       \        /
release/*               ●──────●
                        /
                       /
hotfix/*   ●──────────●
```

| 分支类型 | 命名规则 | 来源 | 合并目标 | 说明 |
|----------|----------|------|----------|------|
| **master** | `master`/`main` | - | - | 生产就绪的代码，只能通过 release/hotfix 合并 |
| **develop** | `develop` | master | master | 集成分支，包含下一个发布版本的所有功能 |
| **feature** | `feature/*` | develop | develop | 新功能开发分支 |
| **release** | `release/*` | develop | master 和 develop | 发布准备分支，只修 Bug 不添功能 |
| **hotfix** | `hotfix/*` | master | master 和 develop | 生产环境紧急修复 |

**适用场景**：有固定发布周期（如每月/每季度）的成熟项目，需要同时维护多个版本。
**团队规模**：10-50 人。
**缺点**：分支繁杂，合入频繁，不适合持续部署（CD）。

### 3.2 GitHub Flow

GitHub 推荐的轻量模型，极简且适合持续部署。

```
main ──●─────────●──────────●─────────●
        \       / \        / \       /
         \     /   \      /   \     /
feature    ●──●     ●────●     ●──●
```

**核心规则**：
1. `main` 分支始终保持可部署状态
2. 从 `main` 创建 feature 分支
3. 在 feature 分支上提交修改
4. 提交 Pull Request 进行代码审查
5. 审查通过后合并到 `main` 并立即部署

**适用场景**：Web 应用等需要快速迭代、持续部署的项目。
**团队规模**：1-20 人。
**优点**：简单，无额外交互成本。
**缺点**：没有明确的发布版本管理，不适合需要同时维护多个版本的项目。

### 3.3 GitLab Flow

GitLab 在 GitHub Flow 的基础上增加了环境分支和 feature flag，兼顾了部署管理和版本控制。

```
main ──●────────●─────────●────────●
         \      / \       / \      /
feature    ●──●   ●─────●   ●──●

environment branches:
production ──●─────────●─────────
             │         │
pre-prod ────●─────────●─────────
```

**核心要素**：
- **环境分支**：`production`、`pre-production`、`staging`，每个环境对应一个分支
- **Feature Flag**：在代码中控制功能开关，允许未完成的功能合并到 main 而不影响生产

**适用场景**：需要复杂环境管理的企业级项目。
**团队规模**：10-100 人。
**优点**：灵活的环境管理和渐进式发布。
**缺点**：需要 Feature Flag 基础设施支持。

### 3.4 Trunk-Based Development（主干开发）

主干开发是 Google、Meta、Netflix 等顶级科技公司采用的高效模型。

```
main ──●──●──●──●──●──●──●──●──●──●
        |  |  |  |  |  |  |  |  |  |
        └── Commits directly to main or short-lived feature branches
```

**核心规则**：
1. 所有开发者直接在 `main`（主干）提交，或创建不超过 2 天的短生命周期分支
2. 频繁提交（每日多次）小粒度的修改
3. 结合 Feature Flag 管理未完成的功能
4. 严格保持主干可部署

**适用场景**：成熟团队、微服务架构、高度自动化的 CI/CD 环境。
**团队规模**：20-100+ 人。
**优点**：减少合并冲突，提升交付速度，适合持续集成/持续部署。
**缺点**：对自动化测试覆盖率和代码质量要求极高，新人适应成本高。

### 3.5 分支策略对比

| 策略 | 复杂度 | 部署频率 | 版本管理 | 学习曲线 | 推荐团队规模 | 适合场景 |
|------|--------|----------|----------|----------|-------------|----------|
| Git Flow | 高 | 低（周/月） | 强 | 陡 | 10-50 人 | 有固定发布周期的产品 |
| GitHub Flow | 低 | 高（天/次） | 弱 | 平 | 1-20 人 | Web 应用、SaaS |
| GitLab Flow | 中 | 中（天/周） | 中 | 中 | 10-100 人 | 企业级多环境部署 |
| Trunk-Based | 低 | 极高（多次/天） | 弱 | 极平 | 20-100+ 人 | CI/CD 成熟团队 |

**选择建议**：
- **初创团队**（1-5 人）：GitHub Flow，够用且简单
- **中型产品团队**（10-30 人）：GitLab Flow 或简化版 Git Flow
- **大型发布型产品**（30+ 人）：Git Flow + Release branch
- **互联网大厂**（持续部署）：Trunk-Based Development

---

## 四、合并策略对比

合并策略是分支管理中最需要团队达成共识的环节。错误的选择会造成历史混乱、冲突频发和难以回退。

### 4.1 Fast-Forward vs 3-Way Merge

**Fast-Forward 合并**（快进合并）：
- 条件：目标分支是当前分支的直接后继，没有分叉
- 结果：线性历史，没有 merge commit
- 命令：`git merge --ff`（默认）

```
Before:  main ●──●──●
             feature ●──●──●
After:   main ●──●──●──●──●──●  (直接前移指针)
```

**3-Way Merge**（三方合并）：
- 条件：两个分支分叉，需要基于共同祖先做三方合并
- 结果：产生 merge commit（有两个 parent）
- 命令：`git merge --no-ff`

```
Before:  main ●──●──●───────────
             feature ●──●──●
After:   main ●──●──●──●──●──●← merge commit
                                      ↑
                            feature ●──●──●
```

**实践策略**：`git merge --no-ff` 在合并 feature 分支时强制创建 merge commit，虽然破坏了线性历史，但保留了"这是一次分支合并"的上下文信息，便于回溯时理解整体结构变化。

### 4.2 Rebase vs Merge vs Squash

这是 Git 工作流中最常见的三种策略选择：

| 策略 | 历史是否重写 | 是否保留分叉信息 | 合并后历史形态 | 推荐场景 |
|------|-------------|-----------------|--------------|----------|
| **Merge** | 否 | 是 | 分叉+merge commit | 公共分支合并（如 feature→develop） |
| **Rebase** | 是 | 否 | 线性、整洁 | 个人分支同步上游变更 |
| **Squash Merge** | 是 | 否 | 压缩为单个 commit | 将多个 WIP 提交变为一个完整功能提交 |

**Merge（当前分支合并到目标分支）**：

```bash
git checkout develop
git merge --no-ff feature/login
```

保留完整分支拓扑，适合公共分支。

**Rebase（变基）**：

```bash
git checkout feature/login
git rebase develop        # 将 feature/login 的提交"移植"到 develop 最新位置
```

```
Before: feature  ●──●──●   (基于较旧的 develop)
         develop ●─●─●─●─● (已前进)
After:  feature           ●──●──●
         develop ●─●─●─●─●
```

Rebase 会让提交历史呈线性，极大提升可读性。但**不要在公共分支上 rebase**——重写已经推送的提交会导致协作成员历史混乱。

**Squash Merge（压缩合并）**：

```bash
git checkout main
git merge --squash feature/login
git commit -m "feat: add login module"
```

将 feature 分支上的所有 commit 压缩为一个新的 commit 追加到 main。适合 PR 合并场景。

### 4.3 冲突解决策略

冲突发生在 Git 无法自动合并时。核心原则：**冷静应对，工具辅助**。

**冲突标记结构**：

```
<<<<<<< HEAD
当前分支的代码
=======
被合并分支的代码
>>>>>>> feature/login
```

**解决步骤**：

1. **可视化工具**：`git mergetool` 或 IDE 内置合并工具（VS Code、IntelliJ）

2. **手动解决策略**：

```
# 1. 接受当前分支版本
git checkout --ours path/to/file

# 2. 接受被合并分支版本
git checkout --theirs path/to/file

# 3. 手动编辑，删除冲突标记后
git add path/to/file
git commit -m "merge: resolve conflict in path/to/file"
```

3. **大型重构冲突预防**：

```bash
# 功能开发前先同步
git checkout feature/large-refactor
git rebase develop   # 在开发过程中频繁 rebase，减小冲突范围
```

4. **`git rerere`**（Reuse Recorded Resolution）：自动记忆并重现相同冲突的解决方案。

```bash
git config --global rerere.enabled true  # 启用后 Git 自动记录冲突解决方式
```

---

## 五、协作工作流

### 5.1 Pull Request / Merge Request 工作流

PR/MR 是现代 Git 协作的核心形式，是代码审查的载体。

**标准流程**：

```
1. 从 main 创建 feature 分支
2. 在 feature 分支上开发并提交
3. git push origin feature/login
4. 在 GitHub/GitLab 上创建 Pull Request
5. 触发 CI 自动运行测试和代码检查
6. 团队成员审查代码，提出评论
7. 根据反馈修改，持续 push 更新
8. 审查通过后合并（Merge/Rebase/Squash）
9. 删除远程 feature 分支
```

**最佳实践**：
- PR 应小而聚焦：单一功能或修复，不超过 400 行变更
- PR 标题遵循 Conventional Commits 规范
- 包含清晰的描述和截图/演示链接
- 尽可能在 24 小时内完成审查

### 5.2 代码审查最佳实践

| 维度 | 审查要点 |
|------|---------|
| **正确性** | 逻辑是否完备？边界条件是否处理？ |
| **安全性** | 是否存在 SQL 注入、XSS、权限泄漏？ |
| **可维护性** | 命名是否清晰？是否有注释？复杂度是否可控？ |
| **测试** | 是否包含单元测试/集成测试？覆盖率达到要求？ |
| **性能** | 是否存在 N+1 查询？是否有不合理的循环嵌套？ |
| **风格** | 是否符合团队代码规范？Lint 是否通过？ |

**审查者心态**：
- 从"作者想做什么"的角度出发，而非"我怎样做得更好"
- 问题用提问代替命令：\`这里用 map 代替 forEach 会不会更清晰？\` 而非\`改成 map\`
- 对事不对人，关注代码本身

### 5.3 Git Hooks

Git Hooks 是挂载在 Git 事件上的脚本，用于自动化验收、检查和格式化。hooks 位于 `.git/hooks/` 目录，需要设置可执行权限。

**pre-commit 钩子**：提交前运行，用于代码格式化和 Lint 检查。

```bash
#!/bin/bash
# .git/hooks/pre-commit

echo "🔍 Running pre-commit checks..."

# 检查是否包含调试代码
if git diff --cached | grep -E "(console\.log|debugger|TODO)" > /dev/null; then
    echo "❌ 提交中包含调试代码 (console.log/debugger/TODO)"
    exit 1
fi

# 运行 ESLint（仅对暂存的文件）
STAGED_FILES=$(git diff --cached --name-only --diff-filter=ACM | grep "\.js$")
if [ -n "$STAGED_FILES" ]; then
    npx eslint $STAGED_FILES
    if [ $? -ne 0 ]; then
        echo "❌ ESLint 检查未通过"
        exit 1
    fi
fi

echo "✅ Pre-commit 检查通过"
```

**commit-msg 钩子**：验证提交消息格式。

```bash
#!/bin/bash
# .git/hooks/commit-msg

COMMIT_MSG=$(cat "$1")
# 检查是否符合 Conventional Commits 规范
if ! echo "$COMMIT_MSG" | grep -qE "^(feat|fix|docs|style|refactor|perf|test|build|ci|chore|revert)(\(.+\))?: .{1,}$"; then
    echo "❌ 提交消息不符合 Conventional Commits 规范"
    echo "   格式: <type>(<scope>): <description>"
    echo "   示例: feat(auth): add OAuth2 login support"
    exit 1
fi
```

**pre-push 钩子**：推送前运行测试。

```bash
#!/bin/bash
# .git/hooks/pre-push

echo "🧪 Running tests before push..."
npm test
if [ $? -ne 0 ]; then
    echo "❌ 测试未通过，推送被阻止"
    exit 1
fi
```

生产团队建议使用 [Husky](https://typicode.github.io/husky/)（前端）或 [pre-commit](https://pre-commit.com/)（通用）管理 hooks，方便版本控制和团队共享。

### 5.4 语义化提交消息（Conventional Commits）

Conventional Commits 规范定义了结构化的提交消息格式，是实现自动化版本发布（semantic-release）、生成 changelog 的基础。

```
<type>(<scope>): <description>

[optional body]

[optional footer(s)]
```

**类型（type）说明**：

| 类型 | 含义 | 是否影响版本号 |
|------|------|--------------|
| `feat` | 新功能 | 次版本号（minor）+1 |
| `fix` | Bug 修复 | 补丁版本号（patch）+1 |
| `BREAKING CHANGE` | 不兼容变更 | 主版本号（major）+1 |
| `docs` | 文档变更 | 不发布 |
| `style` | 代码格式（不影响功能） | 不发布 |
| `refactor` | 重构（既不修复 bug 也不添加功能） | 不发布 |
| `perf` | 性能优化 | 不发布 |
| `test` | 添加或修改测试 | 不发布 |
| `build` | 构建系统或外部依赖变更 | 不发布 |
| `ci` | CI 配置变更 | 不发布 |
| `chore` | 杂项（构建脚本、工具配置等） | 不发布 |
| `revert` | 撤销之前的提交 | 取决于撤销的内容 |

**示例**：

```
feat(auth): add OAuth2 login via Google and GitHub

Implement OAuth2 authorization code flow for social login.
Added new endpoints: GET /auth/google, GET /auth/github

Closes #123
```

```
fix(api): correct null pointer in user profile endpoint

BREAKING CHANGE: The getUserProfile API now returns 404
instead of 200 with null data when user does not exist.
```

### 5.5 签名提交（GPG Signing）

签名提交确保提交者身份的真实性，防止冒充。大型开源项目（Linux 内核、Kubernetes）要求所有 commit 必须签名。

**设置步骤**：

```bash
# 1. 安装 GPG
sudo apt install gpg        # Ubuntu/Debian
brew install gpg            # macOS

# 2. 生成 GPG 密钥
gpg --full-generate-key

# 3. 列出密钥获取 ID
gpg --list-secret-keys --keyid-format LONG
# 输出示例: sec rsa4096/ABC123DEF456G789
# 密钥 ID = ABC123DEF456G789

# 4. 配置 Git 使用 GPG 签名
git config --global user.signingkey ABC123DEF456G789
git config --global commit.gpgsign true    # 自动签名每个提交
git config --global tag.gpgsign true       # 自动签名每个标签

# 5. 导出公钥并添加到 GitHub/GitLab
gpg --armor --export ABC123DEF456G789
```

**验证签名**：

```bash
git log --show-signature       # 显示提交日志中的签名信息
git verify-commit HEAD         # 验证 HEAD 提交的签名
git verify-tag v1.0.0          # 验证标签签名
```

---

## 六、高级主题

### 6.1 Submodules vs Subtrees

当项目需要依赖其他 Git 仓库时，submodule 和 subtree 是两种方案：

| 维度 | Submodules | Subtrees |
|------|-----------|----------|
| 存储方式 | 指针引用（commit hash） | 实际代码副本 |
| 子仓库独立性 | 子仓库保持独立，可自行 pull/push | 代码合并到主仓库，失去独立历史 |
| 主仓库体积 | 轻量（只有引用） | 较大（包含完整代码） |
| 新手友好度 | 低（需要额外操作更新子模块） | 高（透明） |
| 修改子项目 | 直接在子仓库修改并推送 | 需要 subtree push 推回 |
| 版本锁定 | 精确锁定子仓库 commit | 合并时锁定，之后可分离 |

```bash
# Submodule
git submodule add https://github.com/user/lib.git lib/
git submodule update --init --recursive     # 克隆后初始化所有子模块
git submodule update --remote               # 更新子模块到最新 commit

# Subtree
git subtree add --prefix=lib/ https://github.com/user/lib.git main --squash
git subtree pull --prefix=lib/ https://github.com/user/lib.git main --squash
git subtree push --prefix=lib/ https://github.com/user/lib.git main
```

**选择建议**：
- 使用 **submodules** 当：子项目独立开发，变更频繁，需要精确版本控制（如依赖库）
- 使用 **subtrees** 当：需要简化依赖管理，团队规模小，不想增加 Git 操作复杂度

### 6.2 Git Worktree

`git worktree` 允许你在同一仓库的不同目录中同时检出多个分支。这在需要并行处理不同任务时极为高效——无需 stash 或 clone 多个仓库。

```bash
# 在 ../project-hotfix 目录检出一个新的 worktree（hotfix 分支）
git worktree add ../project-hotfix hotfix

# 在 ../project-feature 目录检出一个新分支
git worktree add -b feature/new-dashboard ../project-feature main

# 列出所有 worktree
git worktree list

# 移除 worktree
git worktree remove ../project-hotfix
# 或清理已删除分支的 worktree
git worktree prune
```

**注意**：同一时间同一个分支不能在多个 worktree 中检出一致。

### 6.3 Git Filter-Repo（历史重写）

当需要从 Git 历史中彻底移除敏感信息（密码、密钥）或清理大文件时，`git filter-repo` 是官方推荐的工具（取代 `git filter-branch` 和 `BFG Repo Cleaner`）。

```bash
# 安装
pip install git-filter-repo

# 从整个历史中删除一个文件
git filter-repo --path config/secrets.json --invert-paths

# 从整个历史中删除某个目录
git filter-repo --path docs/old-specs/ --invert-paths

# 替换所有 commit 中的邮箱
git filter-repo --email-callback 'return email.replace(b"@old.com", b"@new.com")'

# 仅保留某个子目录（将子目录提升为根目录）
git filter-repo --path src/mymodule/ --subdirectory-filter src/mymodule/
```

**注意**：`filter-repo` 重写整个仓库历史，所有协作者需要重新 clone。**务必在操作前备份仓库**。

### 6.4 Large File Storage（Git LFS）

Git 本身对大文件处理不佳（二进制文件 diff 无法压缩，导致仓库膨胀）。Git LFS 将大文件替换为文本指针，实际文件存储在远程服务器。

```bash
# 安装
git lfs install

# 跟踪大文件类型
git lfs track "*.psd"
git lfs track "*.zip"
git lfs track "model.pth"

# 上传 LFS 文件
git add .gitattributes
git add model.pth
git commit -m "feat: add trained model"
git push origin main
```

| 对比项 | 普通 Git | Git LFS |
|--------|----------|---------|
| 文件存储 | 直接存于 objects/ | 远程服务器（S3/CDN） |
| 仓库克隆 | 拉取所有版本 | 仅拉取指针文件（约 100 字节） |
| 大文件拉取 | 按需（需全部下载） | 按需（只在 checkout 时下载） |
| 空间占用 | 仓库体积随版本数线性增长 | 仓库轻量，大文件版本在远程 |
| 适用场景 | 代码、文本 | 二进制、媒体、数据集 |

### 6.5 Monorepo vs Multi-repo

| 维度 | Monorepo（单仓库） | Multi-repo（多仓库） |
|------|-------------------|---------------------|
| 定义 | 所有项目代码放在同一个 Git 仓库 | 每个微服务/模块独立一个仓库 |
| 代码共享 | 直接引用，原子化变更 | 通过包管理器发布版本 |
| 原子提交 | ✅ 跨模块变更一次提交 | ❌ 需要协调多个 PR |
| 工具链 | 需要 monorepo 工具（Nx, Turborepo, Bazel） | 标准 Git 操作 |
| CI/CD | 变更检测复杂（需增量分析） | 各服务独立 CI/CD |
| 权限控制 | 粗粒度（所有人访问所有代码） | 细粒度（每个仓库独立权限） |
| 典型代表 | Google、Meta、Uber | Netflix、Amazon、Spotify |
| 仓库规模 | 数十 GB+，数百万 commit | 较小，百 MB 级 |

**选择建议**：
- **Monorepo**：项目间耦合紧密、共享代码多、希望原子化变更的场景
- **Multi-repo**：团队规模大、各服务独立演进、需要独立权限控制的场景

---

## 七、团队规范模板

以下是可以直接复制到团队 Wiki 或 REPO 根目录的规范模板。

### 7.1 提交消息格式模板（CONTRIBUTING.md）

```markdown
# 提交规范

## 格式

```
<type>(<scope>): <description>

[optional body]

[optional footer(s)]
```

## 类型

| 类型 | 用途 |
|------|------|
| feat | 新功能 |
| fix | 修复 Bug |
| docs | 文档变更 |
| style | 代码格式整理 |
| refactor | 重构 |
| perf | 性能优化 |
| test | 添加/修改测试 |
| build | 构建配置变更 |
| ci | CI 配置变更 |
| chore | 其他杂项 |

## 示例

```
feat(auth): implement Google OAuth2 login
fix(api): handle null pointer in GET /users/:id
docs(readme): update installation guide
BREAKING CHANGE: API 响应格式改为 RESTful 规范
```
```

### 7.2 分支命名规范

```
feature/<issue-id>-<short-description>
  → feature/123-user-login

bugfix/<issue-id>-<short-description>
  → bugfix/456-null-pointer-profile

hotfix/<issue-id>-<short-description>
  → hotfix/789-security-patch

release/<version>
  → release/v2.1.0

chore/<description>
  → chore/upgrade-deps
```

### 7.3 Pull Request 模板（.github/PULL_REQUEST_TEMPLATE.md）

```markdown
## 描述

请简要描述此次变更的内容和动机。

Closes #<issue-number>

## 变更类型

- [ ] feat: 新功能
- [ ] fix: Bug 修复
- [ ] refactor: 重构
- [ ] docs: 文档
- [ ] test: 测试
- [ ] chore: 其他

## 自测清单

- [ ] 代码通过了 lint（`npm run lint`）
- [ ] 测试覆盖率未下降
- [ ] 自测通过（手动或自动化）
- [ ] 文档已更新（如需）

## 截图（可选）

<!-- UI 变更请附截图 -->

## 额外说明

<!-- 如有特殊部署步骤或需要关注的变更点 -->
```

### 7.4 合并策略约定

```
# 团队合并策略

## 分支合并规则

| 源 → 目标 | 策略 | 说明 |
|-----------|------|------|
| feature → develop | Merge（--no-ff） | 保留功能分支拓扑 |
| develop → main | Squash Merge | 将版本发布压缩为一个 commit |
| hotfix → main | Merge（--no-ff） | 保留修复路径 |
| main → develop | Merge（--ff） | 同步主干至开发分支 |

## 注意事项

1. 禁止在公共分支（main、develop）上执行 git rebase
2. 合并前确保 CI 通过
3. 需要至少 1 人 Code Review 才能合并
4. 紧急修复（hotfix）需 2 人 Review 才能合并到 main
```

---

## 总结

版本控制不仅是工具的掌握，更是一套**协作契约**。底层理解 Git 的对象模型和三棵树机制，让你在任何 Git 工具上都能快速上手；核心命令的熟练使用是效率基础；分支策略和合并策略的选择决定了团队的协作效率；工作流规范则让这些策略落地为日常习惯。

**三条黄金法则**：
1. **已推送的提交永不 reset，用 revert**
2. **公共分支永不 rebase**
3. **提交消息是写给未来的自己看的——认真写**

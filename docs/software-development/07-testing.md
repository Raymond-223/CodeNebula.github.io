# 测试与质量保障

> 软件测试不是发现 Bug 的手段，而是建立信心的过程。没有测试的代码不是重构，是重写。

---

## 一、测试金字塔

### 1.1 经典金字塔模型

测试金字塔由 Mike Cohn 在《Succeeding with Agile》中提出，定义了三个层级的比例关系：

```
        ┌──────┐
       /│ E2E  │\       少量端到端测试
      / │      │ \
     /├────────┤ \      ──────────────
    / │ 集成测试 │  \   适量集成测试
   /  │        │   \
  /├──────────────┤\   ──────────────
 / │               │ \
/  │   单元测试     │  \  大量单元测试
│  │               │   │
└──┴───────────────┴───┘
```

| 层级 | 比例 | 执行速度 | 定位能力 | 维护成本 |
|------|------|---------|---------|---------|
| 单元测试 | 70% | 毫秒级 | 精确到函数 | 低 |
| 集成测试 | 20% | 秒级 | 模块间 | 中 |
| E2E 测试 | 10% | 分钟级 | 全链路 | 高 |

**核心原则**：下层覆盖足够充分，上层只需验证关键路径。

### 1.2 反模式

**冰激凌甜筒（Ice Cream Cone）**：大量 E2E 测试，少量单元测试。常见于缺乏设计、依赖 Selenium 做全量回归的项目。症状：CI 跑一次要数小时，测试不稳定（flaky），失败后难以定位。

**倒金字塔（Inverted Pyramid）**：集成测试过多，单元测试不足。虽然比冰激凌甜筒好一些，但依然导致测试套件脆弱——任何底层改动都会引发大量失败。

### 1.3 测试奖杯（Test Trophy）

Kent C. Dodds 针对前端应用提出测试奖杯模型：

```
      ┌──────────┐
      │  E2E  │  少量
      ├──────────┤
     /│ 集成测试  │\ 重点
    / │(组件/服务)│ \
   ├──────────────┤
   │   静态分析    │  大量（TypeScript/ESLint）
   ├──────────────┤
   │   单元测试    │  适量（纯逻辑、工具函数）
   └──────────────┘
```

核心观点：**前端测试的重点应该是集成测试**，因为前端最常出 Bug 的地方不在于某个纯函数的逻辑，而在于**组件之间的交互和状态同步**。静态分析（TypeScript、ESLint）能拦截大量低级错误，性价比极高。

---

## 二、单元测试

### 2.1 测什么

单元测试测试的是**最小可测试单元**的行为：

- **纯函数**：给定输入，断言输出。无副作用，最易测试。
- **边界条件**：空值、越界、特殊字符、并发竞争。
- **错误路径**：异常抛出、错误码返回、降级逻辑。
- **业务规则**：折扣计算、权限校验、状态转换。

**不测**：框架行为（Django ORM、React 渲染）、第三方 SDK、配置加载——这些交给集成测试。

### 2.2 pytest（Python）

#### Fixture

```python
import pytest
from myapp.models import Order

@pytest.fixture
def sample_order():
    """创建测试用的订单对象"""
    return Order(user_id=1, total=100.0, items=[{"sku": "A", "qty": 2}])

@pytest.fixture
def db_session(tmp_path):
    """使用内存数据库，测试间隔离"""
    from myapp.database import create_test_db
    session = create_test_db(tmp_path)
    yield session
    session.rollback()

def test_order_discount(sample_order, db_session):
    db_session.add(sample_order)
    result = sample_order.apply_discount(code="NEW_USER10")
    assert result.discounted_total == 90.0
    assert result.discount_applied == 10.0
```

#### Parametrize

```python
@pytest.mark.parametrize("input,expected", [
    ("", 0),
    ("42", 42),
    ("-5", -5),
    ("3.14", 3.14),
    ("not_a_number", None),
])
def test_safe_parse(input, expected):
    from myapp.utils import safe_parse
    assert safe_parse(input) == expected
```

#### Mock

```python
def test_send_welcome_email(mocker):
    mock_smtp = mocker.patch("myapp.services.smtplib.SMTP")
    from myapp.services import send_email
    send_email("user@example.com", "Welcome!")
    mock_smtp.return_value.__enter__.return_value.sendmail.assert_called_once()
    mock_smtp.return_value.__enter__.return_value.quit.assert_called_once()
```

#### Conftest

`conftest.py` 可以跨模块共享 fixture，是 pytest 组织测试的核心机制。将 `db_session`、`client`、`auth_headers` 等高频 fixture 放在 conftest 中，避免重复定义。

### 2.3 Jest（JavaScript/TypeScript）

```typescript
// math.test.ts
import { calculateTax, formatPrice } from "./math";

describe("calculateTax", () => {
  it("计算含税价 - 标准税率", () => {
    expect(calculateTax(100)).toBe(113);
  });

  it("计算含税价 - 零税率", () => {
    expect(calculateTax(100, 0)).toBe(100);
  });

  it("边界 - 负值返回 0", () => {
    expect(calculateTax(-10)).toBe(0);
  });
});

describe("formatPrice", () => {
  it("格式化价格带货币符号", () => {
    const result = formatPrice(1234.5, "USD");
    expect(result).toBe("$1,234.50");
  });
});
```

#### Mock 类型

```typescript
// Stub — 返回固定值
jest.spyOn(paymentApi, "getRate").mockReturnValue(7.2);

// Spy — 记录调用
const logger = jest.spyOn(console, "log");
// ... 执行操作
expect(logger).toHaveBeenCalledWith("Payment completed");

// Mock 模块
jest.mock("../services/payment", () => ({
  charge: jest.fn().mockResolvedValue({ status: "success" }),
}));
```

#### Snapshot 测试

```typescript
it("渲染用户卡片", () => {
  const tree = renderer.create(<UserCard user={mockUser} />).toJSON();
  expect(tree).toMatchSnapshot();
});
```

Snapshot 适合 UI 防回归。更新时运行 `jest --updateSnapshot`，但要仔细审查 diff，避免盲目接受。

### 2.4 Mock、Stub、Fake、Spy、Dummy 辨析

| 术语 | 定义 | 典型场景 |
|------|------|---------|
| **Dummy** | 传递但从不使用的对象 | 填补参数列表 |
| **Stub** | 返回预设值 | 替代外部 API 调用 |
| **Spy** | 记录调用信息 | 验证日志是否写入 |
| **Mock** | 预设期望并验证交互 | 确保支付接口被调用一次 |
| **Fake** | 轻量级实现 | 内存数据库替代真实 DB |

**原则**：能用 Stub 不用 Mock。过度 Mock 导致测试与实现耦合——重构实现时测试不必要地失败。

### 2.5 测试驱动开发（TDD）

**红-绿-重构（Red-Green-Refactor）**：

1. **红**：先写一个会失败的测试，定义期望行为。
2. **绿**：用最简实现让测试通过（可以 hardcode）。
3. **重构**：在测试保护下优化代码质量。

```python
# Step 1: RED — 测试先写
def test_is_prime():
    assert is_prime(2) is True
    assert is_prime(4) is False

# Step 2: GREEN — 最简实现
def is_prime(n: int) -> bool:
    return n == 2  # 暂时只通过一个 case

# Step 3: REFACTOR — 正确实现
def is_prime(n: int) -> bool:
    if n < 2:
        return False
    for i in range(2, int(n ** 0.5) + 1):
        if n % i == 0:
            return False
    return True
```

**何时用 TDD**：
- 业务逻辑复杂（权限、计算、状态机）
- 需求明确且稳定
- 修复 Bug（先写复现测试）

**何时不用**：
- UI 原型阶段（需求频繁变更）
- 探索性研究（算法或架构尚不确定）

### 2.6 代码覆盖率

| 指标 | 含义 | 工具 |
|------|------|------|
| 行覆盖率 | 执行过的代码行数占比 | coverage.py, istanbul |
| 分支覆盖率 | `if/else` 所有分支是否覆盖 | pytest-cov, c8 |
| 函数覆盖率 | 被调用过的函数占比 | 同上 |
| 变异测试 | 修改代码逻辑，看测试能否捕获 | mutmut, Stryker |

```bash
# Python 覆盖率阈值
# pytest.ini
[pytest]
addopts = --cov=myapp --cov-report=term-missing --cov-fail-under=80

# 包含分支覆盖
addopts = --cov-branch --cov-fail-under=75
```

**阈值建议**：
- 新项目：行覆盖率 ≥ 80%，分支覆盖率 ≥ 70%
- 核心业务模块：行覆盖率 ≥ 90%
- 遗留代码：逐步提升，不设硬性门槛

**注意**：100% 覆盖率不代表无 Bug——没有断言的测试、遗漏的边界场景、错误的条件判断都可能在 100% 覆盖下存在。覆盖率是**下限指标**，不是**质量目标**。

---

## 三、集成测试

### 3.1 数据库交互测试

```python
# pytest + SQLAlchemy
@pytest.fixture
def db_session():
    engine = create_engine("sqlite:///:memory:")
    Base.metadata.create_all(engine)
    session = Session(engine)
    yield session
    session.rollback()

def test_create_order_item(db_session):
    order = Order(user_id=1, total=100)
    db_session.add(order)
    db_session.flush()

    item = OrderItem(order_id=order.id, sku="ABC", qty=2)
    db_session.add(item)
    db_session.commit()

    assert db_session.query(OrderItem).count() == 1
    assert order.items[0].sku == "ABC"
```

**回滚策略**：每个测试用例运行在事务中，测试结束后回滚，保证测试间隔离。这种方式比每次重建数据库快 100 倍以上。

### 3.2 API 端点测试

Python（使用 `requests` 和 Flask/FastAPI test client）：

```python
def test_create_user(client):
    response = client.post("/api/users", json={
        "name": "张三",
        "email": "zhangsan@example.com",
    })
    assert response.status_code == 201
    data = response.json()
    assert data["name"] == "张三"
    assert "id" in data

def test_get_nonexistent_user(client):
    response = client.get("/api/users/9999")
    assert response.status_code == 404
    assert response.json()["error"] == "User not found"
```

Node.js（使用 Supertest）：

```typescript
import request from "supertest";
import app from "../app";

describe("POST /api/users", () => {
  it("创建用户成功", async () => {
    const res = await request(app)
      .post("/api/users")
      .send({ name: "李四", email: "lisi@example.com" })
      .expect(201);

    expect(res.body.name).toBe("李四");
  });

  it("邮箱重复返回 409", async () => {
    await request(app)
      .post("/api/users")
      .send({ name: "重复", email: "dup@example.com" })
      .expect(409);
  });
});
```

### 3.3 外部服务模拟

```python
# 使用 WireMock (Java) 或 responses (Python)
import responses

@responses.activate
def test_weather_service():
    responses.add(
        responses.GET,
        "https://api.weather.com/v1/current?city=Beijing",
        json={"temp": 22, "condition": "Sunny"},
        status=200,
    )
    from myapp.services import get_weather
    result = get_weather("Beijing")
    assert result.temperature == 22
```

```python
# 使用 Testcontainers（真实 Docker 容器）
# pip install testcontainers
from testcontainers.postgres import PostgresContainer

def test_with_real_postgres():
    with PostgresContainer("postgres:16-alpine") as postgres:
        conn_url = postgres.get_connection_url()
        # 使用真实 PostgreSQL 运行测试
        # 比 mock 更可靠，但速度较慢
```

Testcontainers 相比 Mock 方案更接近生产环境，适合对数据一致性要求高的场景（如事务隔离级别、索引行为验证）。

### 3.4 异步代码测试

```python
import pytest

@pytest.mark.asyncio
async def test_async_fetch():
    from myapp.services import fetch_data
    result = await fetch_data("https://api.example.com/data")
    assert result["status"] == "ok"

@pytest.mark.asyncio
async def test_concurrent_requests():
    import asyncio
    from myapp.services import fetch_data
    urls = [f"https://api.example.com/page/{i}" for i in range(5)]
    results = await asyncio.gather(*[fetch_data(u) for u in urls])
    assert all(r["status"] == "ok" for r in results)
```

```typescript
// Jest 原生支持 async
it("异步获取用户信息", async () => {
  const user = await userService.getUser(1);
  expect(user.name).toBeDefined();
});

it("并发请求限流", async () => {
  const promises = Array(100)
    .fill(null)
    .map(() => userService.getUser(1));

  const results = await Promise.all(promises);
  // 即使并发 100，服务也不应崩溃
  expect(results.length).toBe(100);
});
```

---

## 四、端到端测试

### 4.1 Playwright

Playwright 是当前最推荐的 E2E 框架，支持多浏览器、多语言，内置自动等待。

```typescript
import { test, expect } from "@playwright/test";

test("用户登录流程", async ({ page }) => {
  await page.goto("https://example.com/login");

  // data-testid 选择器（推荐）
  await page.getByTestId("username").fill("admin");
  await page.getByTestId("password").fill("password123");
  await page.getByTestId("login-btn").click();

  // 等待跳转完成
  await expect(page).toHaveURL(/dashboard/);

  // 断言页面元素
  await expect(page.getByTestId("welcome-msg")).toHaveText("欢迎回来, admin");

  // 截图调试
  await page.screenshot({ path: "login-result.png" });
});
```

**核心特性**：

| 特性 | 用法 |
|------|------|
| Trace Viewer | `playwright show-trace trace.zip` — 回放整个测试过程 |
| Codegen | `npx playwright codegen` — 录制操作生成代码 |
| Auto-wait | 元素可交互前自动等待，无需显式 `sleep` |
| 多浏览器 | Chromium / Firefox / WebKit 同 API |
| API 测试 | `request.post()` 可直接测试接口 |

```bash
# 安装与运行
npm init playwright@latest
npx playwright test --headed    # 可见模式调试
npx playwright test --ui        # UI 模式
npx playwright test --project=chromium  # 指定浏览器
```

### 4.2 Cypress

Cypress 架构与 Playwright 不同——它直接运行在浏览器中（与应用程序同进程）。

```typescript
describe("购物车流程", () => {
  beforeEach(() => {
    cy.visit("/products");
  });

  it("添加商品到购物车", () => {
    cy.get("[data-testid=product-1]").click();
    cy.get("[data-testid=add-to-cart]").click();
    cy.get("[data-testid=cart-count]").should("have.text", "1");
  });

  it("拦截 API 模拟网络", () => {
    cy.intercept("GET", "/api/products", { fixture: "products.json" });
    cy.visit("/products");
    cy.get("[data-testid=product-card]").should("have.length", 10);
  });
});
```

**Cypress vs Playwright**：

| 维度 | Playwright | Cypress |
|------|-----------|---------|
| 浏览器支持 | Chromium/Firefox/WebKit | 仅 Chromium 系 |
| 语言 | TS/JS/Python/Java/.NET | 仅 TS/JS |
| 多标签页 | 原生支持 | 有限支持 |
| 网络拦截 | 完整 API | 内置 `cy.intercept` |
| 组件测试 | 支持 | 支持 (Cypress CT) |
| 调试工具 | Trace Viewer | 时间旅行调试器 |

### 4.3 Selenium

Selenium 仍然在以下场景中有价值：

- **WebDriver 兼容性要求**：需要对接 Cloud 厂商（BrowserStack/SauceLabs）的特定协议
- **Java 技术栈**：Selenium 在 Java 生态中集成最成熟
- **已有大量 Selenium 脚本**：迁移成本高于维护成本

```python
from selenium import webdriver
from selenium.webdriver.common.by import By
from selenium.webdriver.support.ui import WebDriverWait
from selenium.webdriver.support import expected_conditions as EC

driver = webdriver.Chrome()
driver.get("https://example.com/login")
driver.find_element(By.CSS_SELECTOR, "[data-testid=username]").send_keys("admin")
driver.find_element(By.CSS_SELECTOR, "[data-testid=login-btn]").click()
WebDriverWait(driver, 10).until(
    EC.presence_of_element_located((By.CSS_SELECTOR, "[data-testid=dashboard]"))
)
driver.quit()
```

### 4.4 E2E 最佳实践

**1. 使用 `data-testid` 属性**：避免 CSS class 或文本内容变更导致测试失败。

```html
<button data-testid="submit-order" class="btn-primary">提交订单</button>
```

**2. 测试隔离**：每个测试独立创建数据，不依赖其他测试。使用 `beforeEach` 重置状态。

**3. 重试机制**：E2E 测试天然有 flakiness（网络波动、渲染时序），配置自动重试。

```typescript
// playwright.config.ts
export default defineConfig({
  retries: process.env.CI ? 2 : 0,
});
```

**4. 限制 E2E 数量**：只覆盖**核心关键路径**（登录、支付、核心流程），异常路径和边界场景交给下层测试。

---

## 五、行为驱动开发（BDD）

### 5.1 Gherkin 语法

BDD 的核心是用自然语言描述业务行为，Gherkin 是最常用的 DSL：

```gherkin
Feature: 订单折扣
  As a 电商用户
  I want 在符合条件的订单上获得折扣
  So that 我享受优惠

  Scenario: 新用户首次下单享受 10% 折扣
    Given 我是新注册用户
    And 我的购物车中有价值 200 元的商品
    When 我使用折扣码 "NEW_USER10"
    Then 订单总额应为 180 元
    And 折扣金额应显示为 20 元

  Scenario: 折扣码过期
    Given 折扣码 "SUMMER2024" 已过期
    When 我尝试使用该折扣码
    Then 系统提示 "折扣码已失效"
    And 订单总额不变
```

### 5.2 Python Behave 实现

```python
# features/steps/order_steps.py
from behave import given, when, then
from myapp.models import Order, DiscountCode

@given("我是新注册用户")
def step_new_user(context):
    context.user = User(is_new=True)

@given("我的购物车中有价值 {amount} 元的商品")
def step_cart_with_amount(context, amount):
    context.cart = Cart(user=context.user, total=float(amount))

@when('我使用折扣码 "{code}"')
def step_apply_discount(context, code):
    context.order = Order.from_cart(context.cart)
    context.result = context.order.apply_discount(code)

@then("订单总额应为 {expected} 元")
def step_check_total(context, expected):
    assert context.order.total == float(expected), \
        f"Expected {expected}, got {context.order.total}"

@then('系统提示 "{message}"')
def step_check_message(context, message):
    assert context.result.message == message
```

### 5.3 BDD vs TDD

| 维度 | BDD | TDD |
|------|-----|-----|
| **语言** | 自然语言（Gherkin） | 代码（断言） |
| **受众** | 业务人员、QA、开发者 | 主要是开发者 |
| **粒度** | 场景级（用户故事） | 函数级 |
| **驱动方式** | 行为驱动 | 测试驱动 |
| **工具** | Cucumber, Behave, SpecFlow | pytest, Jest, JUnit |
| **适用阶段** | 需求讨论 → 自动化验收 | 开发前 → 单元测试 |

**BDD + TDD 互补**：BDD 用于定义**做什么**（业务层面的验收条件），TDD 用于保障**怎么做**（技术层面的正确性）。实践中，用 BDD 覆盖关键业务流程，用 TDD 覆盖内部实现细节。

---

## 六、静态分析 & Linting

### 6.1 ESLint

```javascript
// eslint.config.js
export default [
  {
    rules: {
      "no-unused-vars": "error",
      "no-console": "warn",
      "eqeqeq": ["error", "always"],
      "complexity": ["warn", { max: 10 }],
    },
  },
];
```

**推荐插件**：

| 插件 | 用途 |
|------|------|
| `@typescript-eslint` | TS 类型相关规则 |
| `eslint-plugin-react` | React Hooks、JSX 规则 |
| `eslint-plugin-import` | 模块导入顺序、未使用导出 |
| `eslint-plugin-security` | 安全敏感模式检测 |

**Pre-commit Hook**：

```bash
# .husky/pre-commit
npx lint-staged
```

```javascript
// package.json
{
  "lint-staged": {
    "*.{ts,tsx}": ["eslint --fix", "prettier --write"],
    "*.py": ["ruff check --fix", "ruff format"]
  }
}
```

### 6.2 Prettier

```json
{
  "semi": true,
  "singleQuote": false,
  "tabWidth": 2,
  "trailingComma": "all",
  "printWidth": 100
}
```

**ESLint + Prettier 集成**：用 `eslint-config-prettier` 关闭 ESLint 中与 Prettier 冲突的规则，用 `eslint-plugin-prettier` 将 Prettier 作为 ESLint 规则运行。推荐方案——直接用 Prettier 做格式化，ESLint 做代码质量检查，互不重叠。

### 6.3 TypeScript 严格模式

```json
{
  "compilerOptions": {
    "strict": true,
    "noUncheckedIndexedAccess": true,
    "noImplicitReturns": true,
    "exactOptionalPropertyTypes": true
  }
}
```

`strict: true` 开启以下子选项：
- `strictNullChecks` — null/undefined 类型安全检查
- `noImplicitAny` — 禁止隐式 any
- `strictFunctionTypes` — 函数类型逆变检查
- `strictBindCallApply` — `bind/call/apply` 类型安全

### 6.4 Python 类型检查

```bash
# mypy
mypy myapp/ --strict --ignore-missing-imports

# pyright (更快)
pyright myapp/
```

```python
# pyproject.toml
[tool.mypy]
strict = true
disallow_untyped_defs = true
warn_return_any = true
warn_unreachable = true
```

### 6.5 高级静态分析

**SonarQube**：持续检查代码异味、安全漏洞、重复代码、复杂度。支持多语言，提供 Quality Gate 门禁。

**CodeQL**：GitHub 出品的语义分析引擎，能发现复杂的安全漏洞（如跨站脚本、SQL 注入、不安全的反序列化）。集成在 GitHub Actions 中自动运行。

```yaml
# .github/workflows/codeql.yml
name: "CodeQL"
on: [push, pull_request]
jobs:
  analyze:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - uses: github/codeql-action/init@v3
      - uses: github/codeql-action/analyze@v3
```

---

## 七、代码审查

### 7.1 审查清单

| 类别 | 检查项 |
|------|--------|
| **正确性** | 逻辑完整吗？边界情况处理了吗？并发安全吗？ |
| **设计** | 职责单一吗？耦合度合理吗？扩展点设计是否过度？ |
| **可读性** | 命名清晰吗？注释解释 Why 而非 What？函数超过 50 行？ |
| **安全性** | 输入验证了吗？SQL 注入/SSTI/XSS？敏感信息未硬编码？ |
| **测试** | 有对应测试吗？测试覆盖了主要路径和边界吗？ |
| **性能** | N+1 查询？不必要的计算？缓存使用合理吗？ |

### 7.2 审查类型

| 类型 | 模式 | 适用场景 |
|------|------|---------|
| **结对编程（Pair）** | 实施时实时审查 | 复杂功能、模块设计 |
| **异步审查（Async）** | PR/MR 评论 | 常规开发、分布式团队 |
| **过肩审查（Over-the-Shoulder）** | 开发者走到你的屏幕前讨论 | 快速反馈、新手指导 |
| **工具辅助审查** | 自动分析 + 人工复审 | 静态分析、覆盖率检查 |

### 7.3 有效反馈

**给予反馈**：

- 用"我"语气而非"你"的指责："这里可能会引发空指针异常" 优于 "你忘了判空"
- 区分阻塞性（Blocking）与非阻塞性（Nitpick）：用标签明确优先级
- 提供改进建议而非只指问题："考虑用 `Optional.ofNullable()` 替代 `if (x != null)`"

**接收反馈**：

- 对事不对人：PR 是代码的评审，不是你个人的评审
- 追问不明之处：如果看不懂反馈，说明代码可能确实不够清晰
- 说"谢谢"：每个认真阅读你代码的同事都值得感谢

### 7.4 PR 大小与节奏

研究（SmartBear, 2020）表明：

- **200 行以内**的 PR 审查质量最高、缺陷发现率最优
- PR 超过 400 行后，缺陷密度呈指数下降
- PR 开放超过**48 小时**会显著延长 Lead Time

**最佳实践**：
- 每个 PR 聚焦一个变更（单一职责）
- 目标：2 小时内开始审查，24 小时内合并
- 大改动用 stacked PR（基链式 PR）拆分为多个小 PR

---

## 八、重构

### 8.1 何时重构

**三次法则（Rule of Three）**：同一段代码修改三次时，就应该考虑重构。

**常见代码坏味（Code Smells）**：

| 坏味 | 症状 | 重构手法 |
|------|------|---------|
| 长方法（Long Method） | 函数超过 20-30 行 | 提取方法（Extract Method） |
| 大类（Large Class） | 类超过 300 行 | 提取类（Extract Class） |
| 功能嫉妒（Feature Envy） | 一个方法频繁访问另一个类的数据 | 移动方法（Move Method） |
| 霰弹式修改（Shotgun Surgery） | 改一个需求需要修改多处代码 | 搬移字段（Move Field），内聚 |
| 过长参数列表（Long Parameter List） | 参数超过 3-4 个 | 引入参数对象 |
| 重复代码（Duplicated Code） | 相同逻辑出现多次 | 提取方法 → 模板方法模式 |
| 过度耦合（Coupling） | 类知道其他类的内部细节 | 迪米特法则（Law of Demeter） |

### 8.2 重构技术

```python
# 示例：提取方法（Extract Method）
# 重构前
def process_order(order):
    total = sum(item.price for item in order.items)
    discount = 0
    if order.user.is_vip:
        discount = total * 0.2
    elif total > 500:
        discount = total * 0.1
    final = total - discount
    # 邮件通知
    subject = "订单确认"
    body = f"您的订单总计 ¥{final}，已优惠 ¥{discount}"
    send_email(order.user.email, subject, body)
    return final

# 重构后
def process_order(order):
    final = calculate_final_total(order)
    send_order_confirmation(order.user, final, order.total - final)
    return final

def calculate_final_total(order):
    total = sum(item.price for item in order.items)
    discount = calculate_discount(order.user, total)
    return total - discount

def calculate_discount(user, total):
    if user.is_vip:
        return total * 0.2
    if total > 500:
        return total * 0.1
    return 0

def send_order_confirmation(user, final_total, discount):
    subject = "订单确认"
    body = f"您的订单总计 ¥{final_total}，已优惠 ¥{discount}"
    send_email(user.email, subject, body)
```

### 8.3 安全重构的前提

**没有测试覆盖，重构是重写**。安全重构的三角约束：

```
         ┌──────────┐
         │  测试覆盖 │
         │ (安全网)  │
         └──────────┘
              ┃
    ┌─────────┨──────────┐
    │         ┃          │
┌───────┐  ┌───────┐  ┌───────┐
│ 小步骤 │  │ 频繁  │  │ 保持  │
│ 逐步修改 │  │ 运行测试 │  │ 功能不变 │
└───────┘  └───────┘  └───────┘
```

### 8.4 遗留代码策略

**表征测试（Characterization Tests）**：在重构前，先运行代码并捕获实际输出作为测试基线。这些测试验证"当前行为"，而非"期望行为"。

**萌芽方法（Sprout Method）**：在遗留类中新增一个 `public` 方法，将新逻辑独立实现，不修改原有代码。

```python
class LegacyPaymentProcessor:
    def process(self, payment):
        # ... 500 行遗留代码，不敢动 ...
        result = self._old_processing(payment)
        # 新增：在尾部插入新逻辑，不影响旧逻辑
        self._audit_log(payment, result)
        return result

    def _audit_log(self, payment, result):
        """新加的审计日志，不修改旧方法内部"""
        log_entry = AuditEntry(payment_id=payment.id, result=result)
        db.session.add(log_entry)
```

**Seam（接缝）**：代码中允许替换行为的位置（如参数、虚函数、依赖注入点）。找到这些位置，测试就能安全地插入。

---

## 九、性能测试

### 9.1 负载测试

**Locust**（Python，最推荐）：

```python
# locustfile.py
from locust import HttpUser, task, between

class WebsiteUser(HttpUser):
    wait_time = between(1, 5)  # 模拟用户思考时间

    @task(3)  # 权重 3
    def view_products(self):
        self.client.get("/api/products")

    @task(1)  # 权重 1
    def create_order(self):
        self.client.post("/api/orders", json={
            "items": [{"sku": "A001", "qty": 1}],
        })
```

```bash
locust -f locustfile.py --headless -u 100 -r 10 --run-time 5m
# --headless: 无 UI
# -u 100: 100 个并发用户
# -r 10: 每秒启动 10 个用户
# --run-time 5m: 运行 5 分钟
```

**k6**（Go 编写，JavaScript 脚本）：

```javascript
import http from "k6/http";
import { check, sleep } from "k6";

export const options = {
  stages: [
    { duration: "2m", target: 50 },   // 爬升到 50 用户
    { duration: "5m", target: 50 },   // 保持
    { duration: "2m", target: 0 },    // 下降
  ],
  thresholds: {
    http_req_duration: ["p(95)<500"],  // 95% 请求 < 500ms
  },
};

export default function () {
  const res = http.get("https://api.example.com/health");
  check(res, { "status is 200": (r) => r.status === 200 });
  sleep(1);
}
```

### 9.2 压力测试 vs 尖峰测试 vs 浸泡测试

| 类型 | 目的 | 做法 | 指标 |
|------|------|------|------|
| **负载测试** | 验证正常负载下的性能 | 目标负载，保持 | 响应时间、吞吐量 |
| **压力测试** | 找到系统极限 | 逐步增加负载直到崩溃 | 最大并发、崩溃点 |
| **尖峰测试** | 模拟突发的流量暴增 | 短时间内骤增用户数 | 自动扩缩、恢复速度 |
| **浸泡测试** | 发现内存泄漏、资源耗尽 | 持续运行 24-72 小时 | 内存增长、GC 频率 |

### 9.3 性能剖析

**Python（cProfile）**：

```bash
python -m cProfile -o profile.prof myapp.py
snakeviz profile.prof  # 可视化火焰图
```

**前端（Chrome DevTools）**：

- **Performance 面板**：录制交互，分析 FPS、CPU、内存
- **Lighthouse**：自动化性能审计（FCP、LCP、CLS、TTI）
- **React DevTools Profiler**：组件渲染次数、耗时、触发原因

**关键前端性能指标**（Core Web Vitals）：

| 指标 | 含义 | 目标 |
|------|------|------|
| LCP | 最大内容绘制 | ≤ 2.5s |
| FID/INP | 首次输入延迟 / 交互到下次绘制 | ≤ 200ms (INP) |
| CLS | 累计布局偏移 | ≤ 0.1 |

---

## 十、软件质量度量

### 10.1 DORA 指标

由 Google DORA（DevOps Research and Assessment）研究提出的四个关键指标，预测 IT 组织绩效。

| 指标 | 定义 | 精英表现 | 高表现 | 中表现 | 低表现 |
|------|------|---------|-------|-------|-------|
| **部署频率** | 多久部署一次到生产 | 每日多次 | 每周一次 | 每月一次 | 每半年一次 |
| **变更前置时间** | 从代码提交到成功运行 | < 1 小时 | 1 天 | 1 周 | 1 月 |
| **变更失败率** | 部署导致故障的比例 | < 5% | < 10% | < 15% | > 30% |
| **服务恢复时间** | 从故障中恢复的时间 | < 1 小时 | < 1 天 | < 1 周 | > 1 月 |

**关键洞察**：**部署频率和前置时间与变更失败率并不呈反比**——精英团队既能更频繁地部署，出现故障也更少。这说明自动化质量门禁和持续改进是有效的。

### 10.2 代码质量指标

**圈复杂度（Cyclomatic Complexity）**：

```
M = E - N + 2P
```

- `E`：图中边的数量
- `N`：节点数量
- `P`：连接组件的数量（通常为 1）

```python
def handle_request(status_code, user_role, is_authenticated):
    # M = 1 （线性，无分支）
    if is_authenticated:                    # +1
        if user_role == "admin":            # +1
            return "full_access"
        elif user_role == "editor":         # +1
            if status_code == 200:          # +1
                return "editor_access"
            return "read_only"
        return "limited_access"
    return "redirect_login"
# 圈复杂度 = 1 + 4 (if/elif) = 5
```

**阈值**：

| 等级 | 范围 | 含义 |
|------|------|------|
| 🟢 良好 | 1-10 | 简单函数，易于理解测试 |
| 🟡 警告 | 11-20 | 较复杂，需要关注 |
| 🟠 次优 | 21-50 | 太复杂，应当拆分 |
| 🔴 危险 | > 50 | 不可测试，必须重构 |

**认知复杂度**：比圈复杂度更能反映人类理解代码的难度。它不计算 `if` 嵌套的指数级路径，而是计算**嵌套深度**导致的认知负担。

**内聚与耦合**：

```
高内聚 + 低耦合 = 可维护代码
高内聚 + 高耦合 = 模块强关联但内部一致
低内聚 + 低耦合 = 杂物箱，什么都有一点
低内聚 + 高耦合 = 最差的境地，谁也改不动
```

### 10.3 技术债务管理

**技术债务象限**（Martin Fowler）：

```
                  谨慎的
        ┌────────────────────────┐
        │  故意 / 已知    │  无意 / 已知    │
        │  （策略性负债） │  （需要偿还）    │
        │  例：为上市而  │  例：遗留代码    │
        │  暂缓重构      │  坏味发现后      │
        ├────────────────────────┤
        │  故意 / 未知    │  无意 / 未知    │
        │  （最危险）     │  （潜伏风险）    │
        │  例：删掉测试   │  例：从未被      │
        │  以为没事      │  检查的模块      │
        └────────────────────────┘
                  鲁莽的
```

**管理策略**：

1. **量化**：用 SonarQube 追踪 Tech Debt Ratio（技术债务比率 = 修复成本 / 重写成本）
2. **留出预算**：每个迭代分配 15-20% 时间偿还技术债务
3. **童子军规则**（Boy Scout Rule）：每次修改代码都让它比之前好一点
4. **债务记录**：用 ADR（Architecture Decision Records）或 Issue 记录技术债务，明确**当前状态、理想状态、预计影响**

---

## 综合测试策略模板

```yaml
# testing-strategy.yaml
project: CodeNebula
version: 1.0

layers:
  unit:
    tools: [pytest, Jest]
    coverage:
      line: ">= 85%"
      branch: ">= 75%"
    target: "src/**/*.py, src/**/*.ts"

  integration:
    tools: [pytest + SQLite, supertest, Testcontainers]
    coverage: "关键 API >= 90%"
    target: "api/, services/, repositories/"

  e2e:
    tools: [Playwright]
    target: "关键用户路径（登录、支付、注册）"
    retries: 2
    max_tests: 20

  static_analysis:
    tools: [ESLint, mypy, SonarQube]
    rules: "errors 必须通过，warnings 48 小时内处理"
    pre_commit: true

  performance:
    tools: [Locust, Lighthouse]
    frequency: "每次发布前 + 每周回归"
    thresholds:
      p95_response_time: 500ms
      lcp: 2.5s
      cls: 0.1

review:
  max_lines_per_pr: 400
  response_time: "< 2 hours"
  checklist: "使用 PR 模板自动生成"

metrics:
  dora:
    deployment_frequency: "每周 ≥ 1"
    lead_time: "< 1 day"
    change_failure_rate: "< 10%"
    mtbf: "持续追踪"
```

---

## 扩展阅读

- 《Working Effectively with Legacy Code》— Michael Feathers（遗留代码圣经）
- 《Refactoring: Improving the Design of Existing Code》— Martin Fowler（重构必读）
- 《xUnit Test Patterns》— Gerard Meszaros（测试模式大全）
- 《Accelerate》— Nicole Forsgren 等（DORA 指标原著）
- Google Testing Blog — testing.googleblog.com
- Playwright 官方文档 — playwright.dev/docs

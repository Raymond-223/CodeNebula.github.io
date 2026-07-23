# DevOps 与部署

> DevOps 不只是工具链，更是一种文化变革——打破开发与运维之间的壁垒，让软件从提交到生产的过程变得快速、可靠、可重复。本文从文化理念到工程实践，覆盖容器化、编排、CI/CD、云服务、监控等核心领域。

---

## 一、DevOps 文化

### 1.1 什么是 DevOps

DevOps 是 Development（开发）与 Operations（运维）的融合，核心目标是通过**文化、自动化、精益、测量和共享**（CALMS）来缩短交付周期、提高部署频率、保证系统可靠性。

**CALMS 框架**：

| 维度 | 英文 | 含义 | 实践 |
|------|------|------|------|
| **文化** | Culture | 协作而非对立，共享目标与责任 | 打破 Silos，开发与运维共同 On-Call |
| **自动化** | Automation | 一切重复性工作由工具完成 | CI/CD、基础设施即代码、自动化测试 |
| **精益** | Lean | 消除浪费，专注价值流 | 限制 WIP、小批量交付、持续改进 |
| **测量** | Measurement | 用数据驱动决策 | DORA 指标、监控仪表盘、SLO 追踪 |
| **共享** | Sharing | 知识、工具、责任的共享 | 共享 Runbook、事后复盘、内部分享 |

> **核心转变**：传统模式中，开发"扔过墙"把代码交给运维；DevOps 模式下，开发参与部署和运维，运维参与设计和测试。

### 1.2 DevOps vs 传统运维

```
传统模式：
开发: 写代码 → 构建 → 交给运维
运维: 收到包 → 手动配置 → 部署 → 祈祷别出事
                                    ↑
                          沟通效率低、部署慢、易出错

DevOps 模式：
开发 + 运维 (同一团队)
  ├── 代码提交 → 自动构建 → 自动测试 → 自动部署
  ├── 基础设施代码化 (IaC)
  ├── 监控告警 → 自动修复 / 快速回滚
  └── 事后复盘 → 持续改进
```

| 维度 | 传统运维 | DevOps |
|------|---------|--------|
| **部署频率** | 每月/每季度一次 | 每天多次甚至每次提交 |
| **团队关系** | 开发与运维分离，互相指责 | 同一团队或紧密协作 |
| **部署方式** | 手动操作，变更管理工单 | 自动化流水线，自助部署 |
| **故障处理** | 开发帮查日志，运维重启 | 开发 On-Call，可自愈系统 |
| **基础设施** | 物理服务器，手动配置 | 云资源，基础设施即代码 |
| **文档** | 静态 Word 文档，过期快 | Runbook + 代码注释，随代码更新 |

### 1.3 DORA 指标

Google DORA 团队通过多年研究，定义了四个关键指标来衡量 DevOps 效能：

| 指标 | 定义 | 高水平（Elite） | 低水平（Low） |
|------|------|----------------|--------------|
| **部署频率** | 多久部署一次代码到生产 | 每天多次 | 每月一次到每季度一次 |
| **变更前置时间** | 从代码提交到成功部署的时间 | 小于1小时 | 1周到1个月 |
| **变更失败率** | 部署导致服务退化或故障的比例 | <5% | 46-60% |
| **故障恢复时间 (MTTR)** | 从服务故障到恢复的时间 | <1小时 | 1周到1个月 |

**如何测量**：

```bash
# 部署频率：统计 git log 中的部署 tag
git log --oneline --since="2024-01-01" --grep="deploy" | wc -l

# 前置时间：commit 到 deploy 的时间差
# 可在 CI/CD 中记录两个时间戳，计算差值
```

> **不要追求指标而忘了目标**：高部署频率不等于高价值交付。指标是改进的工具，不是考核的枷锁。

---

## 二、容器化：Docker

### 2.1 Image vs Container

```
┌──────────────────────────────────────────────┐
│                    Docker                     │
│                                               │
│  Image (镜像)         Container (容器)          │
│  ┌──────────┐        ┌──────────┐             │
│  │ 只读模板  │        │ Image 的 │             │
│  │ OS + 环境 │ ──运行→│ 运行实例  │             │
│  │ + 代码    │        │ + 可写层  │             │
│  └──────────┘        └──────────┘             │
│                                               │
│ 类比：Class/对象     类比：进程实例              │
└──────────────────────────────────────────────┘
```

- **Image**：不可变的只读模板，包含操作系统层、依赖、代码和配置
- **Container**：Image 的运行实例，拥有自己的文件系统、网络和进程空间，上层有可写层

### 2.2 Dockerfile 编写

**多阶段构建（Multi-stage Build）**：在一个 Dockerfile 中使用多个 FROM 语句，构建阶段包含编译工具，运行阶段只保留产物。

```dockerfile
# ===== 构建阶段 =====
FROM golang:1.22-alpine AS builder
WORKDIR /app
COPY go.mod go.sum ./
RUN go mod download
COPY . .
RUN CGO_ENABLED=0 GOOS=linux go build -o /app/server .

# ===== 运行阶段 =====
FROM alpine:3.19
RUN apk --no-cache add ca-certificates tzdata
COPY --from=builder /app/server /server
COPY --from=builder /app/config /config
EXPOSE 8080
USER 1001
ENTRYPOINT ["/server"]
```

**最佳实践**：

```dockerfile
# 1. 减少层数：合并 RUN 命令
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        curl \
        git \
        ca-certificates && \
    rm -rf /var/lib/apt/lists/*

# 2. .dockerignore 排除无关文件
# .dockerignore 内容：
# node_modules
# .git
# .env
# *.md
# Dockerfile

# 3. 使用非 root 用户运行
RUN groupadd -r app && useradd -r -g app app
USER app

# 4. 指定精确的基础镜像版本，不用 latest
FROM python:3.12-slim-bookworm

# 5. 利用构建缓存：不常变动的先 COPY
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt
COPY . .
```

### 2.3 Docker Compose

Docker Compose 用 YAML 定义多容器应用，适合开发环境和中小型部署。

```yaml
# docker-compose.yml
services:
  app:
    build: .
    ports:
      - "8080:8080"
    environment:
      - DB_HOST=db
      - REDIS_HOST=redis
      - APP_ENV=production
    depends_on:
      db:
        condition: service_healthy
      redis:
        condition: service_started
    volumes:
      - app_data:/app/data
    restart: unless-stopped
    healthcheck:
      test: ["CMD", "curl", "-f", "http://localhost:8080/health"]
      interval: 30s
      timeout: 10s
      retries: 3

  db:
    image: postgres:16-alpine
    environment:
      POSTGRES_DB: myapp
      POSTGRES_USER: app
      POSTGRES_PASSWORD: ${DB_PASSWORD}
    volumes:
      - pg_data:/var/lib/postgresql/data
    healthcheck:
      test: ["CMD-SHELL", "pg_isready -U app"]
      interval: 5s

  redis:
    image: redis:7-alpine
    volumes:
      - redis_data:/data
    command: redis-server --appendonly yes

  nginx:
    image: nginx:alpine
    ports:
      - "80:80"
      - "443:443"
    volumes:
      - ./nginx.conf:/etc/nginx/conf.d/default.conf
    depends_on:
      - app

volumes:
  app_data:
  pg_data:
  redis_data:

networks:
  default:
    driver: bridge
```

**常用命令**：

```bash
# 启动所有服务（后台）
docker compose up -d

# 查看日志
docker compose logs -f app

# 重新构建并启动
docker compose up -d --build

# 查看服务状态
docker compose ps

# 停止并删除
docker compose down -v  # -v 会删除卷
```

### 2.4 镜像仓库

| 仓库 | 类型 | 特点 | 适用场景 |
|------|------|------|---------|
| **Docker Hub** | 公共/私有 | 最大生态，免费公共仓库 | 开源项目、个人测试 |
| **Harbor** | 私有（开源） | 安全扫描、RBAC、复制策略、漏洞扫描 | 企业级私有仓库 |
| **AWS ECR** | 云托管 | 与 IAM 集成，自动镜像扫描 | AWS 原生部署 |
| **GitHub Container Registry** | 云托管 | 与 GitHub Actions 集成 | GitHub 生态 |
| **GitLab Container Registry** | 云托管/自托管 | 与 GitLab CI 集成 | GitLab 生态 |

**拉取与推送**：

```bash
# 登录
docker login registry.example.com

# 打标签
docker tag my-app:latest registry.example.com/my-app:v1.2.3

# 推送
docker push registry.example.com/my-app:v1.2.3

# 拉取
docker pull registry.example.com/my-app:v1.2.3
```

### 2.5 安全实践

```bash
# 1. Rootless 模式运行 Docker 守护进程
dockerd-rootless-setuptool.sh install

# 2. 运行时限制权限
docker run --security-opt seccomp=seccomp-profile.json \
           --cap-drop=ALL \
           --cap-add=NET_BIND_SERVICE \
           --read-only \
           my-app

# 3. 镜像漏洞扫描
docker scout cves my-app:latest
trivy image my-app:latest

# 4. 不要将敏感信息写入镜像
# ❌ 错误：密码写在 Dockerfile
ENV DB_PASSWORD=supersecret

# ✅ 正确：运行时注入
docker run -e DB_PASSWORD=$DB_PASSWORD my-app
# 或使用 Docker Secrets（Swarm 模式）
```

---

## 三、容器编排：Kubernetes

### 3.1 核心概念

```
┌───────────────────────────────────────────────────┐
│                   Kubernetes Cluster              │
│                                                   │
│  ┌──── Control Plane ────┐                        │
│  │  API Server  etcd     │                        │
│  │  Scheduler  Controller│                        │
│  └───────────────────────┘                        │
│                                                   │
│  ┌──── Node 1 ────┐    ┌──── Node 2 ────┐        │
│  │  ┌─Pod─┐ ┌─Pod─┐│    │  ┌─Pod─┐ ┌─Pod─┐│       │
│  │  │Ctr 1│ │Ctr 1││    │  │Ctr 1│ │Ctr 1││       │
│  │  └─────┘ └─────┘│    │  └─────┘ └─────┘│       │
│  │  ┌─Pod───┐       │    │  ┌─Pod───┐      │       │
│  │  │Ctr 1  │       │    │  │Ctr 1  │      │       │
│  │  │Ctr 2  │       │    │  │Ctr 2  │      │       │
│  │  └───────┘       │    │  └───────┘      │       │
│  └──────────────────┘    └──────────────────┘       │
└───────────────────────────────────────────────────┘
```

| 资源 | 作用 | 关键点 |
|------|------|--------|
| **Pod** | 最小的部署单元，一个或多个容器共享网络和存储 | Pod 是"逻辑主机" |
| **Deployment** | 管理 Pod 副本数和滚动更新 | 无状态应用的标准方式 |
| **Service** | 提供稳定的网络端点访问一组 Pod | 解耦 Pod 变化与客户端 |
| **ConfigMap** | 将配置与镜像分离 | 存储非敏感配置 |
| **Secret** | 存储敏感数据（base64 编码 + 加密） | 密码、Token、证书 |
| **Ingress** | 七层负载均衡，HTTP/HTTPS 路由 | 域名 + 路径转发 |
| **PersistentVolume (PV) / PersistentVolumeClaim (PVC)** | 持久化存储供给与消费 | 数据库等有状态应用 |
| **Namespace** | 逻辑隔离不同环境或团队 | 多租户管理 |

### 3.2 kubectl 常用命令

```bash
# Pod 操作
kubectl get pods -n my-namespace
kubectl describe pod my-pod
kubectl logs -f deployment/my-app
kubectl exec -it my-pod -- /bin/sh

# Deployment 操作
kubectl get deployments
kubectl scale deployment my-app --replicas=5
kubectl rollout status deployment/my-app
kubectl rollout undo deployment/my-app

# Service 操作
kubectl get svc
kubectl port-forward svc/my-app 8080:80

# 调试与排查
kubectl top nodes                    # 节点资源使用
kubectl top pods                     # Pod 资源使用
kubectl get events --sort-by='.lastTimestamp'
kubectl auth can-i create deployments  # RBAC 检查

# 上下文切换
kubectl config get-contexts
kubectl config use-context my-cluster
```

### 3.3 Service 类型

```yaml
# ClusterIP：集群内部访问（默认）
apiVersion: v1
kind: Service
metadata:
  name: my-app-svc
spec:
  type: ClusterIP
  selector:
    app: my-app
  ports:
    - port: 80
      targetPort: 8080
---
# NodePort：外部可通过节点 IP + 端口访问
apiVersion: v1
kind: Service
metadata:
  name: my-app-nodeport
spec:
  type: NodePort
  selector:
    app: my-app
  ports:
    - port: 80
      targetPort: 8080
      nodePort: 30080
---
# LoadBalancer：云厂商提供外部负载均衡器
apiVersion: v1
kind: Service
metadata:
  name: my-app-lb
spec:
  type: LoadBalancer
  selector:
    app: my-app
  ports:
    - port: 80
      targetPort: 8080
---
# ExternalName：映射到外部 DNS 名称
apiVersion: v1
kind: Service
metadata:
  name: external-db
spec:
  type: ExternalName
  externalName: mydb.example.com
```

### 3.4 滚动更新与回滚

```yaml
# Deployment 配置更新策略
apiVersion: apps/v1
kind: Deployment
metadata:
  name: my-app
spec:
  replicas: 3
  strategy:
    type: RollingUpdate
    rollingUpdate:
      maxSurge: 1        # 最多可以多启动几个 Pod
      maxUnavailable: 0  # 最多可以有多少 Pod 不可用
  selector:
    matchLabels:
      app: my-app
  template:
    metadata:
      labels:
        app: my-app
    spec:
      containers:
        - name: app
          image: registry.example.com/my-app:v1.2.3
          readinessProbe:
            httpGet:
              path: /health
              port: 8080
            initialDelaySeconds: 5
            periodSeconds: 10
          livenessProbe:
            httpGet:
              path: /health
              port: 8080
            initialDelaySeconds: 15
            periodSeconds: 20
```

```bash
# 更新镜像
kubectl set image deployment/my-app app=registry.example.com/my-app:v1.2.4

# 查看更新状态
kubectl rollout status deployment/my-app

# 回滚到上一个版本
kubectl rollout undo deployment/my-app

# 回滚到指定版本
kubectl rollout undo deployment/my-app --to-revision=3

# 查看历史版本
kubectl rollout history deployment/my-app
```

### 3.5 Helm Charts

Helm 是 Kubernetes 的包管理器，将 K8s YAML 打包为可复用、可配置的 Chart。

**Chart 目录结构**：

```
my-app/
├── Chart.yaml          # 元数据：名称、版本、依赖
├── values.yaml         # 默认配置值
├── values-production.yaml  # 不同环境的覆盖
├── templates/
│   ├── deployment.yaml
│   ├── service.yaml
│   ├── ingress.yaml
│   ├── configmap.yaml
│   ├── _helpers.tpl    # 可复用模板函数
│   └── NOTES.txt       # 安装后提示信息
└── charts/             # 子 Chart（依赖）
```

**模板示例**：

```yaml
# templates/deployment.yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: {{ include "my-app.fullname" . }}
  labels:
    {{- include "my-app.labels" . | nindent 4 }}
spec:
  replicas: {{ .Values.replicaCount }}
  strategy:
    type: {{ .Values.strategy.type }}
  selector:
    matchLabels:
      {{- include "my-app.selectorLabels" . | nindent 6 }}
  template:
    metadata:
      labels:
        {{- include "my-app.selectorLabels" . | nindent 8 }}
    spec:
      containers:
        - name: {{ .Chart.Name }}
          image: "{{ .Values.image.repository }}:{{ .Values.image.tag | default .Chart.AppVersion }}"
          ports:
            - containerPort: {{ .Values.service.port }}
          env:
            - name: DB_HOST
              value: {{ .Values.db.host }}
            - name: DB_PORT
              value: "{{ .Values.db.port }}"
          resources:
            {{- toYaml .Values.resources | nindent 12 }}
```

```yaml
# values.yaml
replicaCount: 3
strategy:
  type: RollingUpdate
image:
  repository: registry.example.com/my-app
  tag: ""
service:
  type: ClusterIP
  port: 8080
db:
  host: postgres-cluster
  port: 5432
resources:
  requests:
    cpu: 100m
    memory: 128Mi
  limits:
    cpu: 500m
    memory: 512Mi
ingress:
  enabled: false
```

**常用命令**：

```bash
# 安装 Chart
helm install my-app ./my-app -f values-production.yaml

# 升级
helm upgrade my-app ./my-app --set image.tag=v1.2.4

# 回滚
helm rollback my-app 2

# 查看历史
helm history my-app

# 模板渲染预览（不安装）
helm template ./my-app --debug
```

### 3.6 可观测性

| 维度 | 工具 | 作用 |
|------|------|------|
| **资源指标** | Metrics Server + Prometheus | CPU、内存、网络、存储 |
| **日志** | Loki / ELK + Fluentd/Fluent Bit | 集中日志采集与查询 |
| **链路追踪** | Jaeger / OpenTelemetry | 请求在服务间的完整路径 |
| **事件** | Kubernetes Events | Pod 调度、拉取失败等事件 |

```bash
# 部署 Metrics Server（必须，否则 kubectl top 不可用）
kubectl apply -f https://github.com/kubernetes-sigs/metrics-server/releases/latest/download/components.yaml

# 使用 Prometheus Operator 一键部署监控栈
helm repo add prometheus-community https://prometheus-community.github.io/helm-charts
helm install prometheus prometheus-community/kube-prometheus-stack
```

---

## 四、CI/CD 流水线

### 4.1 CI 与 CD 的区别

```
代码提交
  │
  ▼
┌─────────────────────────────────────────────────────┐
│  CI (持续集成)                                       │
│  ├── 代码检查（Lint）                                 │
│  ├── 单元测试                                        │
│  ├── 构建（编译/打包）                                │
│  ├── 镜像构建                                        │
│  └── 推送镜像到仓库                                   │
└─────────────────────────────────────────────────────┘
  │
  ▼（触发部署）
┌─────────────────────────────────────────────────────┐
│  CD (持续交付/持续部署)                              │
│  ├── 部署到测试环境 → 集成测试 → 验收测试              │
│  ├── 部署到预发布环境 → 冒烟测试                       │
│  └── 部署到生产环境（持续部署是自动的，持续交付需手动审批） │
└─────────────────────────────────────────────────────┘
```

| 概念 | 含义 | 人工介入 |
|------|------|---------|
| **持续集成 (CI)** | 频繁将代码合并到主干，每次合并自动构建和测试 | 无 |
| **持续交付 (CD)** | CI + 自动部署到类生产环境，但部署到生产需人工审批 | 生产部署需审批 |
| **持续部署 (CD)** | CI + 完全自动化部署到生产环境 | 无 |

### 4.2 GitHub Actions

**核心概念**：

```
Workflow (.github/workflows/*.yml)
  └── Job (运行在同一个 Runner 上，共享文件系统)
       ├── Step 1: actions/checkout@v4
       ├── Step 2: 运行脚本
       └── Step 3: actions/upload-artifact@v4
```

**完整示例**：

```yaml
# .github/workflows/deploy.yml
name: CI/CD Pipeline

on:
  push:
    branches: [main]
    paths-ignore:
      - 'README.md'
      - 'docs/**'
  pull_request:
    branches: [main]

env:
  REGISTRY: ghcr.io
  IMAGE_NAME: ${{ github.repository }}

jobs:
  lint-and-test:
    runs-on: ubuntu-latest
    strategy:
      matrix:
        node-version: [18, 20]
    steps:
      - uses: actions/checkout@v4
      - uses: actions/setup-node@v4
        with:
          node-version: ${{ matrix.node-version }}
          cache: 'npm'
      - run: npm ci
      - run: npm run lint
      - run: npm run test:coverage
      - uses: actions/upload-artifact@v4
        with:
          name: coverage-report
          path: coverage/

  build-and-push:
    needs: lint-and-test
    if: github.ref == 'refs/heads/main'
    runs-on: ubuntu-latest
    permissions:
      contents: read
      packages: write
    steps:
      - uses: actions/checkout@v4
      - name: Log in to Container Registry
        uses: docker/login-action@v3
        with:
          registry: ${{ env.REGISTRY }}
          username: ${{ github.actor }}
          password: ${{ secrets.GITHUB_TOKEN }}
      - name: Build and Push Docker Image
        uses: docker/build-push-action@v5
        with:
          context: .
          push: true
          tags: |
            ${{ env.REGISTRY }}/${{ env.IMAGE_NAME }}:latest
            ${{ env.REGISTRY }}/${{ env.IMAGE_NAME }}:${{ github.sha }}

  deploy:
    needs: build-and-push
    runs-on: ubuntu-latest
    environment: production
    steps:
      - name: Deploy to K8s
        run: |
          echo "Connecting to cluster..."
          # 实际部署命令（使用 kubectl 或 helm）
          kubectl set image deployment/my-app \
            app=${{ env.REGISTRY }}/${{ env.IMAGE_NAME }}:${{ github.sha }}
```

**Events 与 Runners**：

```yaml
# 常用事件
on:
  push:                         # 代码推送
  pull_request:                 # PR 创建/更新
  schedule:                     # 定时任务
    - cron: '0 2 * * *'        # 每天凌晨2点
  workflow_dispatch:            # 手动触发
  release:                      # 发布 Release

# 自托管 Runner 标签
runs-on: [self-hosted, linux, gpu]
```

**部署环境与审批**：

```yaml
jobs:
  deploy-production:
    environment:
      name: production
      url: https://myapp.com
    # 此环境配置了"必需的审阅者"
    # 只有审阅者批准后才执行
    steps:
      - run: ./deploy.sh
```

### 4.3 GitLab CI

```yaml
# .gitlab-ci.yml
stages:
  - lint
  - test
  - build
  - deploy

variables:
  DOCKER_IMAGE: $CI_REGISTRY_IMAGE:$CI_COMMIT_SHA
  DOCKER_DRIVER: overlay2

cache:
  key: ${CI_COMMIT_REF_SLUG}
  paths:
    - node_modules/
    - .npm/

lint:
  stage: lint
  image: node:20-alpine
  script:
    - npm ci
    - npm run lint

test:
  stage: test
  image: node:20-alpine
  services:
    - postgres:16-alpine
  variables:
    DATABASE_URL: postgres://postgres:password@postgres/test
  script:
    - npm ci
    - npm run test
  coverage: '/Lines: (\d+\.\d+)%/'
  artifacts:
    reports:
      coverage_report:
        coverage_format: cobertura
        path: coverage/cobertura-coverage.xml

build:
  stage: build
  image: docker:24
  services:
    - docker:24-dind
  script:
    - docker build -t $DOCKER_IMAGE .
    - docker push $DOCKER_IMAGE

.deploy_template: &deploy_job
  stage: deploy
  image: alpine/helm:3.14
  script:
    - helm upgrade --install my-app ./charts/my-app
      --set image.repository=$CI_REGISTRY_IMAGE
      --set image.tag=$CI_COMMIT_SHA
  environment:
    name: $CI_ENVIRONMENT_NAME
    url: https://$CI_ENVIRONMENT_NAME.myapp.com

deploy-staging:
  <<: *deploy_job
  variables:
    CI_ENVIRONMENT_NAME: staging
  only:
    - main

deploy-production:
  <<: *deploy_job
  variables:
    CI_ENVIRONMENT_NAME: production
  only:
    - tags
  when: manual
```

### 4.4 Jenkins

**Jenkinsfile（声明式流水线）**：

```groovy
pipeline {
    agent any

    environment {
        REGISTRY = 'registry.example.com'
        IMAGE_TAG = "${env.BUILD_NUMBER}-${env.GIT_COMMIT.take(8)}"
    }

    stages {
        stage('Checkout') {
            steps {
                checkout scm
            }
        }

        stage('Lint & Test') {
            parallel {
                stage('Lint') {
                    steps {
                        sh 'npm run lint'
                    }
                }
                stage('Unit Test') {
                    steps {
                        sh 'npm run test:ci'
                    }
                    post {
                        success {
                            junit 'reports/*.xml'
                        }
                    }
                }
            }
        }

        stage('Build') {
            steps {
                sh """
                    docker build -t ${REGISTRY}/my-app:${IMAGE_TAG} .
                    docker tag ${REGISTRY}/my-app:${IMAGE_TAG} ${REGISTRY}/my-app:latest
                """
            }
        }

        stage('Push') {
            steps {
                withCredentials([string(credentialsId: 'registry-password', variable: 'PASS')]) {
                    sh """
                        echo ${PASS} | docker login ${REGISTRY} -u admin --password-stdin
                        docker push ${REGISTRY}/my-app:${IMAGE_TAG}
                        docker push ${REGISTRY}/my-app:latest
                    """
                }
            }
        }

        stage('Deploy') {
            input {
                message "确认部署到生产环境？"
                ok "确认"
            }
            steps {
                sh "kubectl set image deployment/my-app app=${REGISTRY}/my-app:${IMAGE_TAG}"
            }
        }
    }

    post {
        failure {
            slackSend(
                channel: '#deploy-alerts',
                message: "❌ 部署失败: ${env.BUILD_URL}"
            )
        }
        success {
            slackSend(
                channel: '#deploy-alerts',
                message: "✅ 部署成功: ${env.BUILD_URL}"
            )
        }
    }
}
```

**共享库（Shared Libraries）**：

```groovy
// vars/deployToK8s.groovy
def call(String image, String namespace = 'default') {
    sh """
        kubectl set image deployment/my-app \
            app=${image} \
            -n ${namespace}
        kubectl rollout status deployment/my-app \
            -n ${namespace}
    """
}
```

### 4.5 CI/CD 平台对比

| 特性 | GitHub Actions | GitLab CI | Jenkins |
|------|---------------|-----------|---------|
| **托管方式** | SaaS | SaaS / 自托管 | 自托管为主 |
| **配置语法** | YAML | YAML | Groovy (DSL) |
| **并发限制** | 免费额度有限 | 免费额度有限 | 取决于自有资源 |
| **插件生态** | ✦ Actions 市场 | ✦ 集成度高 | ✦✦✦✦✦ 最丰富 |
| **自托管 Runner** | ✅ 支持 | ✅ 支持 | ✅ 原生支持 |
| **矩阵构建** | ✅ 原生支持 | ✅ parallel | ✅ parallel |
| **环境管理** | ✅ Environments | ✅ Environments | ✦ 插件实现 |
| **Pipeline 即代码** | ✅ | ✅ | ✅ |
| **学习曲线** | ⭐ 低 | ⭐⭐ 中 | ⭐⭐⭐⭐ 高 |
| **UI 体验** | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐ | ⭐⭐ |
| **适用场景** | GitHub 开源/私有项目 | GitLab 全生命周期 | 复杂流水线/合规要求 |

---

## 五、云服务

### 5.1 云服务模型

```
┌─────────────────────────────────────────────────────────┐
│                                                         │
│  ┌── On-Premises ──────────────────────────────────┐    │
│  │  自己管理: 硬件 + OS + 网络 + 中间件 + 应用         │    │
│  └──────────────────────────────────────────────────┘    │
│                       ↓                                   │
│  ┌── IaaS ──────────────────────────────────────────┐    │
│  │  你管理: OS + 中间件 + 应用                       │    │
│  │  云管: 计算 + 存储 + 网络                          │    │
│  │  e.g. AWS EC2, 阿里云 ECS                         │    │
│  └──────────────────────────────────────────────────┘    │
│                       ↓                                   │
│  ┌── PaaS ──────────────────────────────────────────┐    │
│  │  你管理: 应用代码                                  │    │
│  │  云管: OS + 运行时 + 中间件 + 扩展                   │    │
│  │  e.g. Heroku, AWS Elastic Beanstalk               │    │
│  └──────────────────────────────────────────────────┘    │
│                       ↓                                   │
│  ┌── FaaS ──────────────────────────────────────────┐    │
│  │  你管理: 函数代码                                  │    │
│  │  云管: 一切服务器细节                               │    │
│  │  e.g. AWS Lambda, Cloudflare Workers              │    │
│  └──────────────────────────────────────────────────┘    │
│                       ↓                                   │
│  ┌── SaaS ──────────────────────────────────────────┐    │
│  │  开箱即用, 无需管理任何东西                         │    │
│  │  e.g. GitHub, Slack, Notion                       │    │
│  └──────────────────────────────────────────────────┘    │
│                                                         │
└─────────────────────────────────────────────────────────┘
```

| 模型 | 控制力 | 运维负担 | 弹性伸缩 | 典型示例 |
|------|--------|---------|---------|---------|
| **IaaS** | 高（可配置 OS） | 高（需自己维护） | 手动/自动扩展组 | EC2, GCE, ECS |
| **PaaS** | 中（仅应用层） | 中（平台管理 OS） | 自动 | Heroku, App Engine |
| **FaaS** | 低（仅函数） | 低（事件驱动） | 原生弹性 | Lambda, Cloud Functions |
| **SaaS** | 最低（仅配置） | 无 | 完全托管 | Gmail, Notion |

### 5.2 AWS 核心服务

| 服务 | 类别 | 作用 | 替代品 |
|------|------|------|--------|
| **EC2** | 计算 (IaaS) | 虚拟服务器，完全可控 | GCE, 阿里云 ECS |
| **S3** | 存储 | 对象存储，无限容量 | GCS, MinIO |
| **RDS** | 数据库 (PaaS) | 托管关系数据库 | Cloud SQL, Aurora |
| **Lambda** | 计算 (FaaS) | 无服务器函数计算 | Cloud Functions |
| **ECS/EKS** | 容器编排 | 托管 Docker/K8s | GKE, AKS |
| **CloudFront** | CDN | 全球内容分发 | Cloudflare, Akamai |
| **Route53** | DNS | DNS 解析 + 流量路由 | Cloudflare DNS |
| **ELB/ALB** | 负载均衡 | HTTP/TCP 负载均衡 | GCLB, Nginx |
| **CloudWatch** | 监控 | 指标 + 日志 + 告警 | Datadog, Grafana |
| **IAM** | 安全 | 用户 + 角色 + 权限 | GCP IAM |

### 5.3 部署策略

```mermaid
flowchart LR
    A[当前版本 V1] --> B{选择部署策略}
    B --> C[滚动更新]
    B --> D[蓝绿部署]
    B --> E[金丝雀发布]
    B --> F[特性开关]
```

| 策略 | 原理 | 优势 | 风险 | 适用场景 |
|------|------|------|------|---------|
| **滚动更新** | 逐个替换实例 | 无需额外资源 | 回滚慢 | 无状态服务 |
| **蓝绿部署** | 维护两套完整环境，切换流量 | 瞬间切换/回滚 | 成本翻倍 | 关键业务 |
| **金丝雀发布** | 先升级一小部分实例，逐步扩大 | 风险最小，可回退 | 时间长 | 重大变更 |
| **特性开关** | 代码已部署，通过开关控制功能可见性 | 细粒度控制 | 技术债务 | 功能灰度测试 |

**蓝绿部署示例**：

```yaml
# Kubernetes + 蓝绿部署
# 当前活跃: service-prod → deployment-blue
# 准备就绪: deployment-green

apiVersion: v1
kind: Service
metadata:
  name: my-app
spec:
  selector:
    app: my-app
    version: blue   # 切换为 green 即可切换流量
  ports:
    - port: 80
```

**金丝雀部署（Istio）**：

```yaml
apiVersion: networking.istio.io/v1beta1
kind: VirtualService
metadata:
  name: my-app
spec:
  hosts:
    - my-app
  http:
    - route:
        - destination:
            host: my-app
            subset: stable
          weight: 90
        - destination:
            host: my-app
            subset: canary
          weight: 10
```

### 5.4 基础设施即代码 (IaC)

IaC 的核心思想：用代码管理基础设施，而非手动点击控制台。

```hcl
# Terraform：声明式 IaC
terraform {
  required_providers {
    aws = {
      source  = "hashicorp/aws"
      version = "~> 5.0"
    }
  }
}

provider "aws" {
  region = "ap-northeast-1"
}

variable "app_name" {
  description = "应用名称"
  type        = string
  default     = "my-app"
}

resource "aws_vpc" "main" {
  cidr_block           = "10.0.0.0/16"
  enable_dns_support   = true
  enable_dns_hostnames = true
  tags = {
    Name = "${var.app_name}-vpc"
  }
}

resource "aws_subnet" "public" {
  count             = 2
  vpc_id            = aws_vpc.main.id
  cidr_block        = "10.0.${count.index}.0/24"
  availability_zone = data.aws_availability_zones.available.names[count.index]
  map_public_ip_on_launch = true
}

resource "aws_ecs_cluster" "main" {
  name = var.app_name
}

resource "aws_ecs_service" "app" {
  name            = var.app_name
  cluster         = aws_ecs_cluster.main.id
  task_definition = aws_ecs_task_definition.app.arn
  desired_count   = 3
  launch_type     = "FARGATE"

  network_configuration {
    subnets         = aws_subnet.public[*].id
    security_groups = [aws_security_group.app.id]
  }
}
```

**IaC 工具对比**：

| 工具 | 语言 | 状态管理 | 适用场景 |
|------|------|---------|---------|
| **Terraform** | HCL | 有状态（state 文件） | 跨云资源编排 |
| **OpenTofu** | HCL | 有状态 | Terraform 开源分支 |
| **Pulumi** | Python/TS/Go/... | 有状态 | 开发者友好，代码而非 DSL |
| **AWS CDK** | TypeScript/Python/... | CloudFormation | AWS 生态 |
| **Ansible** | YAML | 无状态（幂等） | 配置管理 + 临时任务 |

### 5.5 配置管理

```yaml
# Ansible Playbook: 安装 Nginx 并部署应用
---
- name: Deploy Web Application
  hosts: webservers
  become: yes
  vars:
    app_version: "1.2.3"
    nginx_port: 80

  tasks:
    - name: Ensure Nginx is installed
      apt:
        name: nginx
        state: present

    - name: Copy Nginx configuration
      template:
        src: nginx.conf.j2
        dest: /etc/nginx/sites-available/myapp
      notify: restart nginx

    - name: Create application directory
      file:
        path: /opt/myapp
        state: directory
        owner: www-data
        group: www-data
        mode: '0755'

    - name: Deploy application binary
      get_url:
        url: "https://artifacts.example.com/myapp/{{ app_version }}/myapp"
        dest: /opt/myapp/myapp
        mode: '0755'

    - name: Start application service
      systemd:
        name: myapp
        state: started
        enabled: yes

  handlers:
    - name: restart nginx
      systemd:
        name: nginx
        state: restarted
```

---

## 六、监控与告警

### 6.1 可观测性三大支柱

```
                            ┌─────────┐
                            │  Metrics │
                            │  (指标)   │
                            │  CPU/内存 │
                            │  请求量   │
                            │  错误率   │
                            └────┬────┘
                                 │
            "什么出了问题？"      │
                                 │
      ┌──────────┐               │              ┌──────────┐
      │  Logging  │◄─────────────┼──────────────►│ Tracing  │
      │  (日志)   │   "为什么？"  │    "谁导致？"  │  (追踪)   │
      │  error.log│               │              │  请求链路  │
      │  access   │               │              │  Span 树  │
      └──────────┘                              └──────────┘
```

| 数据 | 作用 | 问什么 | 示例 |
|------|------|--------|------|
| **Metrics** | 聚合的时序数据 | 系统当前状态？ | CPU 90%, QPS 5000 |
| **Logs** | 离散的事件记录 | 具体发生了什么？ | "Connection timeout" |
| **Traces** | 请求在分布式系统中的完整路径 | 哪个环节慢了？ | SQL 查询耗时 2s |

### 6.2 Prometheus + Grafana

```yaml
# prometheus.yml
global:
  scrape_interval: 15s
  evaluation_interval: 15s

scrape_configs:
  - job_name: 'kubernetes-pods'
    kubernetes_sd_configs:
      - role: pod
    relabel_configs:
      - source_labels: [__meta_kubernetes_pod_annotation_prometheus_io_scrape]
        action: keep
        regex: true
      - source_labels: [__meta_kubernetes_pod_annotation_prometheus_io_path]
        action: replace
        target_label: __metrics_path__
        regex: (.+)
      - source_labels: [__address__, __meta_kubernetes_pod_annotation_prometheus_io_port]
        action: replace
        regex: ([^:]+)(?::\d+)?;(\d+)
        replacement: $1:$2
        target_label: __address__

  - job_name: 'node-exporter'
    static_configs:
      - targets: ['localhost:9100']

alerting:
  alertmanagers:
    - static_configs:
        - targets: ['alertmanager:9093']

rule_files:
  - "/etc/prometheus/alert-rules.yml"
```

**告警规则**：

```yaml
# alert-rules.yml
groups:
  - name: instance-down
    rules:
      - alert: InstanceDown
        expr: up == 0
        for: 5m
        labels:
          severity: critical
        annotations:
          summary: "Instance {{ $labels.instance }} down"
          description: "{{ $labels.instance }} of job {{ $labels.job }} has been down for more than 5 minutes."

  - name: high-cpu
    rules:
      - alert: HighCPUUsage
        expr: (100 - (avg by(instance)(rate(node_cpu_seconds_total{mode="idle"}[5m])) * 100)) > 80
        for: 10m
        labels:
          severity: warning
        annotations:
          summary: "CPU usage > 80% on {{ $labels.instance }}"
```

**Grafana 面板示例**：

```
┌────────────────────────────────────────────────────┐
│  [CPU 使用率]  [内存]   [磁盘IO]  [网络流量]         │
│  ┌────┐ ┌────┐ ┌────┐ ┌────┐                      │
│  │ 65│ │ 4.2│ │ 45│ │120│                      │
│  │ %  │ │ GB │ │ MB│ │Mbps│                      │
│  └────┘ └────┘ └────┘ └────┘                      │
├────────────────────────────────────────────────────┤
│  QPS 趋势图                                         │
│  ╱╲    ╱╲    ╱╲    ╱╲                              │
│ ╱  ╲  ╱  ╲  ╱  ╲  ╱  ╲                             │
├────────────────────────────────────────────────────┤
│  各接口 P99 延迟                                     │
│  /api/users  ━━━━━━━━━━━━━━━━━── 120ms             │
│  /api/orders ━━━━━━━━━━━━━━━━━━━━ 350ms            │
│  /api/pay    ━━━━━━━━━━━━━━━━━━━━━━━━━ 890ms       │
└────────────────────────────────────────────────────┘
```

### 6.3 ELK Stack（日志管理）

```
Application
  │
  ├── Filebeat / Fluent Bit (日志采集器)
  │     └── 发送到 Logstash
  │
  ├── Logstash (日志处理管道)
  │     ├── 解析 (Grok 正则、JSON 解析)
  │     ├── 过滤 (移除敏感信息、增加字段)
  │     └── 输出到 Elasticsearch
  │
  ├── Elasticsearch (存储 + 搜索)
  │     └── 倒排索引，全文搜索
  │
  └── Kibana (可视化 + 探索)
        └── 日志搜索、仪表盘、告警
```

**Logstash 配置示例**：

```ruby
input {
  beats {
    port => 5044
  }
}

filter {
  # 解析 Nginx 访问日志
  if [service] == "nginx" {
    grok {
      match => { "message" => "%{COMBINEDAPACHELOG}" }
    }
    date {
      match => ["timestamp", "dd/MMM/yyyy:HH:mm:ss Z"]
    }
  }

  # 解析 JSON 格式的应用日志
  if [service] == "app" {
    json {
      source => "message"
    }
  }

  # 移除敏感字段
  mutate {
    remove_field => ["password", "credit_card"]
  }
}

output {
  elasticsearch {
    hosts => ["http://elasticsearch:9200"]
    index => "%{[@metadata][beat]}-%{+YYYY.MM.dd}"
  }
}
```

**结构化日志（应用侧）**：

```python
# Python 结构化日志示例
import structlog

logger = structlog.get_logger()
logger.info("user_login", 
    user_id=123, 
    ip="192.168.1.1", 
    device="mobile",
    duration_ms=45
)
# 输出: {"event": "user_login", "user_id": 123, "ip": "192.168.1.1", ...}
```

### 6.4 分布式追踪

**OpenTelemetry 是 CNCF 的追踪标准**，统一了数据采集和上报协议。

```python
# Python + OpenTelemetry 示例
from opentelemetry import trace
from opentelemetry.exporter.otlp.proto.grpc.trace_exporter import OTLPSpanExporter
from opentelemetry.sdk.trace import TracerProvider
from opentelemetry.sdk.trace.export import BatchSpanProcessor

provider = TracerProvider()
processor = BatchSpanProcessor(OTLPSpanExporter(
    endpoint="http://jaeger-collector:4317"
))
provider.add_span_processor(processor)
trace.set_tracer_provider(provider)

tracer = trace.get_tracer(__name__)

with tracer.start_as_current_span("process_order") as span:
    span.set_attribute("order_id", 12345)
    span.add_event("payment_initiated")
    
    with tracer.start_as_current_span("check_inventory"):
        # 库存检查逻辑
        pass
    
    with tracer.start_as_current_span("charge_payment"):
        # 支付逻辑
        pass
```

**Jaeger UI 中的 Trace 视图**：

```
Trace: 8a3f2b1c (257ms)
  ├── frontend: GET /checkout        (257ms)
  │   ├── auth: verify_token         (12ms)
  │   ├── cart: get_cart_items       (45ms)
  │   │   └── redis: hgetall         (8ms)
  │   ├── order: create_order        (180ms)  ← 瓶颈在这里
  │   │   ├── payment: charge        (120ms)
  │   │   ├── inventory: reserve     (35ms)
  │   │   └── db: insert             (25ms)
  │   └── notification: send_email   (20ms)
```

### 6.5 告警最佳实践

| 原则 | 说明 | 坏例子 | 好例子 |
|------|------|--------|--------|
| **减少噪音** | 告警要少而精 | 每个 CPU 尖峰都告警 | 持续 5 分钟 CPU > 90% 才告警 |
| **可操作** | 收到告警能采取行动 | "CPU 高" | "CPU 持续 10 分钟 > 90%，请扩容或排查死循环" |
| **分级** | 根据影响程度分级 | 所有告警都 P0 | P0(核心业务中断)、P1(部分功能受损)、P2(非关键告警) |
| **Runbook** | 每个告警有明确处理流程 | 收到告警不知怎么办 | 告警消息附带排障步骤链接 |
| **On-Call** | 轮值制度 + 升级机制 | 长期无人响应 | 15min 未确认升级到二级 On-Call |
| **事后复盘** | 从故障中学习，不问责 | 追究责任人 | 整改 Runbook 和系统 |

**告警分级示例**：

```yaml
# PagerDuty / Opsgenie 告警规则
Critical (P0):
  - 服务完全不可用
  - 数据库连接数耗尽
  - 支付失败率 > 5%
  响应时间: 15分钟内确认，1小时内修复

Warning (P1):
  - API P99 延迟 > 500ms
  - 错误率 > 1%
  - 磁盘使用率 > 85%
  响应时间: 30分钟内确认，4小时内修复

Info (P2):
  - CPU > 80%（持续30分钟）
  - 证书即将过期（30天内）
  响应时间: 下一个工作日处理
```

---

## 七、数据库部署

### 7.1 数据库迁移自动化

```bash
# 使用 golang-migrate 管理迁移
# 迁移文件命名：{version}_{description}.{up|down}.sql

# 创建迁移文件
migrate create -ext sql -dir migrations -seq create_users_table

# 生成的文件：
# migrations/
# ├── 000001_create_users_table.up.sql
# └── 000001_create_users_table.down.sql
```

**迁移文件示例**：

```sql
-- 000001_create_users_table.up.sql
CREATE TABLE users (
    id         BIGSERIAL PRIMARY KEY,
    name       VARCHAR(255) NOT NULL,
    email      VARCHAR(255) NOT NULL UNIQUE,
    password   VARCHAR(255) NOT NULL,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

CREATE INDEX idx_users_email ON users(email);

-- 000001_create_users_table.down.sql
DROP TABLE IF EXISTS users;
```

**CI/CD 中的迁移执行**：

```yaml
# GitHub Actions 中的数据库迁移
jobs:
  migrate:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - name: Run migrations
        run: |
          migrate -path migrations \
                  -database "${{ secrets.DATABASE_URL }}" \
                  up
      - name: Verify schema
        run: |
          migrate -path migrations \
                  -database "${{ secrets.DATABASE_URL }}" \
                  version
```

### 7.2 备份策略

| 策略 | 方法 | RPO (数据丢失) | RTO (恢复时间) | 存储成本 |
|------|------|---------------|---------------|---------|
| **每日全量** | pg_dump 每天一次 | 24小时 | 中（数小时） | 高 |
| **WAL 归档** | 持续归档 WAL 日志 | < 1秒 | 低（分钟级） | 低 |
| **快照备份** | 存储层面快照（EBS 快照） | 秒级 | 低（分钟级） | 中 |
| **物理复制** | 流复制搭建从库 | 零（同步模式） | 即时 | 中 |

```bash
#!/bin/bash
# 数据库备份脚本 (PostgreSQL)

BACKUP_DIR="/backups/$(date +%Y-%m-%d)"
mkdir -p "$BACKUP_DIR"

# 全量备份
pg_dump -h $DB_HOST -U $DB_USER \
    --format=custom \
    --compress=9 \
    --file="$BACKUP_DIR/db_$(date +%H%M).dump" \
    $DB_NAME

# 保留最近 7 天，删除更早的
find /backups -type d -mtime +7 -exec rm -rf {} \;

# 上传到 S3（异地备份）
aws s3 sync /backups s3://myapp-db-backups/
```

### 7.3 读写分离（读副本）

```
Application
  │
  ├── 写入 → Primary (主库)
  │              │
  │              ├── 流复制 → Replica 1 (只读 + 报表查询)
  │              ├── 流复制 → Replica 2 (只读 + 分析查询)
  │              └── 流复制 → Replica 3 (灾备)
  │
  └── 读取 → Load Balancer → Replicas (轮询/权重)
```

```python
# 应用层读写分离（Django ORM 示例）
DATABASES = {
    'default': {           # 写库
        'ENGINE': 'django.db.backends.postgresql',
        'NAME': 'myapp',
        'HOST': 'primary.cluster.example.com',
    },
    'replica': {           # 读库
        'ENGINE': 'django.db.backends.postgresql',
        'NAME': 'myapp',
        'HOST': 'replica.cluster.example.com',
    }
}

DATABASE_ROUTERS = ['myapp.db_router.PrimaryReplicaRouter']

# db_router.py
class PrimaryReplicaRouter:
    def db_for_read(self, model, **hints):
        return 'replica'

    def db_for_write(self, model, **hints):
        return 'default'

    def allow_relation(self, obj1, obj2, **hints):
        return True
```

### 7.4 连接池

数据库连接池是应用层与数据库之间的中间层，复用连接以减少创建开销。

```bash
# PgBouncer 配置
# pgbouncer.ini
[databases]
myapp = host=primary.cluster.example.com port=5432 dbname=myapp

[pgbouncer]
listen_addr = 0.0.0.0
listen_port = 6432
auth_type = md5
auth_file = /etc/pgbouncer/userlist.txt

# 连接池模式
# session: 每个会话一个连接（默认）
# transaction: 每个事务一个连接（推荐用于 Web 应用）
# statement: 每个语句一个连接
pool_mode = transaction

# 池大小
default_pool_size = 50
max_client_conn = 500
reserve_pool_size = 10
reserve_pool_timeout = 5

# 超时
server_idle_timeout = 300
client_idle_timeout = 600
query_timeout = 30
```

```python
# 应用层连接池（SQLAlchemy 示例）
from sqlalchemy import create_engine
from sqlalchemy.pool import QueuePool

engine = create_engine(
    'postgresql://user:pass@pgbouncer:6432/myapp',
    poolclass=QueuePool,
    pool_size=20,           # 连接池大小
    max_overflow=10,        # 超过 pool_size 时最多创建多少额外连接
    pool_pre_ping=True,     # 每次使用前检查连接是否有效
    pool_recycle=3600,      # 连接超过1小时重新连接
)
```

| 连接池工具 | 部署位置 | 特点 | 适用数据库 |
|-----------|---------|------|-----------|
| **PgBouncer** | 独立服务（Sidecar/Proxy） | 轻量、事务池模式 | PostgreSQL |
| **ProxySQL** | 独立服务 | SQL 路由、查询重写、读写分离 | MySQL |
| **HikariCP** | 应用内库 | Java 生态最快 | 通用（JDBC） |
| **Druid** | 应用内库 | 监控完善、慢查询日志 | 通用（JDBC） |

> **关键要点**：连接池的大小不是越大越好。每个连接消耗内存和 CPU，超过数据库的 max_connections 反而会降低性能。通常 CPU 核心数 × 2~4 是合理的起始点。

---

## 总结

DevOps 是一套理念、实践和工具的结合体，核心目标是将软件从构思到交付的周期缩到最短，同时保证质量与可靠性。

| 领域 | 核心思想 | 关键工具 | 进阶方向 |
|------|---------|---------|---------|
| **文化** | 协作、自动化、持续改进 | CALMS 框架、DORA 指标 | 平台工程、开发者体验 |
| **容器** | 不可变基础设施、环境一致 | Docker、Docker Compose | containerd、WASM |
| **编排** | 声明式管理、自愈、弹性 | Kubernetes、Helm | 服务网格 (Istio/Linkerd) |
| **CI/CD** | 自动化流水线、快速反馈 | GitHub Actions、GitLab CI | ArgoCD、Flux (GitOps) |
| **云服务** | 按需付费、弹性伸缩 | AWS、Terraform | Multi-Cloud、FinOps |
| **监控** | 可观测性、数据驱动决策 | Prometheus、Grafana、ELK | eBPF、OpenTelemetry |
| **数据库部署** | 高可用、数据安全 | PgBouncer、ProxySQL | 分布式数据库 (TiDB/CockroachDB) |

> DevOps 的最终目标是让开发者能专注于**交付业务价值**，而不是被部署流程和运维杂事拖慢。工具和流程只是手段，文化和持续改进才是核心。

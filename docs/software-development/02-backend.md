# 后端开发

> 本章覆盖多语言、多框架的后端开发核心知识。从架构分层开始，逐一深入 Python、Node.js、Java、Go、Rust 五大生态，详解 API 设计、认证授权、消息队列以及安全实践。代码即文档，每节均包含可直接运行的核心示例。

---

## 一、后端架构概览

### 1.1 服务端请求生命周期

一个 HTTP 请求从抵达服务器到返回响应，经过一系列明确定义的阶段。理解这个生命周期是后端开发的起点。

```
Client
  │
  ├── 1. DNS 解析: 域名 → IP
  ├── 2. TCP 三次握手: 建立连接
  ├── 3. TLS 握手: 加密协商 (HTTPS)
  └── 4. HTTP 请求发送
         │
         ▼
    Web Server (Nginx / Caddy / 框架内置)
         │
         ├── 中间件链 (Middleware Pipeline)
         │   ├── 请求日志
         │   ├── CORS 头处理
         │   ├── 认证 / 鉴权
         │   ├── 速率限制
         │   ├── 请求体解析 (JSON / Form / Multipart)
         │   ├── 路由匹配 → 参数提取
         │   └── 上下文注入 (Request ID, 用户信息)
         │
         ├── Controller (控制器)
         │   └── 接收参数, 调用 Service, 返回响应
         │
         ├── Service (业务服务层)
         │   ├── 参数校验与转换
         │   ├── 权限校验
         │   ├── 核心业务逻辑
         │   └── 组合多个 Repository 调用
         │
         ├── 缓存层 (Redis / Memcached)
         │   ├── 命中 → 直接返回
         │   └── 未命中 → 查数据库 → 回填缓存
         │
         ├── Repository / DAO (数据访问层)
         │   ├── ORM 映射 (对象 → SQL)
         │   └── 连接池管理
         │
         ▼
    Database (MySQL / PostgreSQL / MongoDB)
         │
         └── 结果沿原路径返回 → JSON 序列化 → HTTP 响应
```

### 1.2 中间件模式

中间件（Middleware）是后端框架最核心的抽象之一。每个请求依次穿过一个函数链，每个函数可以：

- 执行预处理（日志、认证、CORS）
- 决定是否将请求传递给下一个中间件
- 在响应返回后执行后处理

```python
# 伪代码: 中间件链的本质
def middleware_chain(request, middlewares):
    def run(index, req):
        if index >= len(middlewares):
            return route_handler(req)
        return middlewares[index](req, lambda r: run(index + 1, r))
    return run(0, request)
```

中间件模式的优势在于**横切关注点（Cross-cutting Concerns）**的分离——认证、日志、限流这些逻辑不需要侵入业务代码。

### 1.3 三层架构：Controller → Service → Repository

这是后端应用最经典的分层模式：

| 层 | 职责 | 常见命名 | 注意 |
|---|------|---------|------|
| **Controller** | 接收 HTTP 请求、解析参数、返回响应 | `UserController`、`AuthController` | 不应包含业务逻辑 |
| **Service** | 业务规则、跨 Repo 的组合、事务协调 | `UserService`、`OrderService` | 可复用，不依赖 HTTP |
| **Repository** | 数据存取、ORM 查询构建 | `UserRepository`、`OrderRepo` | 只负责持久化 |

核心约束：**依赖方向必须是 Controller → Service → Repository**，反向依赖通过依赖注入（DI）解耦。

---

## 二、Python 后端生态

Python 在后端领域拥有三个各有千秋的主流框架，覆盖从微型 API 到企业级全栈的全部需求。

### 2.1 Flask — 轻量灵活

Flask 的核心哲学是"微框架"——只提供路由和模板引擎，其他能力通过扩展按需引入。

**应用工厂模式**

```python
# app_factory.py
from flask import Flask

def create_app(config_name: str = "development") -> Flask:
    app = Flask(__name__)
    app.config.from_object(f"config.{config_name.capitalize()}Config")

    # 注册蓝图
    from blueprints.users import users_bp
    from blueprints.auth import auth_bp
    app.register_blueprint(users_bp, url_prefix="/api/users")
    app.register_blueprint(auth_bp, url_prefix="/api/auth")

    # 注册错误处理器
    @app.errorhandler(404)
    def not_found(e):
        return {"error": "资源不存在"}, 404

    @app.errorhandler(500)
    def server_error(e):
        return {"error": "服务器内部错误"}, 500

    # 注册中间件
    @app.before_request
    def log_request():
        app.logger.info(f"{request.method} {request.path}")

    return app
```

**路由与蓝图**

```python
# blueprints/users.py
from flask import Blueprint, request, jsonify

users_bp = Blueprint("users", __name__)

@users_bp.route("/<int:user_id>", methods=["GET"])
def get_user(user_id: int):
    # 实际业务应调用 Service 层
    return jsonify({"id": user_id, "name": "Alice"})

@users_bp.route("/", methods=["POST"])
def create_user():
    data = request.get_json()
    if not data or "name" not in data:
        return {"error": "name is required"}, 400
    # 调用 service.create_user(data)
    return jsonify({"id": 1, **data}), 201
```

### 2.2 Django — 全栈电池

Django 奉行"Batteries-included"哲学，ORM、Admin、表单、认证、信号全部内置。

**MTV 模式**（Model-Template-View，本质是 MVC 的变体）：

```python
# models.py
from django.db import models

class Category(models.Model):
    name = models.CharField(max_length=100)
    created_at = models.DateTimeField(auto_now_add=True)

class Article(models.Model):
    title = models.CharField(max_length=200)
    content = models.TextField()
    category = models.ForeignKey(Category, on_delete=models.CASCADE,
                                  related_name="articles")
    views = models.IntegerField(default=0)
    created_at = models.DateTimeField(auto_now_add=True)

    class Meta:
        indexes = [models.Index(fields=["-created_at"])]
```

**Django REST Framework（DRF）——构建 API 的标准方案**：

```python
# serializers.py
from rest_framework import serializers
from .models import Article

class ArticleSerializer(serializers.ModelSerializer):
    category_name = serializers.CharField(source="category.name", read_only=True)

    class Meta:
        model = Article
        fields = ["id", "title", "content", "category", "category_name",
                   "views", "created_at"]

# views.py
from rest_framework import viewsets, permissions
from .models import Article
from .serializers import ArticleSerializer

class ArticleViewSet(viewsets.ModelViewSet):
    queryset = Article.objects.select_related("category").all()
    serializer_class = ArticleSerializer
    permission_classes = [permissions.IsAuthenticatedOrReadOnly]

    def perform_create(self, serializer):
        serializer.save(author=self.request.user)
```

**Signal —— 解耦事件处理**：

```python
# signals.py
from django.db.models.signals import post_save
from django.dispatch import receiver
from django.core.cache import cache

@receiver(post_save, sender=Article)
def clear_article_cache(sender, instance, **kwargs):
    cache.delete(f"article:{instance.id}")
    cache.delete("article:list")
```

**Admin 配置 —— 零代码管理后台**：

```python
# admin.py
from django.contrib import admin
from .models import Article

@admin.register(Article)
class ArticleAdmin(admin.ModelAdmin):
    list_display = ["title", "category", "views", "created_at"]
    list_filter = ["category", "created_at"]
    search_fields = ["title"]
    ordering = ["-created_at"]
```

### 2.3 FastAPI — 现代异步

FastAPI 基于 Starlette（HTTP 层）和 Pydantic（数据验证层），原生支持异步，自动生成 OpenAPI 文档。

**Pydantic 模型与自动验证**：

```python
from pydantic import BaseModel, Field, EmailStr
from typing import Optional
from datetime import datetime

class UserCreate(BaseModel):
    username: str = Field(..., min_length=3, max_length=50)
    email: EmailStr
    password: str = Field(..., min_length=8)

class UserResponse(BaseModel):
    id: int
    username: str
    email: str
    created_at: datetime

    model_config = {"from_attributes": True}
```

**异步路由与依赖注入**：

```python
from fastapi import FastAPI, Depends, HTTPException, status
from fastapi.security import OAuth2PasswordBearer
from sqlalchemy.ext.asyncio import AsyncSession

app = FastAPI(title="CodeNebula API", version="1.0.0")
oauth2_scheme = OAuth2PasswordBearer(tokenUrl="/auth/login")

# 依赖注入：数据库会话
async def get_db() -> AsyncSession:
    async with async_session() as session:
        yield session

# 依赖注入：当前用户
async def get_current_user(
    token: str = Depends(oauth2_scheme),
    db: AsyncSession = Depends(get_db),
):
    user = await auth_service.verify_token(token, db)
    if not user:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="无效的认证凭据",
        )
    return user

@app.post("/users/", response_model=UserResponse, status_code=201)
async def create_user(
    data: UserCreate,
    db: AsyncSession = Depends(get_db),
):
    # Pydantic 自动完成参数校验
    existing = await user_repo.find_by_email(data.email, db)
    if existing:
        raise HTTPException(409, detail="邮箱已注册")
    return await user_service.create(data, db)

@app.get("/users/me", response_model=UserResponse)
async def read_my_profile(
    current_user: User = Depends(get_current_user),
):
    return current_user
```

**自动 OpenAPI 文档**：启动后访问 `/docs`（Swagger UI）和 `/redoc`（ReDoc），所有接口和模型自动生成文档。

### 2.4 对比：Django vs Flask vs FastAPI

| 维度 | Django | Flask | FastAPI |
|------|--------|-------|---------|
| **类型** | 全栈框架 | 微框架 | API 框架 |
| **ORM** | 内置 Django ORM | 无（可选 SQLAlchemy） | 无（可选 SQLAlchemy / Tortoise） |
| **异步** | 3.0+ 支持 ASGI | 需插件（Quart） | 原生异步（asyncio） |
| **性能** | 中等 | 中等 | 高（Uvicorn + async） |
| **学习曲线** | 中（需学全套概念） | 低（简单直接） | 中（需理解类型提示和 async） |
| **生态规模** | 最大 | 大 | 成长中 |
| **内置 Admin** | ✅ 强大 | ❌（需扩展） | ❌（需扩展） |
| **API 文档** | DRF + Swagger | Flask-RESTx | 自动 OpenAPI |
| **适用场景** | 内容管理、全栈快速开发 | 微服务、原型、小项目 | 高并发 API、ML 推理服务 |
| **社区成熟度** | 非常成熟 | 非常成熟 | 快速成熟中 |

---

## 三、Node.js 后端生态

Node.js 的异步 I/O 模型使其在处理高并发 I/O 密集型场景时表现出色。TypeScript 的普及更让大型 Node.js 项目具备了静态类型的安全网。

### 3.1 Express — 最流行的 Node 框架

Express 是 Node.js 生态中最经典的框架，中间件模式贯穿始终。

**中间件链示例**：

```typescript
import express, { Request, Response, NextFunction } from "express";

const app = express();

// 应用级中间件
app.use(express.json());
app.use(cors());

// 路由级中间件
const authMiddleware = (req: Request, res: Response, next: NextFunction) => {
  const token = req.headers.authorization?.split(" ")[1];
  if (!token) {
    return res.status(401).json({ error: "未提供认证令牌" });
  }
  try {
    (req as any).user = jwt.verify(token, process.env.JWT_SECRET!);
    next();
  } catch {
    return res.status(401).json({ error: "令牌无效或已过期" });
  }
};

// 路由定义
app.get("/api/users/:id", authMiddleware, async (req, res, next) => {
  try {
    const user = await userService.findById(req.params.id);
    if (!user) return res.status(404).json({ error: "用户不存在" });
    res.json(user);
  } catch (err) {
    next(err);  // 传递给错误处理中间件
  }
});

// 错误处理中间件（四个参数）
app.use((err: Error, req: Request, res: Response, next: NextFunction) => {
  console.error("Unhandled error:", err);
  res.status(500).json({ error: "服务器内部错误" });
});

app.listen(3000);
```

**模板引擎**：Express 支持多种模板引擎（EJS、Pug、Handlebars），用于服务端渲染。但在现代 API 开发中更常见的做法是用 Express 仅提供 JSON API。

### 3.2 NestJS — 企业级模块化

NestJS 采用 Angular 风格的模块化架构，结合 TypeScript 装饰器，提供了类似 Spring Boot 的开发体验。

**模块与装饰器**：

```typescript
// user.module.ts
import { Module } from "@nestjs/common";
import { TypeOrmModule } from "@nestjs/typeorm";
import { UserController } from "./user.controller";
import { UserService } from "./user.service";
import { User } from "./user.entity";

@Module({
  imports: [TypeOrmModule.forFeature([User])],
  controllers: [UserController],
  providers: [UserService],
  exports: [UserService],
})
export class UserModule {}

// user.controller.ts
import { Controller, Get, Post, Body, Param, UseGuards } from "@nestjs/common";
import { UserService } from "./user.service";
import { CreateUserDto } from "./dto/create-user.dto";
import { JwtAuthGuard } from "../auth/jwt-auth.guard";

@Controller("api/users")
export class UserController {
  constructor(private readonly userService: UserService) {}

  @Post()
  async create(@Body() dto: CreateUserDto) {
    return this.userService.create(dto);
  }

  @Get(":id")
  @UseGuards(JwtAuthGuard)
  async findOne(@Param("id") id: string) {
    return this.userService.findById(+id);
  }
}
```

**Guards、Interceptors、Pipes** —— NestJS 的 AOP 机制：

```typescript
// guard: 请求守卫（认证/授权）
@Injectable()
export class RolesGuard implements CanActivate {
  constructor(private readonly roles: string[]) {}

  canActivate(context: ExecutionContext): boolean {
    const request = context.switchToHttp().getRequest();
    return this.roles.includes(request.user?.role);
  }
}

// interceptor: 请求/响应拦截
@Injectable()
export class LoggingInterceptor implements NestInterceptor {
  intercept(context: ExecutionContext, next: CallHandler): Observable<any> {
    const start = Date.now();
    return next.handle().pipe(
      tap(() => console.log(`耗时: ${Date.now() - start}ms`)),
    );
  }
}

// pipe: 参数转换与验证
@Injectable()
export class ParseIntPipe implements PipeTransform<string, number> {
  transform(value: string): number {
    const val = parseInt(value, 10);
    if (isNaN(val)) throw new BadRequestException("ID 必须是数字");
    return val;
  }
}
```

### 3.3 Express vs NestJS 对比

| 维度 | Express | NestJS |
|------|---------|--------|
| **架构风格** | 自由（按需组织） | 模块化（强制分层） |
| **类型安全** | 可选（JS 或 TS） | 强制 TypeScript |
| **AOP 支持** | 仅中间件 | Guards + Interceptors + Pipes |
| **DI 容器** | 无（手写或 inversify） | 内置（模块级注入） |
| **性能** | 极高 | 略低（反射开销） |
| **学习曲线** | 低 | 中高 |
| **适合项目** | 小型/中型 API、原型 | 大型企业级项目 |
| **社区生态** | 最大 | 大 |
| **测试工具** | 需自行配置 | 内置 Testing Module |

---

## 四、Java 后端生态

Java 在企业级后端领域的主导地位由 Spring Boot 系列框架确立。类型安全、成熟生态、强大的工具链是其核心竞争力。

### 4.1 Spring Boot — 企业级标准

**IoC 容器与依赖注入**：

```java
// 主应用
@SpringBootApplication
public class CodeNebulaApplication {
    public static void main(String[] args) {
        SpringApplication.run(CodeNebulaApplication.class, args);
    }
}

// 服务层 — 自动注入依赖
@Service
public class UserService {
    private final UserRepository userRepository;
    private final PasswordEncoder passwordEncoder;

    public UserService(UserRepository userRepository,
                       PasswordEncoder passwordEncoder) {
        this.userRepository = userRepository;
        this.passwordEncoder = passwordEncoder;
    }

    @Transactional
    public UserResponse createUser(CreateUserRequest request) {
        if (userRepository.existsByEmail(request.email())) {
            throw new BusinessException("邮箱已被注册");
        }
        User user = new User();
        user.setUsername(request.username());
        user.setEmail(request.email());
        user.setPassword(passwordEncoder.encode(request.password()));
        user = userRepository.save(user);
        return UserResponse.from(user);
    }
}
```

**REST Controller 与 DTO**：

```java
@RestController
@RequestMapping("/api/users")
public class UserController {

    private final UserService userService;

    public UserController(UserService userService) {
        this.userService = userService;
    }

    @PostMapping
    @ResponseStatus(HttpStatus.CREATED)
    public ApiResponse<UserResponse> createUser(
            @Valid @RequestBody CreateUserRequest request) {
        UserResponse response = userService.createUser(request);
        return ApiResponse.success(response);
    }

    @GetMapping("/{id}")
    public ApiResponse<UserResponse> getUser(@PathVariable Long id) {
        return ApiResponse.success(userService.findById(id));
    }

    @GetMapping
    public ApiResponse<Page<UserResponse>> listUsers(
            @PageableDefault(size = 20) Pageable pageable) {
        return ApiResponse.success(userService.findAll(pageable));
    }
}
```

**JPA/Hibernate 实体映射**：

```java
@Entity
@Table(name = "users")
public class User {
    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    private Long id;

    @Column(nullable = false, unique = true, length = 50)
    private String username;

    @Column(nullable = false, unique = true)
    private String email;

    @Column(nullable = false)
    private String password;

    @Enumerated(EnumType.STRING)
    private UserRole role = UserRole.USER;

    @OneToMany(mappedBy = "author", cascade = CascadeType.ALL)
    private List<Article> articles = new ArrayList<>();

    @CreationTimestamp
    private LocalDateTime createdAt;

    @UpdateTimestamp
    private LocalDateTime updatedAt;

    // getters / setters / equals / hashCode
}
```

**Spring Security 配置**：

```java
@Configuration
@EnableWebSecurity
public class SecurityConfig {

    private final JwtAuthFilter jwtAuthFilter;

    public SecurityConfig(JwtAuthFilter jwtAuthFilter) {
        this.jwtAuthFilter = jwtAuthFilter;
    }

    @Bean
    public SecurityFilterChain filterChain(HttpSecurity http) throws Exception {
        return http
            .csrf(AbstractHttpConfigurer::disable)
            .sessionManagement(sm -> sm.sessionCreationPolicy(
                SessionCreationPolicy.STATELESS))
            .authorizeHttpRequests(auth -> auth
                .requestMatchers("/api/auth/**", "/api/public/**").permitAll()
                .requestMatchers("/api/admin/**").hasRole("ADMIN")
                .anyRequest().authenticated()
            )
            .addFilterBefore(jwtAuthFilter, UsernamePasswordAuthenticationFilter.class)
            .exceptionHandling(ex -> ex
                .authenticationEntryPoint((req, res, e) ->
                    res.sendError(HttpServletResponse.SC_UNAUTHORIZED)))
            .build();
    }

    @Bean
    public PasswordEncoder passwordEncoder() {
        return new BCryptPasswordEncoder();
    }
}
```

**Actuator —— 生产就绪端点**：

```yaml
# application.yml
management:
  endpoints:
    web:
      exposure:
        include: health,info,metrics,prometheus
  endpoint:
    health:
      show-details: when-authorized
```

通过 Actuator，应用自动暴露 `/actuator/health`、`/actuator/metrics` 等监控端点，无额外代码即可集成 Prometheus、Grafana。

### 4.2 标准项目结构

```
src/main/java/com/codenebula/
├── CodeNebulaApplication.java    # 启动类
├── config/                       # 配置类
│   ├── SecurityConfig.java
│   ├── SwaggerConfig.java
│   └── CorsConfig.java
├── controller/                   # 控制器层
│   ├── UserController.java
│   └── AuthController.java
├── service/                      # 业务服务层
│   ├── UserService.java
│   └── AuthService.java
├── repository/                   # 数据访问层
│   ├── UserRepository.java
│   └── ArticleRepository.java
├── entity/                       # 数据库实体
│   ├── User.java
│   └── Article.java
├── dto/                          # 数据传输对象
│   ├── request/
│   │   ├── CreateUserRequest.java
│   │   └── LoginRequest.java
│   └── response/
│       ├── UserResponse.java
│       └── ApiResponse.java
├── exception/                    # 异常与错误处理
│   ├── BusinessException.java
│   └── GlobalExceptionHandler.java
└── util/                         # 工具类
    ├── JwtUtil.java
    └── PageUtil.java
```

### 4.3 Maven vs Gradle

| 维度 | Maven | Gradle |
|------|-------|--------|
| **构建语言** | XML（POM） | Groovy / Kotlin DSL |
| **性能** | 较慢 | 快 2-10 倍（增量编译、构建缓存） |
| **依赖管理** | 成熟但冗长 | 简洁灵活 |
| **约定优先** | 强约定 | 可自定义 |
| **多模块** | 支持（较复杂） | 原生优雅支持 |
| **生态** | 标准的 JAR 生态 | 兼容 Maven 仓库 |
| **学习曲线** | 低（XML 扁平） | 中（DSL 灵活性强） |
| **云原生构建** | 不支持 | 支持（Gradle Build Cache） |

> 对于新项目，**推荐 Gradle**。它更快、语法更简洁、Kotlin DSL 让构建脚本也能享受 IDE 类型提示。

---

## 五、Go 后端

Go 语言以其极简的语法、原生并发模型和出色的编译速度，在云原生和微服务领域占据重要地位。

### 5.1 Gin — 高性能 HTTP 框架

Gin 是目前最流行的 Go 框架，性能接近原生 `net/http`，API 设计简洁。

```go
package main

import (
    "net/http"
    "github.com/gin-gonic/gin"
)

type User struct {
    ID       uint   `json:"id"`
    Username string `json:"username" binding:"required,min=3,max=50"`
    Email    string `json:"email" binding:"required,email"`
}

func main() {
    r := gin.Default()

    // 路由分组
    api := r.Group("/api")
    {
        users := api.Group("/users")
        {
            users.GET("/:id", getUser)
            users.POST("/", createUser)
            users.PUT("/:id", updateUser)
            users.DELETE("/:id", authMiddleware(), deleteUser)
        }
    }

    r.GET("/health", func(c *gin.Context) {
        c.JSON(http.StatusOK, gin.H{"status": "ok"})
    })

    r.Run(":8080")
}

func createUser(c *gin.Context) {
    var req User
    if err := c.ShouldBindJSON(&req); err != nil {
        c.JSON(http.StatusBadRequest, gin.H{
            "error": err.Error(),
        })
        return
    }
    // 调用 service 层
    c.JSON(http.StatusCreated, gin.H{
        "id": 1, "username": req.Username, "email": req.Email,
    })
}

func authMiddleware() gin.HandlerFunc {
    return func(c *gin.Context) {
        token := c.GetHeader("Authorization")
        if token == "" {
            c.AbortWithStatusJSON(http.StatusUnauthorized,
                gin.H{"error": "未提供认证令牌"})
            return
        }
        // 验证 token...
        c.Set("userID", uint(1))
        c.Next()
    }
}
```

### 5.2 net/http 标准库

Go 的标准库 `net/http` 已经足够强大，许多生产服务直接用标准库构建而不依赖第三方框架。

```go
package main

import (
    "encoding/json"
    "log"
    "net/http"
    "strings"
)

type APIHandler struct {
    userService *UserService
}

func (h *APIHandler) ServeHTTP(w http.ResponseWriter, r *http.Request) {
    // 简单路由分发
    path := strings.TrimPrefix(r.URL.Path, "/api")
    switch {
    case path == "/users" && r.Method == http.MethodGet:
        h.listUsers(w, r)
    case strings.HasPrefix(path, "/users/") && r.Method == http.MethodGet:
        h.getUser(w, r)
    case path == "/users" && r.Method == http.MethodPost:
        h.createUser(w, r)
    default:
        http.NotFound(w, r)
    }
}

func (h *APIHandler) createUser(w http.ResponseWriter, r *http.Request) {
    var req CreateUserRequest
    if err := json.NewDecoder(r.Body).Decode(&req); err != nil {
        http.Error(w, `{"error":"无效的请求体"}`, http.StatusBadRequest)
        return
    }
    user, err := h.userService.Create(r.Context(), req)
    if err != nil {
        http.Error(w, `{"error":"`+err.Error()+`"}`, http.StatusInternalServerError)
        return
    }
    w.Header().Set("Content-Type", "application/json")
    w.WriteHeader(http.StatusCreated)
    json.NewEncoder(w).Encode(user)
}

// 中间件包装器
func loggingMiddleware(next http.Handler) http.Handler {
    return http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
        log.Printf("[%s] %s %s", r.Method, r.URL.Path, r.RemoteAddr)
        next.ServeHTTP(w, r)
    })
}

func main() {
    handler := &APIHandler{userService: NewUserService()}
    http.Handle("/api/", loggingMiddleware(handler))
    log.Fatal(http.ListenAndServe(":8080", nil))
}
```

### 5.3 Go 并发模型

Go 的 goroutine 和 channel 是语言层面提供的并发原语，比线程更轻量（初始栈仅 2KB）。

```go
// goroutine + channel 基础模式
func worker(id int, jobs <-chan Job, results chan<- Result) {
    for job := range jobs {
        results <- process(job)
    }
}

func main() {
    jobs := make(chan Job, 100)
    results := make(chan Result, 100)

    // 启动 10 个 worker
    for w := 1; w <= 10; w++ {
        go worker(w, jobs, results)
    }

    // 发送任务
    for _, job := range getAllJobs() {
        jobs <- job
    }
    close(jobs)

    // 收集结果
    for r := 1; r <= totalJobs; r++ {
        <-results
    }
}

// select 多路复用
func timeoutExample(ctx context.Context) {
    ch := make(chan string, 1)
    go func() {
        ch <- expensiveOperation()
    }()
    select {
    case result := <-ch:
        fmt.Println("成功:", result)
    case <-ctx.Done():
        fmt.Println("超时或被取消")
    case <-time.After(5 * time.Second):
        fmt.Println("操作超时 (>5s)")
    }
}
```

### 5.4 项目布局标准

Go 社区未强制项目结构，但 [project-layout](https://github.com/golang-standards/project-layout) 已成为事实标准：

```
my-service/
├── cmd/                    # 可执行文件入口
│   └── server/
│       └── main.go
├── internal/               # 私有包（外部不可导入）
│   ├── handler/           # HTTP 处理器
│   ├── service/           # 业务逻辑
│   ├── repository/        # 数据访问
│   ├── model/             # 领域模型
│   └── middleware/        # 中间件
├── pkg/                    # 可公开复用的包
│   ├── auth/
│   └── validator/
├── config/                 # 配置结构体
│   └── config.go
├── api/                    # API 定义（protobuf / OpenAPI）
│   └── openapi.yaml
├── migrations/             # 数据库迁移脚本
├── scripts/                # 构建和运维脚本
├── Makefile
├── go.mod
└── go.sum
```

---

## 六、Rust 后端

Rust 以其零成本抽象、所有权系统和内存安全保证，成为追求极致性能和可靠性的后端首选。

### 6.1 Axum — Tower 生态的现代框架

Axum 基于 Tower 中间件生态，与 Tokio 异步运行时深度集成，类型安全极强。

**路由与提取器**：

```rust
use axum::{
    extract::{Path, Query, State},
    http::StatusCode,
    response::Json,
    routing::{get, post},
    Router,
};
use serde::{Deserialize, Serialize};
use std::sync::Arc;
use sqlx::PgPool;

#[derive(Serialize)]
struct UserResponse {
    id: i32,
    username: String,
    email: String,
}

#[derive(Deserialize)]
struct CreateUserRequest {
    username: String,
    email: String,
    password: String,
}

#[derive(Clone)]
struct AppState {
    db: PgPool,
}

async fn get_user(
    State(state): State<Arc<AppState>>,
    Path(id): Path<i32>,
) -> Result<Json<UserResponse>, StatusCode> {
    let user = sqlx::query_as!(
        UserResponse,
        "SELECT id, username, email FROM users WHERE id = $1",
        id
    )
    .fetch_one(&state.db)
    .await
    .map_err(|_| StatusCode::NOT_FOUND)?;

    Ok(Json(user))
}

async fn create_user(
    State(state): State<Arc<AppState>>,
    Json(req): Json<CreateUserRequest>,
) -> Result<(StatusCode, Json<UserResponse>), StatusCode> {
    let user = sqlx::query_as!(
        UserResponse,
        "INSERT INTO users (username, email, password_hash) VALUES ($1, $2, $3) \
         RETURNING id, username, email",
        req.username,
        req.email,
        bcrypt::hash(req.password, bcrypt::DEFAULT_COST).unwrap(),
    )
    .fetch_one(&state.db)
    .await
    .map_err(|_| StatusCode::CONFLICT)?;

    Ok((StatusCode::CREATED, Json(user)))
}

#[tokio::main]
async fn main() {
    let pool = PgPool::connect("postgres://...").await.unwrap();
    let state = Arc::new(AppState { db: pool });

    let app = Router::new()
        .route("/api/users/:id", get(get_user))
        .route("/api/users", post(create_user))
        .layer(TraceLayer::new_for_http())
        .with_state(state);

    let listener = tokio::net::TcpListener::bind("0.0.0.0:8080").await.unwrap();
    axum::serve(listener, app).await.unwrap();
}
```

### 6.2 所有权与借用在后端实践

Rust 的所有权系统在后端开发中最常见的挑战是**共享可变状态**。模式如下：

```rust
use std::sync::Arc;
use tokio::sync::RwLock;

// 模式 1: Arc<RwLock<T>> — 读写分离的共享状态
struct Cache {
    inner: Arc<RwLock<HashMap<String, CachedValue>>>,
}

impl Cache {
    async fn get(&self, key: &str) -> Option<CachedValue> {
        self.inner.read().await.get(key).cloned()
    }

    async fn set(&self, key: String, value: CachedValue) {
        self.inner.write().await.insert(key, value);
    }
}

// 模式 2: 数据库连接池（Arc<Pool> — 最常用）
// Pool 内部实现了连接复用，Arc 让多个 handler 共享
type SharedPool = Arc<PgPool>;

// 模式 3: Actor 模式 — 通过 channel 序列化访问
enum Command {
    Increment,
    GetValue(u32),
}

async fn counter_actor(mut rx: mpsc::Receiver<Command>) {
    let mut count = 0u64;
    while let Some(cmd) = rx.recv().await {
        match cmd {
            Command::Increment => count += 1,
            Command::GetValue(tx) => { let _ = tx.send(count); }
        }
    }
}
```

### 6.3 零成本抽象

Rust 的零成本抽象意味着高级特性（泛型、trait、闭包、async/await）在编译后可以生成与手写优化代码等效的机器码。这对后端 HTTP 框架意义重大——Axum 的提取器链、中间件组合在运行时零开销。

```rust
// 编译期泛型派发（零成本）
trait Validator<T> {
    fn validate(&self, value: &T) -> Result<(), String>;
}

struct NonEmptyString;
impl Validator<String> for NonEmptyString {
    fn validate(&self, value: &String) -> Result<(), String> {
        if value.is_empty() { Err("不能为空".into()) } else { Ok(()) }
    }
}

// 使用静态派发（monomorphization），无虚表开销
fn process<T: Validator<String>>(v: String, validator: T) {
    validator.validate(&v).unwrap();
}
```

---

## 七、API 设计

### 7.1 RESTful 设计

**资源命名规范**：

| 资源 | URL | HTTP 方法 | 语义 |
|------|-----|-----------|------|
| 用户集合 | `GET /api/users` | GET | 获取用户列表 |
| 单个用户 | `GET /api/users/{id}` | GET | 获取指定用户 |
| 创建用户 | `POST /api/users` | POST | 新增用户 |
| 更新用户 | `PUT /api/users/{id}` | PUT | 全量替换用户 |
| 部分更新 | `PATCH /api/users/{id}` | PATCH | 部分字段更新 |
| 删除用户 | `DELETE /api/users/{id}` | DELETE | 删除指定用户 |
| 子资源 | `GET /api/users/{id}/orders` | GET | 获取用户的订单列表 |

**核心原则**：

- **名词复数命名**：`/users` 而非 `/getUser` 或 `/user`
- **层级关系用路径表示**：`/users/{id}/orders/{orderId}`
- **过滤/排序/分页用查询参数**：`?offset=0&limit=20&sort=-created_at`
- **统一响应结构**：

```json
{
  "success": true,
  "data": { ... },
  "error": null,
  "meta": { "page": 1, "total": 100 }
}
```

**HTTP 状态码速查**：

| 状态码 | 含义 | 典型场景 |
|--------|------|---------|
| 200 OK | 请求成功 | GET、PUT |
| 201 Created | 创建成功 | POST |
| 204 No Content | 成功但无返回体 | DELETE |
| 400 Bad Request | 请求参数错误 | 校验失败 |
| 401 Unauthorized | 未认证 | 缺少/无效 Token |
| 403 Forbidden | 已认证但无权限 | 角色不足 |
| 404 Not Found | 资源不存在 | ID 不存在 |
| 409 Conflict | 资源冲突 | 唯一约束违规 |
| 422 Unprocessable Entity | 语义错误 | 业务逻辑校验 |
| 429 Too Many Requests | 限流 | 超出速率限制 |
| 500 Internal Server Error | 服务器错误 | 未捕获异常 |

### 7.2 GraphQL

GraphQL 由客户端决定需要哪些字段，避免 Overfetching 和 Underfetching。

**Schema 定义**：

```graphql
type User {
  id: ID!
  username: String!
  email: String!
  articles: [Article!]!
  createdAt: DateTime!
}

type Article {
  id: ID!
  title: String!
  content: String!
  author: User!
  views: Int!
}

type Query {
  user(id: ID!): User
  users(page: Int, limit: Int): [User!]!
  search(query: String!): [SearchResult!]!
}

type Mutation {
  createUser(input: CreateUserInput!): User!
  updateUser(id: ID!, input: UpdateUserInput!): User!
  deleteUser(id: ID!): Boolean!
}

union SearchResult = User | Article
```

**N+1 问题与 DataLoader**：

```typescript
// DataLoader 批量合并查询
const UserLoader = new DataLoader<number, User>(async (ids) => {
  const users = await db.user.findMany({
    where: { id: { in: [...ids] } },
  });
  // 保持顺序与 ids 一致
  return ids.map((id) => users.find((u) => u.id === id) ?? null);
});

// 在 resolver 中
const resolvers = {
  Query: {
    user: (_, { id }) => UserLoader.load(id),
  },
};
```

### 7.3 gRPC

gRPC 基于 HTTP/2 和 Protocol Buffers，支持四种通信模式。

**Proto 定义**：

```protobuf
syntax = "proto3";

package codenebula;

service UserService {
  rpc GetUser (GetUserRequest) returns (User) {}
  rpc ListUsers (ListUsersRequest) returns (stream User) {}
  rpc CreateUser (stream CreateUserRequest) returns (User) {}
  rpc Chat (stream ChatMessage) returns (stream ChatMessage) {}
}

message User {
  int32 id = 1;
  string username = 2;
  string email = 3;
  string role = 4;
  google.protobuf.Timestamp created_at = 5;
}

message GetUserRequest {
  int32 id = 1;
}
```

**四种通信模式**：

| 模式 | 描述 | 适用场景 |
|------|------|---------|
| Unary | 客户端发 1 个请求，服务端回 1 个响应 | 标准 CRUD |
| Server Streaming | 客户端发 1 个请求，服务端流式返回多条 | 大列表、日志推送 |
| Client Streaming | 客户端流式发送多条，服务端汇总回 1 个 | 文件上传、批量写入 |
| Bidirectional Streaming | 双方独立发送流式数据 | 实时聊天、协同编辑 |

### 7.4 API 版本化策略

| 策略 | 方式 | 优点 | 缺点 |
|------|------|------|------|
| **URL 路径** | `/api/v1/users` | 简单直观，缓存友好 | 路径冗余 |
| **请求头** | `Accept: application/vnd.codenebula.v1+json` | URL 干净 | 调用方配置复杂 |
| **查询参数** | `?version=1` | 实现简单 | 容易被缓存污染 |
| **内容协商** | Content-Type 中包含版本 | 符合 RESTful 规范 | 调试不便 |

**推荐做法**：URL 路径版本化（最直观）配合优雅的版本迁移策略——旧版本标记 deprecated 并保留至少 6 个月。

### 7.5 OpenAPI / Swagger

```yaml
openapi: "3.0.0"
info:
  title: CodeNebula API
  version: "1.0.0"
paths:
  /api/users:
    get:
      summary: 获取用户列表
      parameters:
        - name: page
          in: query
          schema:
            type: integer
            default: 0
        - name: limit
          in: query
          schema:
            type: integer
            default: 20
      responses:
        "200":
          description: 用户列表
          content:
            application/json:
              schema:
                type: array
                items:
                  $ref: "#/components/schemas/User"
components:
  schemas:
    User:
      type: object
      properties:
        id:
          type: integer
        username:
          type: string
        email:
          type: string
          format: email
```

---

## 八、认证与授权

### 8.1 Session-based 认证

传统 Web 应用最常见的认证方式。用户登录后，服务端创建 Session，将 Session ID 通过 Cookie 返回给客户端。

```
客户端                         服务端
  │                              │
  │── POST /login ──────────────→│  验证用户名密码
  │                              │  创建 Session (存 Redis/DB)
  │←── Set-Cookie: session=abc ──│  Session ID → Cookie
  │                              │
  │── GET /profile ─────────────→│  从 Cookie 提取 Session ID
  │   Cookie: session=abc        │  从 Redis 查询 Session
  │                              │  返回用户信息
  │←── 响应 ────────────────────│
```

**优点**：服务端完全控制会话，注销即时生效。
**缺点**：需要持久化 Session 存储，不适合移动端和跨域场景。

### 8.2 JWT（JSON Web Token）

JWT 将用户信息编码在 Token 本身，服务端无需存储会话。

**JWT 结构**：`Header.Payload.Signature`

```json
{
  "header": {"alg": "HS256", "typ": "JWT"},
  "payload": {
    "sub": "1234567890",
    "name": "Alice",
    "role": "admin",
    "iat": 1516239022,
    "exp": 1516242622
  },
  "signature": "HMACSHA256(base64UrlEncode(header) + '.' + base64UrlEncode(payload), secret)"
}
```

**Access Token + Refresh Token 模式**：

```python
# JWT 生成与验证（Python 示例）
import jwt
from datetime import datetime, timedelta

SECRET = "your-secret-key"
REFRESH_SECRET = "your-refresh-secret"

def create_access_token(user_id: int, role: str) -> str:
    payload = {
        "sub": user_id,
        "role": role,
        "iat": datetime.utcnow(),
        "exp": datetime.utcnow() + timedelta(minutes=15),
    }
    return jwt.encode(payload, SECRET, algorithm="HS256")

def create_refresh_token(user_id: int) -> str:
    payload = {
        "sub": user_id,
        "iat": datetime.utcnow(),
        "exp": datetime.utcnow() + timedelta(days=7),
    }
    return jwt.encode(payload, REFRESH_SECRET, algorithm="HS256")

def verify_token(token: str) -> dict:
    try:
        return jwt.decode(token, SECRET, algorithms=["HS256"])
    except jwt.ExpiredSignatureError:
        raise Exception("Token 已过期")
    except jwt.InvalidTokenError:
        raise Exception("Token 无效")
```

**Token 黑名单**：JWT 一旦签发在过期前无法撤销。解决方案是维护一个 Redis 黑名单（TTL 设为 Token 的剩余有效期），在注销时将 Token 的 `jti` 加入黑名单。

### 8.3 OAuth 2.0

OAuth 2.0 解决的是**授权**问题——让第三方应用在用户授权下访问用户资源，而无需暴露用户密码。

**授权码流程（Authorization Code Flow）**——最安全的流程：

```
用户                   前端应用                  后端服务                 认证服务器
  │                      │                       │                       │
  │  1. 点击"微信登录"    │                       │                       │
  │─────────────────────→│                       │                       │
  │                      │  2. 重定向到认证服务器  │                       │
  │                      │─────────────────────────────────────────────→│
  │  3. 用户授权          │                       │                       │
  │←────────────────────────────────────────────│                       │
  │                      │  4. 重定向附带授权码    │                       │
  │                      │←─────────────────────────────────────────────│
  │                      │  5. 用授权码换 Token   │                       │
  │                      │─────────────────────────────────────────────→│
  │                      │  6. 返回 Access Token  │                       │
  │                      │←─────────────────────────────────────────────│
  │  7. 使用 Token 访问   │                       │                       │
  │─────────────────────→│──────────────────────→│                       │
```

**PKCE（Proof Key for Code Exchange）**——为纯前端应用（SPA）设计的增强。客户端先创建 `code_verifier` 和 `code_challenge`，授权码与 `code_challenge` 绑定，即使授权码被截获也无法换取 Token。

### 8.4 RBAC vs ABAC

| 模型 | 原理 | 优点 | 缺点 | 适用场景 |
|------|------|------|------|---------|
| **RBAC**（基于角色的访问控制） | 用户 → 角色 → 权限 | 简单直观，管理成本低 | 细粒度不足 | 大多数企业管理后台 |
| **ABAC**（基于属性的访问控制） | 用户属性 + 资源属性 + 环境条件 → 策略 | 灵活，支持动态上下文 | 策略复杂，性能开销大 | 金融、医疗等合规严格场景 |

**RBAC 实现示例**：

```python
# 角色权限矩阵
PERMISSIONS = {
    "admin":   {"user:create", "user:read", "user:update", "user:delete"},
    "editor":  {"user:read", "user:update"},
    "viewer":  {"user:read"},
}

def check_permission(user_role: str, required: str) -> bool:
    return required in PERMISSIONS.get(user_role, set())
```

**ABAC 策略示例（使用 Casbin）**：

```python
# 策略: 只允许作者编辑自己的文章
# sub, obj, act -> effect
# Alice, article:123, write -> allow (Alice 是 article:123 的作者)
```

---

## 九、消息队列

### 9.1 RabbitMQ vs Kafka vs Redis Pub/Sub

| 维度 | RabbitMQ | Kafka | Redis Pub/Sub |
|------|---------|-------|---------------|
| **模型** | 生产者/消费者 + 交换机/队列 | 生产者/消费者 + Topic/分区 | 发布/订阅 |
| **消息排序** | 单队列有序 | Topic 内分区有序 | 无序 |
| **消息持久化** | ✅ 支持 | ✅ 强大（磁盘顺序写入） | ❌ 默认不持久化 |
| **吞吐量** | 万级/秒 | 百万级/秒 | 十万级/秒 |
| **消息回溯** | ❌ 消费后删除 | ✅ 可回溯消费 | ❌ 不存储 |
| **路由灵活度** | 极高（多种 Exchange Type） | 低（基于 Topic） | 低（频道匹配） |
| **延迟** | 微秒级 | 毫秒级 | 微秒级 |
| **运维复杂度** | 中等 | 高 | 极低 |
| **适用场景** | 任务队列、RPC、事务消息 | 事件流、日志聚合、数据管道 | 实时通知、简单广播 |

**选择建议**：

- **任务分发 / RPC / 复杂路由** → RabbitMQ（灵活可靠）
- **高吞吐事件流 / 日志 / 数据管道** → Kafka（持久、可回溯）
- **轻量实时通知** → Redis Pub/Sub（极低延迟，适合 WebSocket 广播）

### 9.2 事件驱动架构

```
┌──────────┐  UserCreatedEvent  ┌──────────┐
│ User     │───────────────────→│ 事件总线  │
│ Service  │                    │ (Kafka)  │
└──────────┘                    └──────────┘
                                      │
                    ┌─────────────────┼─────────────────┐
                    │                 │                  │
                    ▼                 ▼                  ▼
              ┌──────────┐    ┌──────────┐     ┌──────────┐
              │ Email    │    │ Analytics│     │ 通知     │
              │ Service  │    │ Service  │     │ Service  │
              └──────────┘    └──────────┘     └──────────┘
```

每个服务处理自己的领域事件，通过事件总线异步通信，服务之间松耦合。

### 9.3 CQRS 模式

CQRS（Command Query Responsibility Segregation）将读写模型分离：

```
Command Side (写)                     Query Side (读)
┌──────────────────┐                ┌──────────────────┐
│ 写操作 → Command  │                │ 读操作 → Query    │
│ UserService.Create│                │ UserService.Find  │
│      ↓           │                │      ↑           │
│ 写模型 (规范化)    │                │ 读模型 (反规范化)  │
│ RDBMS (3NF)      │                │ Redis / ES / MV   │
│      ↓           │                │      ↑           │
└──────────────────┘                └──────────────────┘
        ↓ 同步事件 (最终一致) ↑
   └─────────────────────────┘
```

> **适用判断**：CQRS 只在读写差异极大的场景下有价值（如写少读多的报表系统）。绝大多数 CRUD 应用不需要 CQRS——它会引入不必要的复杂度。

---

## 十、安全实践

### 10.1 SQL 注入防护

SQL 注入是历史最悠久也最常见的 Web 漏洞。**核心原则：永远不要拼接 SQL 字符串**。

```python
# ❌ 错误 — 字符串拼接
cursor.execute(f"SELECT * FROM users WHERE email = '{email}'")

# ✅ 正确 — 参数化查询
cursor.execute("SELECT * FROM users WHERE email = %s", (email,))

# ✅ ORM 更安全
User.objects.filter(email=email)
```

**常见防护措施**：

- 始终使用参数化查询（Prepared Statements）
- ORM 框架自带注入防护
- 对动态表名、ORDER BY 列名等无法参数化的部分做白名单校验
- 最小化数据库用户权限（应用账号不应有 `DROP TABLE` 权限）

### 10.2 XSS / CSRF 防护

**XSS（跨站脚本攻击）**：攻击者注入恶意脚本到页面中。

| XSS 类型 | 描述 | 防护 |
|----------|------|------|
| **存储型** | 恶意脚本存储在服务器（如评论区） | 输出编码、CSP |
| **反射型** | 恶意脚本在 URL 参数中 | 输入校验 + 输出编码 |
| **DOM 型** | 前端 JS 动态创建 DOM 时触发 | 避免使用 `innerHTML`，用 `textContent` |

```python
# Flask 中默认启用 Jinja2 自动转义
# {{ user_input }}  → 自动转义 HTML
# 如果需要输出 HTML，明确使用 |safe 但需确保内容安全
```

**CSRF（跨站请求伪造）**：攻击者诱导用户点击链接，在用户已登录状态下执行非自愿操作。

```python
# Django — 内置 CSRF 保护
# 模板中: {% csrf_token %}
# 在 POST 表单时，Django 自动验证 CSRF Token

# FastAPI — 使用 Starlette 的 SessionMiddleware + CSRF token
```

**CSP（Content Security Policy）**——浏览器级安全防线：

```
Content-Security-Policy: default-src 'self'; script-src 'self' cdn.example.com; style-src 'self' 'unsafe-inline'
```

### 10.3 速率限制（Rate Limiting）

```python
# FastAPI + slowapi 实现速率限制
from slowapi import Limiter, _rate_limit_exceeded_handler
from slowapi.util import get_remote_address

limiter = Limiter(key_func=get_remote_address)
app.state.limiter = limiter
app.add_exception_handler(429, _rate_limit_exceeded_handler)

@app.get("/api/users")
@limiter.limit("100/minute")
async def list_users(request: Request):
    return {"users": [...]}

# 更细粒度的限制
@limiter.limit("20/minute")
@app.post("/api/auth/login")
async def login(request: Request):
    pass  # 登录接口限流更严格
```

**常用限流算法**：

| 算法 | 原理 | 特点 |
|------|------|------|
| **令牌桶（Token Bucket）** | 固定速率生成令牌，消费需令牌 | 允许突发流量 |
| **漏桶（Leaky Bucket）** | 请求按固定速率流出 | 平滑流量，无突发 |
| **滑动窗口（Sliding Window）** | 统计窗口内请求数 | 边界平滑，应用最广 |
| **固定窗口（Fixed Window）** | 按固定时间窗口计数 | 边界尖峰问题 |

### 10.4 输入验证

**第一道防线永远是服务端验证**，前端验证仅用于提升用户体验。

```python
# Pydantic 示例 — 多层验证
from pydantic import BaseModel, Field, EmailStr, validator
from typing import Optional

class RegisterRequest(BaseModel):
    username: str = Field(..., min_length=3, max_length=30,
                          pattern=r"^[a-zA-Z0-9_]+$")
    email: EmailStr
    password: str = Field(..., min_length=8, max_length=128)
    age: Optional[int] = Field(None, ge=0, le=150)

    @validator("password")
    def password_strength(cls, v):
        if not any(c.isupper() for c in v):
            raise ValueError("密码必须包含大写字母")
        if not any(c.isdigit() for c in v):
            raise ValueError("密码必须包含数字")
        return v
```

**输入验证清单**：

- 类型校验（字符串、数字、布尔等）
- 长度/范围约束
- 格式校验（邮箱、URL、手机号、日期等）
- 业务规则校验（如密码强度）
- 白名单校验（枚举值、允许的字符集）
- 防止 Prototype Pollution（JS）、Zip Slip（文件解压）等特殊漏洞

### 10.5 HTTPS 与 TLS

```
TLS 1.3 握手（1-RTT，简化版）:
Client                  Server
  │                       │
  │── ClientHello ──────→│  支持的密码套件
  │←── ServerHello ──────│  选择的套件 + 证书
  │←── Certificate ─────│  服务器证书（含公钥）
  │── Finished ────────→│  生成会话密钥
  │←── Finished ───────│
  │── 加密的应用数据 ──→│
```

**最佳实践**：

- 使用 Let's Encrypt 免费 TLS 证书
- 禁用 TLS 1.0/1.1，仅启用 TLS 1.2/1.3
- 使用现代密码套件：`TLS_AES_128_GCM_SHA256` / `TLS_AES_256_GCM_SHA384`
- HSTS 头部：`Strict-Transport-Security: max-age=31536000; includeSubDomains`
- 全站 HTTPS + 自动 HTTP → HTTPS 重定向

---

> **本文是 CodeNebula 后端开发知识体系的完整章节。下一章：[数据库与存储 →](./03-database.md)** — 从 SQL 优化到 NoSQL 选型，构建数据层深度认知。

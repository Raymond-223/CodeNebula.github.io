# 软件架构与设计模式

> 软件架构是系统的骨骼——它决定了系统能长多大、能跑多快、能活多久。本章从架构设计思想出发，覆盖架构风格、GoF 设计模式、企业集成、分布式系统设计、系统设计方法论和 API 网关六大主题，为构建高质量软件系统提供系统性参考。

---

## 一、架构设计思想

### 1.1 什么是软件架构

软件架构描述的是系统的**结构（Structure）**、**行为（Behavior）** 和**属性（Properties）**。它不是某个具体的实现细节，而是系统的宏观蓝图。

| 维度 | 定义 | 实例 |
|------|------|------|
| **结构** | 系统的组件及其关系 | 模块划分、服务分层、依赖方向 |
| **行为** | 组件之间的交互与协作 | 消息传递、RPC 调用、事件通知 |
| **属性** | 系统层面的非功能性特征 | 性能、可扩展性、安全性 |

IEEE 1471 标准将架构定义为："系统在其环境中的基本概念或属性，体现在其元素、关系以及设计与演进原则中。"

### 1.2 质量属性（Quality Attributes）

架构的质量决定了系统的**生存能力**。以下是关键质量属性及其权衡：

| 质量属性 | 定义 | 典型指标 | 常见冲突 |
|----------|------|----------|---------|
| **可扩展性（Scalability）** | 通过增加资源提升吞吐的能力 | QPS、并发用户数 | 与一致性冲突 |
| **可用性（Availability）** | 系统正常服务的时间比例 | 99.9%（3个9）→ 99.999%（5个9） | 与一致性冲突 |
| **可维护性（Maintainability）** | 修复缺陷和添加功能的成本 | MTTR、代码复杂度 | 与性能有时冲突 |
| **性能（Performance）** | 响应速度和吞吐量 | P50/P95/P99 延迟 | 与可扩展性部分冲突 |
| **安全性（Security）** | 防御未授权访问和攻击 | 渗透测试结果、认证强度 | 增加性能开销 |
| **可测试性（Testability）** | 系统可被验证的容易程度 | 代码覆盖率、测试执行时间 | 可能增加复杂度 |

> **架构即权衡**：没有完美的架构，只有针对特定场景的最优取舍。CAP 定理、一致性模型选择、缓存策略——本质上都在做权衡决策。

### 1.3 架构 vs 设计

架构和设计是同一枚硬币的两面，但关注点不同：

| 维度 | 架构（Architecture） | 设计（Design） |
|------|---------------------|---------------|
| **范围** | 系统级别、宏观 | 模块级别、微观 |
| **关注点** | 组件划分、通信方式、部署策略 | 类结构、算法、数据结构 |
| **决策影响** | 跨团队、跨模块、难变更 | 局部范围、相对容易重构 |
| **时机** | 项目初期必须确定 | 可随迭代逐步细化 |
| **经典提问** | "我们用单体还是微服务？" | "这个类用继承还是组合？" |

> **重要认知**：架构决策通常是**不可逆**或**高成本逆转**的。选择微服务后想切回单体，几乎等于重写系统。

### 1.4 架构驱动因素（Architectural Drivers）

架构决策不是随意的——它由三个因素驱动：

**1. 功能需求（Functional Requirements）**
- 系统必须做什么
- 直接影响组件划分：电商系统要有订单、支付、库存模块

**2. 质量属性（Quality Attributes）**
- 系统必须做到什么程度
- 决定了架构风格选择：

```text
QPS < 100, 单体够用
QPS 100-1000, 水平扩展 + 缓存
QPS > 10000, 微服务 + 分片 + CQRS
```

**3. 约束（Constraints）**
- 不可变更的外部限制
- 例如：必须使用 Java、必须部署在客户私有云、预算限制

> **架构决策矩阵**：每次架构决策时，列出选项，按驱动因素评分，选择综合最优方案。

---

## 二、架构风格

### 2.1 单体架构（Monolithic Architecture）

将所有功能打包到单个可部署单元中：

```text
┌──────────────────────────────────────┐
│           Monolith App               │
│  ┌────────┐ ┌────────┐ ┌────────┐   │
│  │ 用户模块│ │ 订单模块│ │ 支付模块│   │
│  └────────┘ └────────┘ └────────┘   │
│  ┌────────┐ ┌────────┐ ┌────────┐   │
│  │ 库存模块│ │ 通知模块│ │ 管理模块│   │
│  └────────┘ └────────┘ └────────┘   │
│         Database (单库)              │
└──────────────────────────────────────┘
```

**优点**：
- 开发简单：单一代码库、单一部署
- 测试容易：端到端测试无需跨服务
- 部署快速：构建一次即可
- 事务简单：ACID 事务天然支持

**缺点**：
- 耦合严重：模块间紧耦合，牵一发而动全身
- 扩展粒度过粗：只能整体扩展，即使只有支付模块是瓶颈
- 技术锁定：所有模块必须使用相同的技术栈
- 部署风险大：任何小改动都需全量部署

**适用场景**：
- 初创项目 / MVP 阶段
- 团队规模小（<10 人）
- 系统复杂度低，预期增长有限
- 对可用性要求不高（允许停机部署）

### 2.2 分层架构（Layered Architecture）

最经典的架构风格，将系统按职责垂直切分：

```text
┌──────────────────────────────────────────┐
│  展现层（Presentation）                    │
│  REST API / GraphQL / gRPC Endpoints     │
├──────────────────────────────────────────┤
│  业务逻辑层（Business Logic）              │
│  Service / UseCase / Domain Objects      │
├──────────────────────────────────────────┤
│  持久化层（Persistence）                   │
│  Repository / DAO / ORM / Data Mapper    │
├──────────────────────────────────────────┤
│  基础设施层（Infrastructure）              │
│  DB / Cache / Queue / File System        │
└──────────────────────────────────────────┘
```

**关键原则**：
- **上层依赖下层**：展现层调用业务层，业务层调用持久化层
- **依赖倒置**：通过接口解耦，业务层定义接口，持久化层实现
- **严格分层 vs 开放分层**：严格分层禁止跨层调用（展现层→持久化层），开放分层允许适度跨越

**优点**：关注点分离清晰、团队分工明确、技术栈解耦
**缺点**：分层过多导致"胶水代码"膨胀、单体趋势明显

### 2.3 六边形架构（Hexagonal / Ports & Adapters）

由 Alistair Cockburn 提出，核心思想是**业务逻辑对外部世界一无所知**：

```text
       ┌─────────────────────┐
       │      Web Adapter     │
       └──────┬──────────────┘
              │ HTTP
       ┌──────▼──────────────┐
       │     Inbound Port     │  ← 接口定义
       │ ┌─────────────────┐ │
       │ │   Domain Core   │ │  ← 纯业务逻辑
       │ └─────────────────┘ │
       │    Outbound Port    │  ← 接口定义
       └──────┬──────────────┘
              │
  ┌───────────┼───────────────┐
  │           │               │
  ▼           ▼               ▼
DB Adapter  Queue Adapter   Cache Adapter
```

**核心优势**：
- 领域模型完全独立于基础设施
- 替换数据库、消息队列等无需改业务代码
- 单元测试友好：端口可轻松 mock

**代码示例**：

```java
// Inbound Port
public interface OrderService {
    Order createOrder(CreateOrderRequest request);
}

// Domain Core
public class OrderServiceImpl implements OrderService {
    private final OrderRepository repo; // Outbound Port
    private final PaymentGateway payment; // Outbound Port
    
    public Order createOrder(CreateOrderRequest req) {
        Order order = Order.create(req.userId(), req.items());
        payment.charge(req.userId(), order.total());
        return repo.save(order);
    }
}

// Outbound Port
public interface OrderRepository {
    Order save(Order order);
}

// Adapter (JPA 实现)
@Repository
public class JpaOrderRepository implements OrderRepository {
    private final JpaOrderEntityRepo jpaRepo;
    
    public Order save(Order order) {
        OrderEntity entity = OrderEntity.fromDomain(order);
        return jpaRepo.save(entity).toDomain();
    }
}
```

### 2.4 微服务架构（Microservices）

将系统拆分为一组**独立部署**、**围绕业务能力组织**的服务：

**核心特征**：
- 每个服务拥有自己的数据库（Database per Service）
- 服务间通过轻量级通信（HTTP/REST、gRPC、消息队列）
- 独立部署、独立扩展、独立技术栈
- 围绕业务领域（Bounded Context）划分

**优点**：
| 维度 | 收益 |
|------|------|
| 独立部署 | 一个服务改动不影响其他服务 |
| 独立扩展 | 只为瓶颈服务增加资源 |
| 技术多样性 | 不同服务可用不同语言/数据库 |
| 故障隔离 | 一个服务崩溃不会拖垮整个系统 |
| 团队自治 | 每个团队拥有自己的服务 |

**挑战**：
| 维度 | 成本 |
|------|------|
| 分布式复杂性 | 网络延迟、部分失败、数据一致性 |
| 运维成本 | 服务数量增加导致监控、日志、部署复杂度指数增长 |
| 数据一致性 | 分布式事务比 ACID 事务复杂得多 |
| 测试难度 | 端到端测试需要启动多个服务 |
| 团队要求 | 需要 DevOps 文化、容器编排、CI/CD 成熟度 |

**什么情况下不应该用微服务**：
- 团队 < 20 人 → 单体 + 模块化即可
- 业务逻辑高度耦合 → 强行拆分导致跨服务调用激增
- 数据一致性要求极高 → 分布式事务是巨大的技术债
- 运维能力不足 → 没有 Kubernetes / CI/CD 经验前不要尝试

> **Martin Fowler 的忠告**："不要一开始就用微服务。先建一个模块化的单体（Modular Monolith），当单体确实撑不住了再考虑拆分。"

### 2.5 事件驱动架构（Event-Driven Architecture, EDA）

组件通过**事件**进行异步通信，生产者发布事件，消费者订阅事件：

```text
┌──────────┐     ┌──────────────┐     ┌──────────┐
│ 订单服务  │────→│              │────→│ 通知服务  │
└──────────┘     │              │     └──────────┘
                 │  Event Bus   │     ┌──────────┐
┌──────────┐     │  (Kafka/     │────→│ 库存服务  │
│ 支付服务  │────→│   RabbitMQ)  │     └──────────┘
└──────────┘     │              │     ┌──────────┐
                 │              │────→│ 分析服务  │
                 └──────────────┘     └──────────┘
```

#### 事件溯源（Event Sourcing）

将状态变更存储为事件序列，而非当前状态：

```text
传统方式:   用户余额 = $100 （覆盖写）
事件溯源:   [开户$0] → [存入$100] → [消费$30] → [存入$50]
            当前余额 = 0 + 100 - 30 + 50 = $120
```

**优点**：完整审计轨迹、时间旅行调试、事件天然可复现
**缺点**：查询复杂（需遍历事件流）、存储量巨大、学习曲线陡峭

#### CQRS（Command Query Responsibility Segregation）

命令（写）和查询（读）使用不同的模型：

```text
Client
  │
  ├── Command: POST /orders  ──→ Command Model ──→ DB (写库)
  │                               │
  │                               └── Event ──→ Sync ──→ Read DB
  │                                                        │
  └── Query: GET /orders/123 ──→ Query Model ←─────────────┘
```

**适用**：读写负载差异巨大的系统（如写少读多、或写多读少）

#### Saga 模式

管理跨服务分布式事务的机制，有两种实现方式：

**编排式（Choreography）**：每个服务监听事件并执行本地事务 + 发布新事件

```text
订单服务: 创建订单 → 发布"订单已创建"事件
  ↓
支付服务: 监听 → 扣款 → 发布"支付成功"事件
  ↓
库存服务: 监听 → 扣库存 → 发布"库存已扣"事件
  ↓
通知服务: 监听 → 发送确认邮件
```

**若某步骤失败**：发布补偿事件回滚之前的操作。

**编排式（Orchestration）**：由 Saga 协调器统一指挥

```text
Saga Coordinator
  1. 调用 Order Service → 创建订单
  2. 调用 Payment Service → 扣款（失败则执行补偿: 取消订单）
  3. 调用 Inventory Service → 扣库存（失败则执行补偿: 退款+取消订单）
```

### 2.6 无服务架构（Serverless）

开发者专注于业务代码，无需管理服务器：

```text
Client ──→ API Gateway ──→ Lambda ──→ DynamoDB
                              │
                              └── Lambda (后台处理)
```

| 维度 | 说明 |
|------|------|
| **计费模型** | 按调用次数 + 执行时间计费 |
| **扩展** | 自动伸缩，零流量时缩到零 |
| **冷启动** | 函数首次调用有延迟（几百毫秒） |
| **限制** | 执行时长限制（通常 15min）、内存限制 |
| **适用** | 定时任务、Webhook、数据处理管道、轻量 API |

**不适合**：长连接、高延迟敏感、大内存需求的应用。

### 2.7 架构风格对比总结

| 风格 | 优点 | 缺点 | 适用规模 | 复杂度 |
|------|------|------|---------|-------|
| **单体** | 简单、快速开发 | 扩展性差、耦合高 | 小（<10人） | ⭐ |
| **分层** | 关注点分离、团队分工 | 可能臃肿 | 中 | ⭐⭐ |
| **六边形** | 领域独立、测试友好 | 抽象层多 | 中 | ⭐⭐⭐ |
| **微服务** | 独立扩展、独立部署 | 分布式复杂性 | 大（>50人） | ⭐⭐⭐⭐⭐ |
| **事件驱动** | 松耦合、异步 | 一致性难保证 | 中大 | ⭐⭐⭐⭐ |
| **Serverless** | 零运维、自动伸缩 | 冷启动、限制多 | 小中 | ⭐⭐ |

---

## 三、设计模式（GoF 23 种模式）

设计模式是**可复用的解决方案**，应对特定上下文中的常见设计问题。以下按三大分类逐一讲解，并附代码示例。

### 3.1 创建型模式（Creational Patterns）

#### 单例模式（Singleton）
保证一个类只有一个实例，并提供全局访问点。

```java
// 双重检查锁定（Double-Checked Locking）— 线程安全
public class DatabaseConnectionPool {
    private static volatile DatabaseConnectionPool instance;
    
    private DatabaseConnectionPool() { /* 私有构造函数 */ }
    
    public static DatabaseConnectionPool getInstance() {
        if (instance == null) {
            synchronized (DatabaseConnectionPool.class) {
                if (instance == null) {
                    instance = new DatabaseConnectionPool();
                }
            }
        }
        return instance;
    }
}
```

> **注意**：单例模式容易引入全局状态，增加测试难度。现代实践中倾向于用依赖注入容器管理单例生命周期。

#### 工厂方法（Factory Method）
定义创建对象的接口，让子类决定实例化哪个类。

```java
// 接口
interface Document {
    void open();
    void save();
}

// 具体产品
class PdfDocument implements Document { /* ... */ }
class WordDocument implements Document { /* ... */ }

// 工厂
abstract class DocumentCreator {
    abstract Document createDocument(); // 工厂方法
    
    public void process(String fileName) {
        Document doc = createDocument();
        doc.open();
        // 处理文档...
        doc.save();
    }
}

class PdfCreator extends DocumentCreator {
    Document createDocument() { return new PdfDocument(); }
}

class WordCreator extends DocumentCreator {
    Document createDocument() { return new WordDocument(); }
}
```

#### 抽象工厂（Abstract Factory）
创建相关或依赖对象的家族，而不指定具体类。

```java
interface GUIFactory {
    Button createButton();
    Checkbox createCheckbox();
}

class WindowsFactory implements GUIFactory {
    public Button createButton() { return new WindowsButton(); }
    public Checkbox createCheckbox() { return new WindowsCheckbox(); }
}

class MacFactory implements GUIFactory {
    public Button createButton() { return new MacButton(); }
    public Checkbox createCheckbox() { return new MacCheckbox(); }
}

// 使用
GUIFactory factory = new WindowsFactory();
Button btn = factory.createButton(); // Windows 风格按钮
```

#### 构建者模式（Builder）
分步构造复杂对象，允许用相同的构建过程创建不同的表示。

```java
// 链式调用
Order order = new Order.Builder()
    .withUserId(123)
    .withItem(new Item("Laptop", 1, 9999.00))
    .withItem(new Item("Mouse", 2, 199.00))
    .withShippingAddress("北京市朝阳区...")
    .withCoupon("SUMMER2025")
    .build();
```

#### 原型模式（Prototype）
通过复制已有对象来创建新对象，避免昂贵的构造函数调用。

```java
abstract class Shape implements Cloneable {
    protected String type;
    abstract void draw();
    
    public Shape clone() {
        return (Shape) super.clone(); // 浅拷贝
    }
}

class Circle extends Shape {
    private int radius;
    
    Circle() { 
        this.type = "Circle";
        this.radius = 10; // 默认值
    }
    
    Circle(Circle source) { // 深拷贝构造函数
        this.radius = source.radius;
    }
    
    void draw() { /* ... */ }
    
    @Override
    public Circle clone() {
        return new Circle(this); // 调用深拷贝构造函数
    }
}
```

### 3.2 结构型模式（Structural Patterns）

#### 适配器模式（Adapter）
将一个类的接口转换成客户端期望的另一个接口。

```java
// 目标接口（客户端期望的）
interface JsonApi {
    String toJson();
}

// 被适配者（第三方库）
class XmlResponse {
    public String toXml() { return "<data>...</data>"; }
}

// 适配器
class XmlToJsonAdapter implements JsonApi {
    private XmlResponse xmlResponse;
    
    XmlToJsonAdapter(XmlResponse xmlResponse) {
        this.xmlResponse = xmlResponse;
    }
    
    public String toJson() {
        String xml = xmlResponse.toXml();
        return convertXmlToJson(xml); // 适配逻辑
    }
    
    private String convertXmlToJson(String xml) {
        // XML → JSON 转换
        return "{\"data\":\"...\"}";
    }
}
```

#### 桥接模式（Bridge）
将抽象部分与实现部分分离，使它们可以独立变化。

```java
// 实现接口
interface Device {
    void turnOn();
    void turnOff();
    void setVolume(int percent);
}

class TV implements Device { /* ... */ }
class Radio implements Device { /* ... */ }

// 抽象
abstract class RemoteControl {
    protected Device device;
    
    RemoteControl(Device device) { this.device = device; }
    
    abstract void togglePower();
    abstract void volumeUp();
    abstract void volumeDown();
}

class BasicRemote extends RemoteControl {
    BasicRemote(Device device) { super(device); }
    
    void togglePower() { /* 切换电源 */ }
    void volumeUp() { device.setVolume(device.getVolume() + 10); }
    void volumeDown() { device.setVolume(device.getVolume() - 10); }
}
```

#### 组合模式（Composite）
将对象组合成树形结构以表示"部分-整体"的层次结构。

```java
interface Component {
    double getPrice();
}

class Product implements Component { // 叶子节点
    private String name;
    private double price;
    
    public double getPrice() { return price; }
}

class Box implements Component { // 组合节点
    private List<Component> items = new ArrayList<>();
    
    public void add(Component item) { items.add(item); }
    
    public double getPrice() {
        return items.stream().mapToDouble(Component::getPrice).sum();
    }
}

// 使用
Box mainBox = new Box();
mainBox.add(new Product("笔记本电脑", 8999));
mainBox.add(new Product("鼠标", 199));

Box accessoriesBox = new Box();
accessoriesBox.add(new Product("键盘", 399));
accessoriesBox.add(new Product("鼠标垫", 29));
mainBox.add(accessoriesBox);

double total = mainBox.getPrice(); // 8999 + 199 + 399 + 29 = 9626
```

#### 装饰器模式（Decorator）
动态地为对象添加新功能，是继承的灵活替代方案。

```java
interface Coffee {
    double cost();
    String description();
}

class Espresso implements Coffee {
    public double cost() { return 25.0; }
    public String description() { return "浓缩咖啡"; }
}

// 装饰器
abstract class CoffeeDecorator implements Coffee {
    protected Coffee coffee;
    
    CoffeeDecorator(Coffee coffee) { this.coffee = coffee; }
}

class Milk extends CoffeeDecorator {
    Milk(Coffee coffee) { super(coffee); }
    
    public double cost() { return coffee.cost() + 5.0; }
    public String description() { return coffee.description() + " + 牛奶"; }
}

class Whip extends CoffeeDecorator {
    Whip(Coffee coffee) { super(coffee); }
    
    public double cost() { return coffee.cost() + 3.0; }
    public String description() { return coffee.description() + " + 奶油"; }
}

// 使用：浓缩 + 牛奶 + 奶油
Coffee coffee = new Whip(new Milk(new Espresso()));
System.out.println(coffee.description() + " = ¥" + coffee.cost());
// 输出: 浓缩咖啡 + 牛奶 + 奶油 = ¥33.0
```

#### 外观模式（Facade）
为复杂子系统提供统一的高层接口。

```java
class OrderFacade {
    private InventoryService inventory;
    private PaymentService payment;
    private ShippingService shipping;
    private NotificationService notification;
    
    public OrderResult placeOrder(OrderRequest request) {
        // 1. 检查库存
        if (!inventory.checkStock(request.items())) {
            return OrderResult.failure("库存不足");
        }
        // 2. 扣款
        PaymentResult payResult = payment.charge(request.userId(), request.total());
        if (!payResult.success()) {
            return OrderResult.failure("支付失败: " + payResult.error());
        }
        // 3. 扣库存
        inventory.deduct(request.items());
        // 4. 创建物流
        String trackingId = shipping.createShipment(request.address());
        // 5. 发送通知
        notification.sendConfirmation(request.userId(), trackingId);
        
        return OrderResult.success(trackingId);
    }
}
```

#### 享元模式（Flyweight）
通过共享大量细粒度对象来减少内存使用。

```java
// 享元对象 — 内部状态可共享
class TreeType {
    private final String name;
    private final String color;
    private final String texture; // 纹理图片URL
    
    TreeType(String name, String color, String texture) {
        this.name = name;
        this.color = color;
        this.texture = texture;
    }
    
    void draw(int x, int y) { /* 在 (x,y) 绘制此类型的树 */ }
}

// 工厂
class TreeFactory {
    static Map<String, TreeType> types = new HashMap<>();
    
    static TreeType getTreeType(String name, String color, String texture) {
        String key = name + "|" + color;
        return types.computeIfAbsent(key, k -> new TreeType(name, color, texture));
    }
}

// 使用：内存中只存了少数 TreeType 实例，但可以有成千上万个位置
for (int i = 0; i < 1000000; i++) {
    TreeType type = TreeFactory.getTreeType("松树", "深绿", "pine.png");
    // type.draw(randomX, randomY); — 外部状态通过参数传递
}
```

#### 代理模式（Proxy）
为另一个对象提供替身，控制对其的访问。

```java
// 远程代理
interface Image {
    void display();
}

class RealImage implements Image {
    private String filename;
    
    RealImage(String filename) {
        this.filename = filename;
        loadFromDisk(); // 昂贵的操作
    }
    
    private void loadFromDisk() { /* 从磁盘加载 */ }
    public void display() { /* 显示图片 */ }
}

class ImageProxy implements Image {
    private String filename;
    private RealImage realImage; // 延迟初始化
    
    ImageProxy(String filename) { this.filename = filename; }
    
    public void display() {
        if (realImage == null) {
            realImage = new RealImage(filename); // 首次调用时加载
        }
        realImage.display();
    }
}
```

### 3.3 行为型模式（Behavioral Patterns）

#### 观察者模式（Observer）
定义一对多依赖关系，状态变化时通知所有依赖者。

```java
// 主题
interface Subject {
    void attach(Observer observer);
    void detach(Observer observer);
    void notifyObservers();
}

class WeatherStation implements Subject {
    private List<Observer> observers = new ArrayList<>();
    private float temperature;
    
    public void attach(Observer o) { observers.add(o); }
    public void detach(Observer o) { observers.remove(o); }
    
    public void setTemperature(float temp) {
        this.temperature = temp;
        notifyObservers();
    }
    
    public void notifyObservers() {
        for (Observer o : observers) {
            o.update(temperature);
        }
    }
}

// 观察者
interface Observer {
    void update(float temperature);
}

class PhoneDisplay implements Observer {
    public void update(float temp) { System.out.println("手机显示温度: " + temp); }
}

class WebDisplay implements Observer {
    public void update(float temp) { System.out.println("网页显示温度: " + temp); }
}
```

#### 策略模式（Strategy）
定义一组算法，将它们封装并使它们可以互相替换。

```java
// 策略接口
interface PaymentStrategy {
    void pay(double amount);
}

class CreditCardPayment implements PaymentStrategy {
    private String cardNumber;
    
    public void pay(double amount) { /* 信用卡支付 */ }
}

class AlipayPayment implements PaymentStrategy {
    public void pay(double amount) { /* 支付宝支付 */ }
}

class WechatPayment implements PaymentStrategy {
    public void pay(double amount) { /* 微信支付 */ }
}

// 上下文
class ShoppingCart {
    private PaymentStrategy paymentStrategy;
    
    void setPaymentStrategy(PaymentStrategy strategy) {
        this.paymentStrategy = strategy;
    }
    
    void checkout(double total) {
        paymentStrategy.pay(total);
    }
}

// 使用
ShoppingCart cart = new ShoppingCart();
cart.setPaymentStrategy(new AlipayPayment());
cart.checkout(299.00);
```

> **现代替代**：函数式接口（Java）或高阶函数（JavaScript/Python）可以直接替代策略模式：
> ```java
> // Java 函数式替代
> @FunctionalInterface
> interface PaymentStrategy { void pay(double amount); }
> 
> cart.setPaymentStrategy(amt -> System.out.println("支付宝支付: " + amt));
> ```

#### 命令模式（Command）
将请求封装为对象，支持参数化、队列、日志和撤销操作。

```java
interface Command {
    void execute();
    void undo();
}

class LightOnCommand implements Command {
    private Light light;
    
    LightOnCommand(Light light) { this.light = light; }
    
    public void execute() { light.on(); }
    public void undo() { light.off(); }
}

class CommandInvoker {
    private Stack<Command> history = new Stack<>();
    
    void executeCommand(Command cmd) {
        cmd.execute();
        history.push(cmd);
    }
    
    void undoLastCommand() {
        if (!history.isEmpty()) {
            Command cmd = history.pop();
            cmd.undo();
        }
    }
}
```

#### 模板方法模式（Template Method）
在父类中定义算法骨架，让子类重写特定步骤。

```java
abstract class DataMiner {
    // 模板方法 — 定义算法骨架
    public final void mine(String path) {
        openFile(path);
        extractData();
        parseData();
        analyzeData();
        sendReport();
        closeFile();
    }
    
    abstract void openFile(String path);
    abstract void extractData();
    abstract void parseData();
    
    void analyzeData() { /* 默认实现 */ }
    void sendReport() { System.out.println("发送报告"); }
    void closeFile() { /* 通用实现 */ }
}

class PdfDataMiner extends DataMiner {
    void openFile(String path) { /* 用 PDF 阅读器打开 */ }
    void extractData() { /* 从 PDF 提取原始文本 */ }
    void parseData() { /* 解析 PDF 格式 */ }
}

class CsvDataMiner extends DataMiner {
    void openFile(String path) { /* 用文本阅读器打开 */ }
    void extractData() { /* 逐行读取 */ }
    void parseData() { /* 按逗号分隔 */ }
}
```

### 3.4 现代替代方案

| 传统模式 | 现代替代 | 说明 |
|---------|---------|------|
| **Factory / Abstract Factory** | **依赖注入（DI）** | Spring / Guice 容器接管对象创建，无需显式工厂 |
| **Strategy** | **函数式接口 / Lambda** | Java `Function<T,R>`、Python 高阶函数 |
| **Observer** | **响应式流（Reactive Streams）** | RxJava / Project Reactor / WebFlux |
| **Command** | **消息队列 / 事件驱动** | Kafka / RabbitMQ 天然支持命令排队和执行 |
| **Iterator** | **Stream API** | Java Stream / LINQ / Python 生成器 |
| **Singleton** | **DI 容器管理的单例** | Spring `@Scope("singleton")` |

---

## 四、企业集成模式

### 4.1 消息模式

| 模式 | 目的 | 示例 |
|------|------|------|
| **Command Message** | 调用远程过程 / 执行操作 | 订单创建请求 |
| **Document Message** | 传递数据 | 客户资料同步 |
| **Event Message** | 通知状态变化 | 订单已发货事件 |

### 4.2 集成风格对比

| 风格 | 耦合度 | 实时性 | 数据一致性 | 适用场景 |
|------|--------|--------|-----------|---------|
| **File Transfer** | 低 | 低（批量） | 最终一致 | 数据仓库、ETL |
| **Shared Database** | 高 | 高 | 强一致 | 单体应用 |
| **RPC** | 中 | 高 | 调用方决定 | 同步调用 |
| **Messaging** | 极低 | 中 | 最终一致 | 异步解耦 |

### 4.3 企业集成模式（EIP）

EIP 由 Gregor Hohpe 和 Bobby Woolf 系统化整理，以下是关键模式：

**消息路由器（Message Router）**：根据条件将消息路由到不同通道。

```python
# 简单内容路由器
def route_message(message):
    if message.type == "order":
        publish("orders_queue", message)
    elif message.type == "payment":
        publish("payments_queue", message)
    elif message.type == "notification":
        publish("notifications_queue", message)
    else:
        publish("dead_letter", message)
```

**分割器（Splitter）**：将一个复合消息拆分为多条独立消息。

```python
def split_order(order):
    for item in order.items:
        # 每个商品独立处理
        process_item(item)
```

**聚合器（Aggregator）**：将多条相关消息合并为一条消息。

```python
# 聚合所有支付确认后发货
class ShipmentAggregator:
    def __init__(self):
        self.pending = {}  # order_id -> [payment_confirm, inventory_confirm]
    
    def handle_event(self, event):
        order_id = event.order_id
        if order_id not in self.pending:
            self.pending[order_id] = set()
        self.pending[order_id].add(event.type)
        
        # 聚合条件：同时收到支付确认和库存确认
        if {"payment_confirmed", "inventory_deducted"}.issubset(self.pending[order_id]):
            ship_order(order_id)
            del self.pending[order_id]
```

**死信通道（Dead Letter Channel）**：处理无法投递或无法处理的消息。

```python
try:
    process_message(message)
except Exception as e:
    # 重试三次后进入死信队列
    if message.retry_count >= 3:
        publish("dead_letter_queue", message)
        alert_ops_team(message, e)
    else:
        message.retry_count += 1
        publish_with_delay("input_queue", message, delay=2 ** message.retry_count)
```

---

## 五、分布式系统设计

### 5.1 CAP 定理的工程实践

CAP 定理指出分布式系统在**网络分区**下必须在**一致性（C）** 和**可用性（A）** 之间取舍：

| 选择 | 说明 | 典型系统 | 适用场景 |
|------|------|---------|---------|
| **CP** | 分区时牺牲可用性，保证一致性 | Zookeeper、etcd、MongoDB（默认） | 配置管理、分布式锁、金融交易 |
| **AP** | 分区时牺牲强一致性，保证可用性 | Cassandra、DynamoDB、Eureka | 社交动态、推荐、缓存 |

> **工程实战**：99.9% 的情况下你会遇到网络分区，所以 CAP 不是"三选二"，而是"CP vs AP 二选一"。但现实中大多数系统选择**最终一致性 + 补偿机制**。

### 5.2 一致性模型层次

| 模型 | 保证 | 性能影响 | 使用场景 |
|------|------|---------|---------|
| **强一致性** | 写入后立即读取到最新值 | 最高延迟 | 金融、分布式锁 |
| **因果一致性** | 有因果关系的操作按顺序可见 | 中等 | 评论系统 |
| **读自己写（Read-Your-Writes）** | 客户端总是读到自己的写入 | 低 | 用户资料更新 |
| **最终一致性** | 若无新写入，最终所有副本一致 | 最低延迟 | DNS、CDN、社交动态 |
| **单调读** | 不会读到更早版本的数据 | 低 | 防止页面"回滚" |

### 5.3 共识算法

| 算法 | 核心思想 | 容错 | 领导者 | 性能 | 使用 |
|------|---------|------|--------|------|------|
| **Paxos** | 两阶段提交 + 多数派 | (n-1)/2 | 无固定领导者 | 低（多轮通信） | 理论基石，实现复杂 |
| **Raft** | 领导者选举 + 日志复制 | (n-1)/2 | 强领导者 | 中高 | **工程实践首选** |
| **Zab** | 原子广播 + 崩溃恢复 | (n-1)/2 | 强领导者 | 高 | Zookeeper 内部使用 |
| **Gossip** | 点对点传播 | 高 | 无 | 最终一致 | Cassandra、Redis Cluster |

**Raft 简化的关键步骤**：

```text
1. 选举阶段: 所有节点启动时是 Follower
   - Follower 超时 (150-300ms 随机) → 变为 Candidate
   - Candidate 发起投票 → 获取多数票 → 成为 Leader
   
2. 日志复制阶段:
   - Client → Leader: 写入请求
   - Leader → Followers: AppendEntries RPC
   - Followers 确认后 → Leader 提交 → 响应 Client

3. 安全保证:
   - 只有一个 Leader (Term 机制)
   - Leader 不会覆盖已提交的日志
   - 只有多数派确认后才算提交
```

### 5.4 分布式事务模式

| 模式 | 一致性 | 性能 | 复杂度 | 适用 |
|------|--------|------|--------|------|
| **2PC（两阶段提交）** | 强一致 | 低（阻塞） | 中 | 短事务、低并发 |
| **SAGA** | 最终一致 | 高（异步） | 高 | 长事务、跨服务 |
| **TCC（Try-Confirm/Cancel）** | 最终一致 | 高 | 最高 | 资源预留型 |
| **Outbox 模式** | 最终一致 | 高 | 中 | 微服务事务 |

**Outbox 模式实现**：

```java
// 在同一个本地事务中写入业务表和 outbox 表
@Transactional
public void createOrder(CreateOrderRequest req) {
    // 1. 写入业务表
    Order order = orderRepository.save(new Order(req));
    
    // 2. 写入 outbox 表（同一个事务）
    OutboxMessage message = new OutboxMessage(
        "OrderCreated",
        serializeEvent(order),
        "pending"
    );
    outboxRepository.save(message);
    // 提交事务 → 两件事同时成功或同时失败
}

// 后台轮询 outbox 表，发送到消息队列
@Scheduled(fixedDelay = 1000)
public void publishOutbox() {
    List<OutboxMessage> pending = outboxRepository.findByStatus("pending");
    for (OutboxMessage msg : pending) {
        try {
            kafkaTemplate.send("orders", msg.getPayload());
            msg.setStatus("published");
        } catch (Exception e) {
            msg.setRetryCount(msg.getRetryCount() + 1);
            // 超过重试次数后标记为 dead_letter
        }
        outboxRepository.save(msg);
    }
}
```

### 5.5 服务发现

| 方式 | 实现 | 优点 | 缺点 |
|------|------|------|------|
| **客户端发现** | 客户端从注册中心获取服务地址 | 少一跳、延迟低 | 每种语言需集成 SDK |
| **服务端发现** | 通过负载均衡器 | 语言无关 | 多一跳、负载均衡器是瓶颈 |

**注册中心对比**：

| 系统 | CAP 选择 | 一致性 | 健康检查 | 使用场景 |
|------|---------|--------|---------|---------|
| **Consul** | CP | Raft | 支持（HTTP/gRPC/TCP） | 通用服务发现 |
| **etcd** | CP | Raft | 需自行实现 | Kubernetes 使用 |
| **Eureka** | AP | 最终一致 | 心跳机制 | Spring Cloud |
| **Zookeeper** | CP | Zab | 临时节点 | 分布式协调 |

### 5.6 容错模式

| 模式 | 目的 | 实现方式 |
|------|------|---------|
| **Circuit Breaker** | 防止级联故障 | 关闭 → 打开 → 半开 | 
| **Bulkhead** | 隔离资源，一个失败不影响其他 | 线程池隔离 / 信号量隔离 |
| **Retry** | 临时故障自动重试 | 指数退避 + 抖动 |
| **Timeout** | 防止调用无限等待 | 连接超时 + 读取超时 |

**断路器实现示例**：

```java
public class CircuitBreaker {
    private enum State { CLOSED, OPEN, HALF_OPEN }
    
    private State state = State.CLOSED;
    private int failureCount = 0;
    private final int threshold = 5;        // 失败 5 次后打开
    private final long timeoutMs = 30000;   // 30 秒后进入半开
    private long lastFailureTime;
    
    public <T> T call(Supplier<T> operation) {
        if (state == State.OPEN) {
            if (System.currentTimeMillis() - lastFailureTime > timeoutMs) {
                state = State.HALF_OPEN;
            } else {
                throw new CircuitBreakerOpenException();
            }
        }
        
        try {
            T result = operation.get();
            if (state == State.HALF_OPEN) {
                state = State.CLOSED;  // 请求成功，关闭断路器
                failureCount = 0;
            }
            return result;
        } catch (Exception e) {
            failureCount++;
            lastFailureTime = System.currentTimeMillis();
            if (failureCount >= threshold) {
                state = State.OPEN;
            }
            throw e;
        }
    }
}
```

### 5.7 分布式缓存模式

| 模式 | 说明 | 适用 | 风险 |
|------|------|------|------|
| **Cache-Aside** | 读时先查缓存，未命中查DB后回写 | 通用 | 缓存雪崩 |
| **Read-Through** | 缓存层自动从DB加载 | 一致性要求不高 | 缓存穿透 |
| **Write-Through** | 写操作先写缓存再同步到DB | 一致性要求高 | 写入延迟增加 |
| **Write-Behind** | 写操作只写缓存，异步批量写DB | 写入密集 | 宕机丢数据 |

**缓存穿透（空值缓存 + Bloom Filter）**：

```java
public User getUserById(String userId) {
    // 1. Bloom Filter 快速判断 userId 是否存在
    if (!bloomFilter.mightContain(userId)) {
        return null; // 肯定不存在，直接返回
    }
    
    // 2. 查缓存
    User user = redis.get("user:" + userId);
    if (user != null) {
        return user; // 缓存命中
    }
    
    // 3. 查数据库
    user = db.query("SELECT * FROM users WHERE id = ?", userId);
    
    if (user != null) {
        redis.set("user:" + userId, user, TTL_1H);
    } else {
        // 空值缓存，防止恶意 key 穿透
        redis.set("user:" + userId, NULL_PLACEHOLDER, TTL_5MIN);
    }
    
    return user;
}
```

### 5.8 限流算法

| 算法 | 描述 | 特点 | 适用 |
|------|------|------|------|
| **Token Bucket** | 固定速率生成令牌，有令牌才能处理 | 允许突发流量 | API 限流 |
| **Leaky Bucket** | 固定速率处理请求，超出丢弃 | 平滑输出 | 数据处理管道 |
| **Sliding Window Log** | 记录时间戳窗口内的请求数 | 精确但内存占用高 | 精确控制 |
| **Sliding Window Counter** | 分桶计数，滑动汇总 | 内存效率高 | 生产环境常用 |

**Token Bucket 实现**：

```python
import time

class TokenBucket:
    def __init__(self, rate: float, capacity: int):
        self.rate = rate          # 每秒生成令牌数
        self.capacity = capacity  # 桶容量(最大突发)
        self.tokens = capacity    # 当前令牌数
        self.last_refill = time.monotonic()
    
    def allow(self) -> bool:
        now = time.monotonic()
        # 补充令牌
        elapsed = now - self.last_refill
        self.tokens = min(self.capacity, self.tokens + elapsed * self.rate)
        self.last_refill = now
        
        if self.tokens >= 1:
            self.tokens -= 1
            return True
        return False

# 使用：每用户 10 QPS，突发 20
bucket = TokenBucket(rate=10, capacity=20)
if bucket.allow():
    process_request()
else:
    return 429 Too Many Requests
```

**Sliding Window Counter 实现**：

```python
from collections import deque
import time

class SlidingWindowCounter:
    def __init__(self, window_size_sec: int, max_requests: int):
        self.window_size = window_size_sec
        self.max_requests = max_requests
        self.requests = deque()  # 存储每个请求的时间戳
    
    def allow(self) -> bool:
        now = time.monotonic()
        # 移除窗口外的请求
        while self.requests and self.requests[0] <= now - self.window_size:
            self.requests.popleft()
        
        if len(self.requests) < self.max_requests:
            self.requests.append(now)
            return True
        return False
```

---

## 六、系统设计方法论

### 6.1 理解需求

**功能需求 vs 非功能需求**：

| 类型 | 定义 | 例子 |
|------|------|------|
| **功能需求** | 系统必须做什么 | 用户可以创建短链接；重定向到原链接 |
| **非功能需求** | 系统必须做到什么程度 | P99 延迟 < 50ms；可用性 > 99.9% |

**估算方法论**：

1. **QPS 估算**：`日活用户 × 平均请求数 / 86400`
2. **流量估算**：`QPS × 平均请求大小`
3. **存储估算**：`日增数据量 × 保留天数 × 副本数`

### 6.2 信封背面估算（Back-of-the-Envelope）

```text
常用参考值:
- 每天秒数: 86400 ≈ 10^5
- 每月秒数: 2.5 × 10^6
- 每年毫秒数: 3.156 × 10^10

- MySQL QPS: ~1000 (单机)
- Redis QPS: ~100000 (单机)
- 网络延迟(同机房): ~0.5ms
- 网络延迟(同城): ~1-2ms
- 磁盘顺序读: ~1GB/s
- 磁盘随机读: ~100 IOPS (HDD) / ~10000 IOPS (SSD)

估算公式:
- 1 天 = 86400s ≈ 10^5s
- 月活 1 亿 → 日活≈5000万 → 每秒请求(日活/86400) ≈ 580
- 每条记录 1KB → 1亿条 ≈ 100GB
```

### 6.3 案例：设计 URL 短链接系统

**需求**：
- 日生成 1 亿条短链接
- 重定向 P99 延迟 < 50ms
- 短链接有效期永不过期

**估算**：

```text
写入 QPS: 1亿 / 86400 ≈ 1157 QPS
峰值 QPS: 1157 × 3 = 3471 QPS
读取 QPS: 写入 × 100 = 115700 QPS (每次短链接被访问约 100 次)
存储: 1亿 × 500bytes = 50GB/天 → 1年 = 18TB
```

**数据模型**：

```sql
CREATE TABLE url_mapping (
    short_id   VARCHAR(7)  PRIMARY KEY,  -- Base62 编码，7位 = 62^7 ≈ 3.5万亿
    long_url   TEXT        NOT NULL,
    created_at TIMESTAMP   NOT NULL DEFAULT NOW(),
    expire_at  TIMESTAMP,                 -- NULL 表示永不过期
    INDEX idx_created (created_at)
);
```

**架构设计**：

```text
┌────────┐    ┌──────────────┐    ┌────────────────┐
│ Client │───→│   API Gateway  │───→│  Write Service  │───→ DB (主库)
└────────┘    └──────────────┘    └────────────────┘
                    │                      │
                    │                      └──→ Kafka → 分析服务
                    │                      │
                    │              ┌────────────────┐
                    │──→ Cache ──→│  Read Service   │
                    │   (Redis)   │  (无状态,可水平扩展) │
                    │              └────────────────┘
                    │                      │
                    │              ┌────────────────┐
                    └─────────────→│  Redirect       │──→ CDN / 302
                                   │  (极致优化)     │
                                   └────────────────┘
```

**短链接生成算法**：

```python
import base62

def generate_short_id(sequence_id: int) -> str:
    """将分布式序列ID编码为Base62短字符串"""
    return base62.encode(sequence_id)  # 1 → '1', 1000 → 'g8', 62^7 ≈ 3.5万亿

# 分布式序列ID生成方案: Redis INCR / Snowflake / DB自增
# 使用 Redis:
# redis.incr("url_counter") → 返回递增ID
```

**缓存策略**：

```python
def redirect(short_id: str) -> str:
    # 1. 查 Redis 缓存
    long_url = redis.get(f"url:{short_id}")
    if long_url:
        return long_url
    
    # 2. 缓存未命中，查数据库
    row = db.query("SELECT long_url FROM url_mapping WHERE short_id = ?", short_id)
    if not row:
        return 404
    
    # 3. 回写缓存（带TTL，热点访问自动保持缓存热点）
    redis.setex(f"url:{short_id}", TTL_24H, row.long_url)
    return row.long_url
```

**扩展方案**：
- 读写分离：写主库、读从库 + Redis 缓存
- 分库分表：按 short_id 范围分片
- CDN + 预加载：将热门短链接预推到 CDN

### 6.4 案例：设计聊天系统

**核心需求**：
- 支持 1v1 聊天和群聊
- 消息延迟 < 100ms
- 支持多媒体消息（图片、文件）
- 消息持久化，历史可回溯

**架构**：

```text
┌────────┐    WebSocket      ┌──────────────┐
│ Client │═══════════════════│  Chat Server  │──────→ Redis Pub/Sub
│ (App)  │                   │  (无状态)      │        (消息实时分发)
└────────┘                   └──────────────┘
     │                            │
     │ HTTP/HTTPS                 │
     │ 上传图片/文件               │
     ▼                            ▼
┌──────────┐              ┌──────────────┐
│ 文件服务   │              │ Kafka / Pulsar│
│ (OSS/CDN) │              │ (消息持久化)   │
└──────────┘              └──────┬───────┘
                                 │
                                 ▼
                          ┌──────────────┐
                          │ Message DB   │
                          │ (Cassandra/   │
                          │  TiDB)        │
                          └──────────────┘
```

**数据模型**：

```sql
-- 消息表（按 conversation_id 分片）
CREATE TABLE messages (
    conversation_id  VARCHAR(32)   NOT NULL,  -- 会话ID
    message_id       BIGINT        NOT NULL,  -- 递增的消息ID
    sender_id        VARCHAR(32)   NOT NULL,
    content          TEXT,
    media_url        VARCHAR(512),
    message_type     TINYINT       NOT NULL,  -- text/image/file/system
    created_at       TIMESTAMP     NOT NULL,
    PRIMARY KEY (conversation_id, message_id)
) PARTITION BY HASH(conversation_id);

-- 对话表
CREATE TABLE conversations (
    id           VARCHAR(32) PRIMARY KEY,
    type         TINYINT   NOT NULL,  -- 1:1 / group
    name         VARCHAR(128),
    created_at   TIMESTAMP NOT NULL,
    last_message TEXT
);

-- 对话成员表
CREATE TABLE conversation_members (
    conversation_id VARCHAR(32) NOT NULL,
    user_id         VARCHAR(32) NOT NULL,
    last_read_id    BIGINT    NOT NULL DEFAULT 0,
    joined_at       TIMESTAMP NOT NULL,
    PRIMARY KEY (conversation_id, user_id)
);
```

**消息发送流程**：

```text
1. Client A → Chat Server: 发送消息(conversation_id, content)
2. Chat Server 验证权限 (确认 A 在 conversation 中)
3. Chat Server → DB: INSERT messages (返回 message_id)
4. Chat Server → Kafka: 发布消息事件
5. Kafka Consumer:
   a. 更新 conversation 的 last_message
   b. 处理离线推送 (APNs / FCM)
   c. 更新未读计数
6. Chat Server → Redis Pub/Sub: 推送消息给在线成员
7. Redis Pub/Sub 分发给各 WebSocket 连接 → Client B, C, D...
```

**消息同步（读扩散 vs 写扩散）**：

| 方案 | 说明 | 优点 | 缺点 | 适用 |
|------|------|------|------|------|
| **读扩散** | 每条消息存一份，用户读时拉取 | 存储少 | 拉取延迟高 | 小群聊 |
| **写扩散** | 每条消息为每个成员存一份 | 读延迟低 | 存储大、写入放大 | 1v1、千人群 |

> **优化方案**：两者结合——1v1 和百人以下小群用写扩散；大群用读扩散 + 缓存。

---

## 七、API 网关与流量管理

### 7.1 API 网关模式

| 模式 | 描述 | 适用场景 |
|------|------|---------|
| **BFF（Backend For Frontend）** | 为每种客户端提供专属网关 | 多端（Web/iOS/Android） |
| **请求聚合** | 聚合多个后端服务的响应 | 移动端减少网络请求 |
| **路由** | 根据路径将请求转发到不同服务 | 微服务统一入口 |
| **认证授权** | 集中处理 JWT/OAuth2 认证 | 统一安全策略 |

**BFF 架构示例**：

```text
Client ──→ Mobile BFF ──→ 用户服务
                          ├── 订单服务
                          └── 推荐服务

Client ──→ Web BFF  ───→ 用户服务
                          ├── 订单服务
                          └── CMS 服务
```

### 7.2 网关产品对比

| 特性 | Kong | Nginx | Envoy | AWS API Gateway |
|------|------|-------|-------|----------------|
| **部署方式** | 独立进程 | 独立进程 | Sidecar/独立 | 托管服务 |
| **语言** | Lua+Nginx | C | C++ | 托管 |
| **性能** | 高 | 高 | 高 | 中（受网络延迟影响）|
| **插件生态** | 丰富（官方+社区） | 有限（Lua模块） | 丰富（Lua/WASM）| 受限（AWS 生态）|
| **配置方式** | Admin API + DB | 静态文件 | xDS（动态） | 控制台 + IaC |
| **动态路由** | 支持 | 需要 reload | 原生支持 | 原生支持 |
| **服务发现** | 插件支持 | 需自行实现 | 原生支持 | 仅 AWS 服务 |
| **适用** | 微服务网关 | 反向代理/负载均衡 | Service Mesh 数据面 | AWS 生态 |

### 7.3 网关级限流

API 网关是实现集中式限流的理想位置：

```yaml
# Kong 限流配置示例
plugins:
  - name: rate-limiting
    config:
      second: 10    # 每秒 10 次
      minute: 600   # 每分钟 600 次
      policy: local # local / redis / cluster
      fault_tolerant: true  # 限流服务宕机时放行
```

**多维度限流策略**：

```python
class GatewayRateLimiter:
    """API 网关限流器"""
    
    def is_allowed(self, request):
        checks = [
            # 全局限流: 每秒 10000 请求
            self.check_global(limit=10000, window=1),
            # 每用户限流: 每秒 100 请求
            self.check_user(request.user_id, limit=100, window=1),
            # 每 IP 限流: 每秒 500 请求
            self.check_ip(request.client_ip, limit=500, window=1),
            # 每 API 限流: 特定 API 每秒 50 请求
            self.check_api(request.path, limit=50, window=1),
        ]
        return all(checks)
```

---

> **核心认知总结**：
>
> 1. **架构没有银弹**——每种风格都有其适用场景和代价，关键是用架构决策矩阵做理性选择
> 2. **模式是交流语言**——设计模式降低了沟通成本，但模式不是目标，解决实际问题才是
> 3. **分布式是最后的选择**——单体够用时别引入分布式复杂性
> 4. **先测量，再优化**——不测量就没有方向；不在测量指导下优化就是瞎优化
> 5. **简单优于灵活**——"你不会需要它"（YAGNI）在架构层面同样成立

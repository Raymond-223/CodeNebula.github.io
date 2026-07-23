# 数据库与存储

> 数据库是软件系统的基石。本文从理论到实践，覆盖关系型数据库、NoSQL、ORM、数据库设计、缓存策略等核心知识领域，帮助你建立完整的数据库知识体系。

---

## 一、数据库基础理论

### 1.1 ACID 特性

ACID 是关系型数据库事务的四大特性，保证数据可靠性。

| 特性 | 英文 | 含义 | 反面例子 |
|------|------|------|---------|
| **原子性** | Atomicity | 事务要么全部成功，要么全部回滚 | 银行转账：扣款成功但加款失败 |
| **一致性** | Consistency | 事务前后数据满足所有约束 | 账户总余额在转账前后保持不变 |
| **隔离性** | Isolation | 并发事务互不干扰 | 两个用户同时修改同一条记录 |
| **持久性** | Durability | 提交后的事务永久保存 | 系统崩溃后数据不丢失 |

**原子性示例**：
```sql
-- 如果第二个 UPDATE 失败，第一个也会回滚
BEGIN;
UPDATE accounts SET balance = balance - 100 WHERE id = 1;
UPDATE accounts SET balance = balance + 100 WHERE id = 2;
COMMIT;
```

**持久性的实现**：MySQL 通过 Redo Log（重做日志）保证持久性。事务提交时，先写 Redo Log（磁盘顺序写），再写数据页（随机写）。崩溃恢复时，通过 Redo Log 重放未完成的操作。

### 1.2 CAP 定理

分布式系统中的三个属性无法同时完美满足，最多只能兼顾两个：

```
          Consistency (一致性)
               │
               │
               ├── CP 系统：ZooKeeper、etcd
               │
               ├── AP 系统：Cassandra、DynamoDB
               │
               └── CA 系统：传统单机数据库（无分区容忍）
```

| 系统 | 偏向 | 说明 |
|------|------|------|
| **MySQL 主从** | CA | 网络分区时停止写入以保证一致性 |
| **Cassandra** | AP | 分区时仍可写入，但数据可能不一致 |
| **ZooKeeper** | CP | 分区时多数派节点继续服务，少数派停止服务 |
| **Redis Cluster** | CP（多数场景） | 分区时少数分区不可写入 |

> **重要理解**：网络分区（P）是分布式系统中的必然事件，因此实际上我们只能在 CP 和 AP 之间选择。CA 系统在网络分区发生时只能牺牲 C 或 A。

### 1.3 BASE 理论

BASE 是 NoSQL 系统常用的弱一致性模型：

- **Basically Available**（基本可用）：系统在故障时仍可响应请求，可能返回降级结果
- **Soft State**（软状态）：系统状态可以随时间变化，不要求时刻一致
- **Eventually Consistent**（最终一致）：经过足够长的时间，所有副本最终会达到一致

**举例**：DNS 系统就是 BASE 的典型——你更新了一个域名记录，全球 DNS 服务器可能需要几个小时才能全部更新，但最终会一致。

### 1.4 隔离级别与并发问题

SQL 标准定义了四个隔离级别，每个级别解决了特定的并发问题：

```
                       ┌─────────────────────┐
                       │                     │
    Serializable ───────┤  Phantom Read      │ ← 最高隔离，性能最差
                       │  (幻读)             │
    Repeatable Read ────┤  Non-repeatable    │
                       │  Read (不可重复读)   │
    Read Committed ─────┤  Dirty Read        │
                       │  (脏读)             │
    Read Uncommitted ───┤                    │ ← 性能最好，问题最多
                       └─────────────────────┘
```

**三种并发问题**：

| 问题 | 定义 | 示例 |
|------|------|------|
| **脏读** | 读到另一个事务未提交的数据 | A 转给 B 100 元，事务未提交，B 看到余额已增加 |
| **不可重复读** | 同一事务内两次读取同一行，结果不同（行更新） | 第一次查 price=100，第二次查 price=150 |
| **幻读** | 同一事务内两次查询，结果集不同（行插入/删除） | 第一次查 count=10，第二次查 count=11 |

**MySQL 语法**：
```sql
-- 查看和设置隔离级别
SELECT @@transaction_isolation;
SET SESSION TRANSACTION ISOLATION LEVEL REPEATABLE READ;
```

> **MySQL 的 REPEATABLE READ**：通过 MVCC（多版本并发控制）解决了幻读问题。但严格来说，只在快照读（普通 SELECT）上解决了幻读，当前读（SELECT ... FOR UPDATE）仍可能产生幻读。

---

## 二、关系型数据库

### 2.1 MySQL

#### InnoDB vs MyISAM

| 特性 | InnoDB | MyISAM |
|------|--------|--------|
| **事务** | ✅ ACID 事务 | ❌ 不支持 |
| **外键** | ✅ 支持 | ❌ 不支持 |
| **锁粒度** | 行级锁 | 表级锁 |
| **全文索引** | ✅（5.6+ 支持） | ✅ |
| **聚集索引** | ✅（必须） | ❌（堆表） |
| **崩溃恢复** | ✅（Redo Log） | ❌（需修复） |
| **MVCC** | ✅ | ❌ |

**何时选择 MyISAM**？几乎不需要了。InnoDB 在 MySQL 5.6+ 已是默认引擎，功能全面超越 MyISAM。

#### B+Tree 索引

MySQL InnoDB 使用 B+Tree 作为默认索引结构：

```
                    [根节点: 50, 120]
                   /        |         \
           [30, 40]    [70, 90]    [150, 180]
          /    |    \    /   |   \     |    \
        (10) (35) (45) (55)(75)(95)  (130)(160)
```

**特点**：
- 所有数据存在叶子节点，非叶子节点只存键值
- 叶子节点形成有序链表，支持范围查询
- 树高通常 3-4 层，IO 次数极少

**聚集索引 vs 二级索引**：
- 聚集索引：主键即索引，叶子节点存整行数据
- 二级索引（普通索引）：叶子节点存主键值，需要回表查询

```sql
-- 创建索引
CREATE INDEX idx_email ON users(email);
CREATE UNIQUE INDEX idx_phone ON users(phone);
-- 复合索引（最左前缀原则）
CREATE INDEX idx_name_age ON users(name, age);
-- 上述索引能匹配: WHERE name=?, WHERE name=? AND age=?
-- 不能匹配: WHERE age=?（没有 name 前缀）
```

#### 复制架构

**主从复制（Master-Slave）**：
```
Application
    │   写请求 → Master ──→ Binary Log ──→ Slave 1 (只读)
    │   读请求 → Slave 2 (只读)
    └──────────→ Slave 3 (只读/备份)
```

**半同步复制**：Master 至少等待一个 Slave 确认收到 Binlog 才返回客户端，兼顾性能和数据安全。

**组复制（Group Replication）**：MySQL 5.7+ 引入的多主复制方案，基于 Paxos 协议，提供强一致性。

#### 分区表

```sql
-- 范围分区
CREATE TABLE orders (
    id INT, order_date DATE, amount DECIMAL(10,2)
) PARTITION BY RANGE (YEAR(order_date)) (
    PARTITION p2022 VALUES LESS THAN (2023),
    PARTITION p2023 VALUES LESS THAN (2024),
    PARTITION p2024 VALUES LESS THAN (2025)
);

-- 哈希分区
CREATE TABLE users (
    id INT, name VARCHAR(100)
) PARTITION BY HASH (id) PARTITIONS 4;
```

**分区表的优势**：分区裁剪（Partition Pruning）可以跳过不相关的分区，提高查询效率；方便归档和删除历史数据。

### 2.2 PostgreSQL

PostgreSQL 是"最先进的开源关系型数据库"，以扩展性和标准兼容性著称。

#### MVCC 实现

PostgreSQL 的 MVCC 通过 **元组版本** 实现——每个 UPDATE 产生一个新版本的元组，旧版本保留在表中（直到 VACUUM 清理）。

```sql
-- 查看事务快照信息
SELECT xmin, xmax, * FROM users;
```

与 MySQL 的差异：
- MySQL InnoDB 的 MVCC 将旧版本存在 Undo Log 中
- PostgreSQL 的旧版本直接留在数据表中，需要定期 VACUUM

#### JSONB 与全文搜索

```sql
-- JSONB 数据类型
CREATE TABLE products (
    id SERIAL PRIMARY KEY,
    metadata JSONB
);

-- GIN 索引加速 JSONB 查询
CREATE INDEX idx_metadata ON products USING GIN (metadata);

-- JSONB 查询
SELECT * FROM products WHERE metadata @> '{"brand": "Apple"}';
SELECT * FROM products WHERE metadata -> 'specs' ->> 'ram' = '16GB';

-- 全文搜索
CREATE INDEX idx_search ON articles USING GIN (to_tsvector('simple', content));

SELECT * FROM articles
WHERE to_tsvector('simple', content) @@ to_tsquery('simple', 'database & optimization');
```

#### 扩展生态

```sql
-- PostGIS (地理信息)
CREATE EXTENSION postgis;
SELECT ST_Distance(
    ST_SetSRID(ST_MakePoint(116.4, 39.9), 4326),  -- 北京
    ST_SetSRID(ST_MakePoint(121.47, 31.23), 4326)  -- 上海
);

-- pg_stat_statements (SQL 性能统计)
CREATE EXTENSION pg_stat_statements;
SELECT query, calls, total_time, mean_time
FROM pg_stat_statements ORDER BY total_time DESC LIMIT 10;
```

#### CTE 与窗口函数

```sql
-- CTE (Common Table Expression)
WITH regional_sales AS (
    SELECT region, SUM(amount) AS total_sales
    FROM orders GROUP BY region
)
SELECT region, total_sales
FROM regional_sales
WHERE total_sales > (SELECT AVG(total_sales) FROM regional_sales);

-- 递归 CTE (适用于树形数据)
WITH RECURSIVE org_tree AS (
    SELECT id, name, parent_id, 1 AS level
    FROM employees WHERE parent_id IS NULL
    UNION ALL
    SELECT e.id, e.name, e.parent_id, t.level + 1
    FROM employees e JOIN org_tree t ON e.parent_id = t.id
)
SELECT * FROM org_tree;

-- 窗口函数
SELECT
    name,
    department,
    salary,
    RANK() OVER (PARTITION BY department ORDER BY salary DESC) AS dept_rank,
    SUM(salary) OVER (PARTITION BY department) AS dept_total,
    AVG(salary) OVER (PARTITION BY department) AS dept_avg,
    salary - AVG(salary) OVER (PARTITION BY department) AS diff_from_avg
FROM employees;
```

### 2.3 SQLite

**当使用 SQLite**：
- 移动端应用（Android/iOS 本地数据存储）
- 桌面应用的本地数据库
- 嵌入式设备
- 开发和测试环境
- 原型验证阶段

**当不使用 SQLite**：
- 高并发写入场景（SQLite 写操作串行化）
- 需要用户管理的访问控制
- 需要网络访问的 C/S 架构
- 超大数据集（超过 140TB 有限制，但实际几十 GB 以上建议换 PG）

**WAL 模式（Write-Ahead Logging）**：
```sql
-- 提升并发读性能
PRAGMA journal_mode=WAL;
-- 写入性能优化
PRAGMA synchronous=NORMAL;
PRAGMA cache_size=-64000;  -- 64MB 缓存
```

### 2.4 三大关系型数据库对比

| 维度 | MySQL | PostgreSQL | SQLite |
|------|-------|-----------|--------|
| **架构** | 客户端-服务器 | 客户端-服务器 | 嵌入式 |
| **ACID** | ✅（InnoDB） | ✅ | ✅ |
| **SQL 标准** | 部分兼容 | ⭐ 高度兼容 | 核心标准 |
| **并发控制** | MVCC（行级锁） | MVCC（行级锁） | 读写锁/WAL |
| **索引类型** | B+Tree, Hash, Full-Text | B+Tree, Hash, GiST, GIN, BRIN, SP-GiST | B-Tree |
| **JSON** | JSON 数据类型（5.7+） | ⭐ JSONB（性能更好） | ❌ 有限支持 |
| **窗口函数** | 8.0+ 支持 | ✅ 原生支持 | ❌ 不支持 |
| **扩展性** | 插件有限 | ⭐ 极其丰富 | 有限 |
| **复制** | 主从/组复制/Galera | 流复制/逻辑复制 | ❌ 无 |
| **使用场景** | Web 应用主力 | 复杂查询/地理/金融 | 移动端/嵌入式 |
| **许可证** | GPL / 商业版 | PostgreSQL 授权 | Public Domain |

---

## 三、SQL 进阶

### 3.1 连接（JOIN）

```sql
-- 建表
CREATE TABLE users (id INT PRIMARY KEY, name TEXT);
CREATE TABLE orders (id INT PRIMARY KEY, user_id INT, amount DECIMAL);

-- INNER JOIN: 两表都匹配的记录
SELECT u.name, o.amount
FROM users u
INNER JOIN orders o ON u.id = o.user_id;

-- LEFT JOIN: 左表全部 + 右表匹配的记录（无匹配则为 NULL）
SELECT u.name, COALESCE(o.amount, 0) AS amount
FROM users u
LEFT JOIN orders o ON u.id = o.user_id;

-- RIGHT JOIN: 右表全部 + 左表匹配的记录
SELECT u.name, o.amount
FROM users u
RIGHT JOIN orders o ON u.id = o.user_id;

-- FULL JOIN: 两表的全部记录（MySQL 不支持，用 UNION 模拟）
SELECT * FROM users u
LEFT JOIN orders o ON u.id = o.user_id
UNION
SELECT * FROM users u
RIGHT JOIN orders o ON u.id = o.user_id;

-- CROSS JOIN: 笛卡尔积（慎用！）
SELECT u.name, p.product_name
FROM users u
CROSS JOIN products p;

-- SELF JOIN: 自连接，查找上下级关系
SELECT e1.name AS employee, e2.name AS manager
FROM employees e1
LEFT JOIN employees e2 ON e1.manager_id = e2.id;

-- LATERAL JOIN (PostgreSQL): 右侧子查询可以引用左侧的列
SELECT u.name, recent.order_id, recent.amount
FROM users u
LEFT JOIN LATERAL (
    SELECT id AS order_id, amount
    FROM orders
    WHERE user_id = u.id
    ORDER BY created_at DESC
    LIMIT 1
) recent ON true;
```

### 3.2 子查询

```sql
-- 非关联子查询（独立执行一次）
SELECT * FROM users
WHERE id IN (SELECT user_id FROM orders WHERE amount > 1000);

-- 关联子查询（外层每行执行一次）
-- 查找工资高于部门平均的员工
SELECT e1.name, e1.salary, e1.department
FROM employees e1
WHERE e1.salary > (
    SELECT AVG(e2.salary)
    FROM employees e2
    WHERE e2.department = e1.department
);

-- EXISTS（半连接语义，找到即停止）
SELECT d.name
FROM departments d
WHERE EXISTS (
    SELECT 1 FROM employees e
    WHERE e.department_id = d.id AND e.salary > 100000
);

-- ANY / ALL
SELECT * FROM products
WHERE price > ALL (SELECT price FROM products WHERE category = 'Budget');
```

### 3.3 窗口函数

窗口函数在不聚合行的情况下进行计算，是 SQL 进阶的核心技能。

```sql
-- ROW_NUMBER: 唯一行号
SELECT
    name, department, salary,
    ROW_NUMBER() OVER (PARTITION BY department ORDER BY salary DESC) AS rank_seq
FROM employees;

-- RANK / DENSE_RANK: 排名（区别在于并列处理）
SELECT
    score,
    RANK() OVER (ORDER BY score DESC) AS rank,           -- 1, 2, 2, 4
    DENSE_RANK() OVER (ORDER BY score DESC) AS dense_rank -- 1, 2, 2, 3
FROM exam_scores;

-- LAG / LEAD: 前后行值
SELECT
    date, amount,
    LAG(amount, 1) OVER (ORDER BY date) AS prev_day,
    LEAD(amount, 1) OVER (ORDER BY date) AS next_day,
    amount - LAG(amount, 1) OVER (ORDER BY date) AS change
FROM daily_sales;

-- NTILE: 分桶（例如分成 4 组等分）
SELECT name, sales,
    NTILE(4) OVER (ORDER BY sales DESC) AS quartile
FROM salespeople;

-- 累计和/移动平均
SELECT
    date, amount,
    SUM(amount) OVER (ORDER BY date) AS running_total,
    AVG(amount) OVER (ORDER BY date ROWS BETWEEN 6 PRECEDING AND CURRENT ROW) AS moving_avg_7d
FROM daily_sales;
```

### 3.4 CTE（公用表表达式）

```sql
-- WITH 子句，使复杂查询更清晰
WITH
high_value AS (
    SELECT user_id, SUM(amount) AS total
    FROM orders
    WHERE created_at >= '2024-01-01'
    GROUP BY user_id
    HAVING SUM(amount) > 10000
),
top_customers AS (
    SELECT u.name, h.total
    FROM users u
    JOIN high_value h ON u.id = h.user_id
    ORDER BY h.total DESC
    LIMIT 10
)
SELECT * FROM top_customers;

-- 递归 CTE: 遍历树形结构
WITH RECURSIVE menu_tree AS (
    -- 基础情况：根节点
    SELECT id, name, parent_id, 1 AS depth,
           CAST(name AS TEXT) AS path
    FROM menu_items
    WHERE parent_id IS NULL

    UNION ALL

    -- 递归步骤：子节点
    SELECT m.id, m.name, m.parent_id, t.depth + 1,
           CAST(t.path || ' → ' || m.name AS TEXT)
    FROM menu_items m
    JOIN menu_tree t ON m.parent_id = t.id
)
SELECT * FROM menu_tree ORDER BY path;
```

### 3.5 事务与保存点

```sql
-- 基本事务
BEGIN;
UPDATE inventory SET quantity = quantity - 1 WHERE product_id = 100;
INSERT INTO order_items (order_id, product_id, quantity) VALUES (500, 100, 1);
COMMIT;
-- 或 ROLLBACK;

-- 保存点：部分回滚
BEGIN;
INSERT INTO logs (message) VALUES ('Step 1 completed');
SAVEPOINT step1;
INSERT INTO logs (message) VALUES ('Step 2 completed');
-- 如果 Step 2 失败，回滚到 step1
ROLLBACK TO SAVEPOINT step1;
-- Step 1 的插入还在，Step 2 被撤销
COMMIT;
```

### 3.6 索引进阶

```sql
-- 复合索引（最左前缀）
CREATE INDEX idx_city_age ON users(city, age);
-- 匹配: WHERE city='Beijing' AND age=30
-- 匹配: WHERE city='Beijing'
-- 不匹配: WHERE age=30（跳过了 city）

-- 部分索引（PostgreSQL）
CREATE INDEX idx_active_users ON users(email) WHERE status = 'active';

-- 覆盖索引（索引包含查询所需所有列，避免回表）
CREATE INDEX idx_covering ON orders(user_id, amount, created_at);
-- 查询可以直接从索引获取数据：
SELECT user_id, amount, created_at FROM orders WHERE user_id = 100;

-- 全文索引
CREATE FULLTEXT INDEX idx_content ON articles(title, body);
SELECT * FROM articles WHERE MATCH(title, body) AGAINST('database optimization');

-- GiST/GIN 索引（PostgreSQL 高级索引）
CREATE INDEX idx_geo ON locations USING GIST (coord);  -- 地理空间
CREATE INDEX idx_tags ON posts USING GIN (tags);        -- 数组/JSON
```

### 3.7 查询优化

```sql
-- EXPLAIN ANALYZE: 实际执行计划（最常用的分析工具）
EXPLAIN ANALYZE
SELECT u.name, COUNT(o.id) AS order_count
FROM users u
LEFT JOIN orders o ON u.id = o.user_id
WHERE u.created_at > '2024-01-01'
GROUP BY u.id, u.name
HAVING COUNT(o.id) > 5;

-- 慢查询日志（MySQL）
SET GLOBAL slow_query_log = ON;
SET GLOBAL long_query_time = 1;  -- 超过 1 秒的记录
SET GLOBAL log_queries_not_using_indexes = ON;
```

**优化原则**：
1. **用小结果集驱动大结果集**：先过滤再 JOIN
2. **避免 SELECT \***：只取必要字段，利用覆盖索引
3. **避免函数包裹索引列**：`WHERE DATE(created_at) = '2024-01-01'` 导致索引失效，应改为 `WHERE created_at >= '2024-01-01' AND created_at < '2024-01-02'`
4. **合理使用 LIMIT**：减少数据传输量
5. **分析慢查询**：使用 EXPLAIN ANALYZE 定位瓶颈

### 3.8 存储过程 vs 函数 vs 触发器

| 特性 | 存储过程 | 函数 | 触发器 |
|------|---------|------|--------|
| **返回值** | 可有可无（OUT 参数） | 必有一个返回值 | 无 |
| **事务控制** | ✅ 支持 COMMIT/ROLLBACK | ❌ 不支持 | 继承父事务 |
| **SQL 调用** | CALL proc() | SELECT func() | 自动触发 |
| **参数** | IN/OUT/INOUT | IN | 无（通过 NEW/OLD 访问） |

```sql
-- 函数示例（PostgreSQL）
CREATE FUNCTION get_user_order_count(user_id INT)
RETURNS INT AS $$
BEGIN
    RETURN (SELECT COUNT(*) FROM orders WHERE user_id = $1);
END;
$$ LANGUAGE plpgsql;

-- 触发器示例
CREATE TRIGGER update_timestamp
BEFORE UPDATE ON users
FOR EACH ROW
EXECUTE FUNCTION set_updated_at();
```

---

## 四、NoSQL 数据库

### 4.1 MongoDB

#### 文档模型

MongoDB 以 BSON（二进制 JSON）格式存储文档，Schema 灵活：

```json
{
  "_id": ObjectId("..."),
  "name": "Alice",
  "email": "alice@example.com",
  "address": {
    "city": "Beijing",
    "district": "Haidian"
  },
  "tags": ["premium", "vip"],
  "orders": [
    { "product": "Laptop", "price": 9999, "qty": 1 }
  ]
}
```

#### 聚合管道

```javascript
// 聚合管道示例
db.orders.aggregate([
  { $match: { status: "completed" } },
  { $group: {
    _id: "$customer_id",
    total: { $sum: "$amount" },
    count: { $sum: 1 }
  }},
  { $sort: { total: -1 } },
  { $limit: 10 },
  { $lookup: {
    from: "customers",
    localField: "_id",
    foreignField: "_id",
    as: "customer"
  }},
  { $unwind: "$customer" },
  { $project: {
    customer_name: "$customer.name",
    total: 1,
    count: 1
  }}
]);
```

#### 索引

```javascript
// 单字段索引
db.users.createIndex({ email: 1 });

// 复合索引
db.orders.createIndex({ status: 1, created_at: -1 });

// 文本索引
db.articles.createIndex({ title: "text", body: "text" });
db.articles.find({ $text: { $search: "database optimization" } });

// TTL 索引（自动过期）
db.sessions.createIndex({ last_access: 1 }, { expireAfterSeconds: 3600 });

// 部分索引
db.users.createIndex(
  { email: 1 },
  { partialFilterExpression: { status: "active" } }
);
```

#### 副本集与分片

**副本集（Replica Set）**：一个 Primary 节点（处理写请求）+ 多个 Secondary 节点（数据复制，可承担读请求）。Primary 故障时自动选举新的 Primary。

**分片（Sharding）**：
- **分片键**决定数据分布策略（范围分片/哈希分片）
- **mongos** 路由节点，应用层通过 mongos 访问
- **Config Server** 存储集群元数据

### 4.2 Redis

Redis 是内存中的数据存储，以极高性能著称（单实例可达 10万+ QPS）。

#### 数据结构

```bash
# String（缓存、计数器）
SET user:1000:name "Alice"
INCR page_view:homepage
SETEX session:abc123 3600 "user_data"  # 带过期时间

# List（消息队列、最新列表）
LPUSH notifications:user1 "new_message"
BRPOP queues:email 0  # 阻塞弹出

# Set（标签、去重）
SADD user:1000:tags "python" "redis" "database"
SINTER user:1000:tags user:2000:tags  # 共同标签

# Sorted Set（排行榜、延迟队列）
ZADD leaderboard 100 "player1" 200 "player2"
ZREVRANGE leaderboard 0 9 WITHSCORES  # Top 10

# Hash（对象存储）
HSET user:1000 name "Alice" age 30 email "alice@example.com"
HGETALL user:1000

# Stream（消息队列，类似 Kafka）
XADD events * type "login" user_id 1000
XREAD COUNT 10 BLOCK 5000 STREAMS events 0

# Geospatial（地理位置）
GEOADD cities 116.4 39.9 "Beijing"
GEORADIUS cities 116.4 39.9 100 km  # 方圆 100km
```

#### 缓存策略模式

**Cache Aside（旁路缓存）**：
```
读：先查缓存 → 命中返回 | 未命中查DB → 写缓存 → 返回
写：先更新DB → 再删除缓存（或更新缓存）
```

```python
# Cache Aside 读
def get_user(user_id):
    user = redis.get(f"user:{user_id}")
    if user:
        return user
    user = db.query("SELECT * FROM users WHERE id = ?", user_id)
    if user:
        redis.setex(f"user:{user_id}", 3600, user)
    return user

# Cache Aside 写
def update_user(user_id, data):
    db.execute("UPDATE users SET ... WHERE id = ?", data, user_id)
    redis.delete(f"user:{user_id}")  # 删除缓存，而不是更新
```

> **为什么写时删除缓存而不是更新？** 更新缓存存在并发问题——两个写操作乱序可能导致缓存中出现旧数据。而删除缓存+下次读取时重建，是更安全的做法。

**Read-Through（穿透读）**：应用层只操作缓存，缓存负责从数据库加载数据（需要缓存中间件支持）。

**Write-Through（穿透写）**：数据同时写入缓存和数据库，保证一致性但增加写延迟。

**Write-Behind（异步写）**：先写入缓存，后台异步批量写入数据库，牺牲一致性换取高吞吐。

#### 分布式锁

```python
# Redlock 简化版：使用 SET NX EX
def acquire_lock(redis, lock_name, ttl_ms=10000):
    lock_key = f"lock:{lock_name}"
    lock_value = str(uuid.uuid4())  # 唯一标识
    if redis.set(lock_key, lock_value, nx=True, px=ttl_ms):
        return lock_value
    return None

def release_lock(redis, lock_name, lock_value):
    lock_key = f"lock:{lock_name}"
    # 使用 Lua 脚本保证原子性：只释放自己持有的锁
    script = """
    if redis.call("get", KEYS[1]) == ARGV[1] then
        return redis.call("del", KEYS[1])
    else
        return 0
    end
    """
    redis.eval(script, 1, lock_key, lock_value)
```

#### Pub/Sub

```bash
# 发布者
PUBLISH channel:notifications "Hello, everyone!"

# 订阅者
SUBSCRIBE channel:notifications
```

### 4.3 Elasticsearch

#### 倒排索引

Elasticsearch 的核心是倒排索引（Inverted Index）：

```
文档内容: "The quick brown fox jumps over the lazy dog"
             ↓
倒排索引:
  term    →  [doc_id, position]
  "the"   →  [(1,0), (1,7)]
  "quick" →  [(1,1)]
  "brown" →  [(1,2)]
  "fox"   →  [(1,3)]
```

#### 分析器

```json
// 自定义分析器
PUT /my_index
{
  "settings": {
    "analysis": {
      "analyzer": {
        "my_analyzer": {
          "type": "custom",
          "tokenizer": "standard",
          "filter": ["lowercase", "stop", "snowball"]
        }
      }
    }
  }
}
```

#### Query DSL

```json
// 全文搜索
GET /articles/_search
{
  "query": {
    "match": {
      "content": "database optimization"
    }
  }
}

// 复合查询
GET /products/_search
{
  "query": {
    "bool": {
      "must": [
        { "match": { "name": "laptop" } },
        { "range": { "price": { "gte": 5000, "lte": 15000 } } }
      ],
      "filter": [
        { "term": { "status": "active" } }
      ]
    }
  },
  "sort": [
    { "price": { "order": "asc" } }
  ],
  "aggs": {
    "by_brand": {
      "terms": { "field": "brand.keyword" }
    }
  }
}
```

**集群架构**：
- **Node**：单个 ES 实例
- **Index**：等同于关系型数据库的"表"
- **Shard**：索引的分片（每个分片是一个 Lucene 实例）
- **Replica**：分片的副本
- **Cluster**：多个 Node 组成的集群

---

## 五、ORM 深入

### 5.1 SQLAlchemy（Python）

```python
from sqlalchemy import create_engine, Column, Integer, String, ForeignKey, Text
from sqlalchemy.orm import declarative_base, relationship, Session
from sqlalchemy.orm import joinedload, selectinload

Base = declarative_base()

# 模型定义
class User(Base):
    __tablename__ = "users"
    id = Column(Integer, primary_key=True)
    name = Column(String(100), nullable=False)
    email = Column(String(200), unique=True)

    posts = relationship("Post", back_populates="author")

class Post(Base):
    __tablename__ = "posts"
    id = Column(Integer, primary_key=True)
    title = Column(String(200), nullable=False)
    content = Column(Text)
    user_id = Column(Integer, ForeignKey("users.id"))

    author = relationship("User", back_populates="posts")

# Session 生命周期
engine = create_engine("postgresql://user:pass@localhost/db")
session = Session(engine)

# 加载策略
# 懒加载（默认）：访问 posts 时才查询
user = session.get(User, 1)
print(user.posts)  # 此时执行 SELECT

# 预加载：一次 JOIN 查询
user = session.query(User).options(
    joinedload(User.posts)
).filter(User.id == 1).first()

# 子查询预加载：两次查询但避免笛卡尔积
user = session.query(User).options(
    selectinload(User.posts)
).filter(User.id == 1).first()
```

**Alembic 迁移**：
```bash
alembic init alembic
alembic revision --autogenerate -m "add user table"
alembic upgrade head
```

### 5.2 Prisma（TypeScript/Node.js）

```prisma
// schema.prisma
generator client {
  provider = "prisma-client-js"
}

datasource db {
  provider = "postgresql"
  url      = env("DATABASE_URL")
}

model User {
  id        Int      @id @default(autoincrement())
  email     String   @unique
  name      String
  posts     Post[]
  createdAt DateTime @default(now())
}

model Post {
  id        Int      @id @default(autoincrement())
  title     String
  content   String?
  published Boolean  @default(false)
  author    User     @relation(fields: [authorId], references: [id])
  authorId  Int
}
```

```typescript
// 客户端使用
import { PrismaClient } from '@prisma/client'
const prisma = new PrismaClient()

// 查询
const users = await prisma.user.findMany({
  where: { email: { contains: 'example.com' } },
  include: { posts: true },
  orderBy: { createdAt: 'desc' },
  take: 10,
  skip: 0
})

// 分页：游标分页
const page = await prisma.user.findMany({
  take: 10,
  cursor: { id: lastUserId },
  skip: 1,  // 跳过游标本身
  orderBy: { id: 'asc' }
})
```

### 5.3 TypeORM（TypeScript/Node.js）

```typescript
// 实体定义
@Entity()
export class User {
    @PrimaryGeneratedColumn()
    id: number;

    @Column({ unique: true })
    email: string;

    @Column()
    name: string;

    @OneToMany(() => Post, post => post.author)
    posts: Post[];

    @CreateDateColumn()
    createdAt: Date;
}

// Query Builder
const users = await dataSource
    .getRepository(User)
    .createQueryBuilder("user")
    .leftJoinAndSelect("user.posts", "post")
    .where("user.email LIKE :email", { email: "%@example.com" })
    .orderBy("user.createdAt", "DESC")
    .take(10)
    .getMany();

// 迁移
typeorm migration:create -n CreateUserTable
typeorm migration:run
```

### 5.4 N+1 问题与解决方案

**N+1 问题**：查询 1 次主表 + N 次关联查询。例如查询 100 个用户及其订单，如果懒加载，会执行 1 次 `SELECT * FROM users` + 100 次 `SELECT * FROM orders WHERE user_id = ?`。

**解决方案**：

1. **Eager Loading（预加载）**：
```python
# SQLAlchemy
users = session.query(User).options(joinedload(User.orders)).all()

# TypeORM
const users = await userRepository.find({ relations: ['orders'] });

# Prisma（自动使用 IN 查询）
const users = await prisma.user.findMany({ include: { orders: true } });
```

2. **Batch Loading（批量加载）**：
```python
# 手动批量加载
user_ids = [user.id for user in users]
orders = session.query(Order).filter(Order.user_id.in_(user_ids)).all()
# 然后按 user_id 分组
```

3. **DataLoader**（Facebook 的方案）：
```javascript
const DataLoader = require('dataloader');

const batchUsers = async (ids) => {
    const users = await db.user.findAll({ where: { id: ids } });
    return ids.map(id => users.find(u => u.id === id));
};

const userLoader = new DataLoader(batchUsers);
const user1 = await userLoader.load(1);
const user2 = await userLoader.load(2);
// 合并成一次查询
```

---

## 六、数据库设计

### 6.1 范式化

**第一范式（1NF）**：每个字段不可再分（原子性）。

❌ 违反 1NF：
| id | name | phones |
|----|------|--------|
| 1 | Alice | 138xxx, 139xxx |

✅ 符合 1NF：
| id | name | phone |
|----|------|-------|
| 1 | Alice | 138xxx |
| 2 | Alice | 139xxx |

**第二范式（2NF）**：在 1NF 基础上，每个非主键字段完全依赖于主键（消除部分依赖）。

❌ 违反 2NF（复合主键 `(order_id, product_id)`，但 `product_name` 只依赖于 `product_id`）：
| order_id | product_id | product_name | quantity |
|----------|-----------|-------------|----------|
| 1 | 100 | Laptop | 1 |
| 1 | 101 | Mouse | 2 |

✅ 符合 2NF：将 `product_name` 移到 `products` 表。

**第三范式（3NF）**：在 2NF 基础上，非主键字段不传递依赖于主键。

❌ 违反 3NF（`city` 通过 `zip_code` 与主键传递依赖）：
| id | name | zip_code | city |
|----|------|---------|------|
| 1 | Alice | 100000 | Beijing |

✅ 符合 3NF：将 `zip_code → city` 关系分离到 `zip_codes` 表。

**BCNF（Boyce-Codd Normal Form）**：3NF 的强化版——所有决定因素必须是候选键。大多数情况下满足 3NF 就足够了。

### 6.2 反范式化

**何时反范式化**：
- 读多写少的场景（如报表、数据仓库）
- 需要避免 JOIN 的查询热点
- 实时性要求高，无法承受 JOIN 开销

**权衡**：
```
范式化 → 存储少、更新快、数据一致 ← 写优化
反范式化 → 查询快、读性能好、冗余多 ← 读优化
```

**常见反范式化手段**：
- 在订单表中冗余「用户名」（避免 JOIN users 表）
- 在文章表中冗余「评论数」（避免 COUNT 查询）
- 反范式化需要应用程序维护数据一致，或通过触发器/事件同步

### 6.3 ER 建模

**实体-关系图（ERD）** 的核心元素：

- **实体**（Entity）：矩形，如 User、Order
- **属性**（Attribute）：椭圆，如 name、email
- **关系**（Relationship）：菱形，如 "创建"
- **基数**（Cardinality）：
  - `1:1`：一个人对应一个身份证号
  - `1:N`：一个用户有多个订单
  - `M:N`：一个学生可选多门课程，一门课程有多个学生

**设计模式示例**：
```
┌──────────┐    1:N    ┌──────────┐    M:N    ┌──────────┐
│  User    │───────────│  Order   │───────────│ Product  │
├──────────┤           ├──────────┤           ├──────────┤
│ id (PK)  │           │ id (PK)  │           │ id (PK)  │
│ name     │           │ user_id  │           │ name     │
│ email    │           │ total    │           │ price    │
└──────────┘           │ status   │           └──────────┘
                       └──────────┘
```

### 6.4 连接池

```python
# HikariCP (Java) - 默认配置即优秀
HikariConfig config = new HikariConfig();
config.setJdbcUrl("jdbc:postgresql://localhost/db");
config.setMaximumPoolSize(20);
config.setMinimumIdle(5);
config.setConnectionTimeout(30000);
config.setIdleTimeout(600000);
config.setMaxLifetime(1800000);

// pgBouncer (PostgreSQL 连接池代理)
// pgBouncer.ini
[databases]
mydb = host=127.0.0.1 port=5432

[pgbouncer]
pool_mode = transaction  -- 事务级复用
default_pool_size = 25
max_client_conn = 100
```

**连接池推荐大小**：`(core_count * 2) + effective_spindle_count`。对于大多数应用，10-30 个连接就足够了。过多的连接反而会导致数据库上下文切换开销。

### 6.5 迁移策略

**可逆迁移**：每个迁移必须提供 `up` 和 `down`（回滚）操作。

```python
# Alembic 迁移示例
def upgrade():
    op.add_column('users', sa.Column('phone', sa.String(20)))

def downgrade():
    op.drop_column('users', 'phone')
```

**零停机迁移原则**：
1. **加列**：安全，旧代码忽略新列
2. **删列**：先修改代码不再引用该列，部署后再删除
3. **重命名列**：分三步——加新列 → 双写（新旧同时更新） → 切换读取新列 → 删旧列
4. **大表加索引**：使用 `CONCURRENTLY`（PostgreSQL）避免锁表
```sql
CREATE INDEX CONCURRENTLY idx_name ON big_table(name);
```

### 6.6 备份与恢复

| 备份类型 | 特点 | 适用场景 |
|---------|------|---------|
| **逻辑备份**（mysqldump/pg_dump） | 生成 SQL 文件，可移植，恢复慢 | 小数据库、迁移 |
| **物理备份**（文件拷贝/XtraBackup） | 二进制拷贝，恢复快，体积大 | 大数据库、PITR |
| **快照备份**（LVM/ZFS） | 秒级完成，文件系统级别 | 需要快速恢复 |

**PITR（时间点恢复）**：全量备份 + WAL 归档（连续归档日志），可恢复到任意时间点。

```bash
# PostgreSQL PITR
# 1. 开启归档
archive_mode = on
archive_command = 'cp %p /backup/%f'

# 2. 全量备份
pg_basebackup -D /backup/base -F t -z

# 3. 恢复：使用全量备份 + 回放到指定时间
# recovery.conf
restore_command = 'cp /backup/%f %p'
recovery_target_time = '2024-03-15 14:30:00'
```

---

## 七、缓存策略

### 7.1 缓存模式对比

| 模式 | 读流程 | 写流程 | 一致性 | 复杂度 |
|------|-------|-------|--------|--------|
| **Cache Aside** | 先查缓存→未命中查DB→写缓存 | 写DB→删缓存 | 最终一致（有窗口期） | ⭐ 低 |
| **Read-Through** | 缓存层负责加载 | 写DB→删缓存 | 最终一致 | ⭐⭐ 中 |
| **Write-Through** | 同上 | 同时写缓存+DB | 强一致 | ⭐⭐ 中 |
| **Write-Behind** | 同上 | 只写缓存，异步写DB | 弱一致（有丢数据风险） | ⭐⭐⭐ 高 |

**Cache Aside 的缓存击穿问题**：
```
场景：热点 key 刚好过期，大量请求同时涌入
问题：所有请求都去查数据库，DB 被打垮
方案：互斥锁（只让一个请求查 DB 重建缓存）
```

```python
def get_hot_data(key):
    data = redis.get(key)
    if data:
        return data
    
    # 互斥锁重建缓存
    lock_key = f"lock:{key}"
    if redis.setnx(lock_key, 1, ex=5):  # 获取锁
        try:
            data = db.query("SELECT ...")
            redis.setex(key, 3600, data)
            return data
        finally:
            redis.delete(lock_key)
    else:
        time.sleep(0.1)
        return get_hot_data(key)  # 重试
```

### 7.2 缓存失效策略

**TTL（Time-To-Live）**：
- 最简单的失效方式，设置过期时间
- 问题：同一时刻大量缓存过期导致缓存雪崩
- 解决：过期时间加随机偏移 `TTL + random(0, 300)`

**事件驱动失效**：
- 数据变更时主动删除/更新缓存
- 使用消息队列解耦（更新DB→发消息→删缓存）
- 问题：消息可能丢失或延迟

**写穿透**：
- 每次写操作同时更新缓存和DB
- 保证缓存始终最新
- 问题：写延迟增加，不频繁读的数据也在缓存中

### 7.3 分布式缓存

| 方案 | 架构 | 数据分布 | 高可用 | 适用场景 |
|------|------|---------|-------|---------|
| **Redis Cluster** | 去中心化 | 哈希槽（16384个槽） | 主从切换 | 大型缓存、高写入 |
| **Redis Sentinel** | 主从+哨兵 | 单主写入 | 自动故障转移 | 中小规模、读多写少 |
| **Memcached** | 分布式客户端分片 | 一致性哈希 | 无复制（节点挂了丢数据） | 纯 KV、简单缓存 |

```bash
# Redis Cluster 创建
redis-cli --cluster create \
    192.168.1.1:7000 192.168.1.1:7001 \
    192.168.1.2:7000 192.168.1.2:7001 \
    192.168.1.3:7000 192.168.1.3:7001 \
    --cluster-replicas 1
```

### 7.4 CDN 缓存

CDN（Content Delivery Network）将静态资源缓存到全球边缘节点：

- 用户请求 → DNS 解析到最近的 CDN 节点
- 节点有缓存 → 直接返回
- 节点无缓存 → 回源站拉取 → 缓存并返回

**缓存控制**：
```
Edge-Control: max-age=86400  # CDN 缓存 1 天
CDN-Cache-Control: no-cache  # CDN 不缓存
```

### 7.5 浏览器缓存

**强缓存**（不请求服务器）：
```http
Cache-Control: max-age=3600          # HTTP/1.1 标准
Expires: Wed, 21 Oct 2024 07:28:00   # HTTP/1.0，已被 Cache-Control 替代
```

**协商缓存**（请求服务器，304 Not Modified 则复用缓存）：
```http
# 方式一：基于时间
Last-Modified: Wed, 21 Oct 2024 07:00:00 GMT  # 服务器返回
If-Modified-Since: Wed, 21 Oct 2024 07:00:00 GMT  # 客户端请求

# 方式二：基于内容哈希（推荐）
ETag: "33a64df551425fcc55e4d42a148795d9f25f89d4"  # 服务器返回
If-None-Match: "33a64df551425fcc55e4d42a148795d9f25f89d4"  # 客户端请求
```

**最佳实践**：
- 静态资源（JS/CSS/图片）使用强缓存 + 文件名带内容哈希
```html
<script src="/static/app.a1b2c3d4.js"></script>
```
- HTML 页面使用协商缓存（`no-cache`）
- 敏感数据不缓存（`no-store`）

---

> **本系列下一章**：[版本控制 →](./04-version-control.md) — Git 深入

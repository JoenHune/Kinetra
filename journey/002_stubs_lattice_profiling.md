# 002 — 消灭 Stubs + Lattice Planner + ARMv7 Profiling + 测试覆盖

**日期**: 2026-02-28  
**里程碑**: 所有 stub 实现补全、第四种规划器上线、性能基准就绪、测试从 69 增至 104

---

## 背景

在 000 和 001 完成后，一次全面审计发现项目约 72% 完成度。主要短板：

- **NLP Problem**：头文件声明了 10 个成员函数，全部未实现
- **Polygon Obstacles**：仅 5 行空壳
- **Scenario Loader**：仅 12 行空壳
- **Lattice Planner**：路线图中计划的第四种规划器，完全缺失
- **ARMv7 Profiling**：路线图目标，无基准数据
- **测试盲区**：STOMP、NLP、JSON Export、Polygon、Scenario 均无专属测试

本次迭代一口气解决以上全部问题。

---

## 完成内容

### 1. NLP Problem 实现 (~100 行)

`src/optimization/nlp_problem.cpp`

实现了 `NLPProblem` 所有成员函数：
- `variableValues()` / `setVariableValues()` —— 拼接/拆分多个 VariableSet
- `variableLowerBounds()` / `variableUpperBounds()` —— 汇总所有变量的上下界
- `constraintValues()` / `constraintLowerBounds()` / `constraintUpperBounds()` —— 汇总约束求值
- `constraintJacobian()` —— 逐个 ConstraintSet 计算并拼合稀疏 Jacobian
- `totalCost()` / `costGradient()` —— 汇总所有 CostTerm 求和

### 2. Polygon Obstacles — GJK 碰撞检测 (~185 行)

`src/collision/polygon_obstacles.cpp`

从 5 行空壳重写为完整的凸多边形碰撞检测：
- **GJK 核心**：`support()`、`supportMinkowski()`、`tripleProduct()`、`gjkIntersect()`
- **点在多边形**：`pointInConvexPolygon()` (ray casting)
- **有符号距离**：`signedDistanceToPolygon()`
- **栅格化**：`addPolygonObstacle()` 将凸多边形光栅化到占用网格
- **静态接口**：`isPolygonCollisionFree()` GJK 两凸多边形相交测试

### 3. Scenario Loader — 零依赖 JSON 解析器 (~270 行)

`src/io/scenario_loader.cpp`

从 12 行空壳重写为完整的场景 I/O：
- **递归下降解析器**：`JsonValue`（variant 存储）、`JsonParser` 类
- `loadScenarioJSON()` —— 从 JSON 文件反序列化 `PlanningProblem`
- `loadScenariosFromDir()` —— 扫描目录加载全部 `.json` 场景
- `saveScenarioJSON()` —— 序列化 `PlanningProblem` 为格式化 JSON
- 支持 circle / rectangle / polygon 三种障碍物类型

### 4. Lattice Planner — SE(2) 状态格子规划器 (~370 行)

`include/kinetra/planners/lattice.hpp` + `src/planners/lattice.cpp`

第四种规划器（RRT*、STOMP、iLQR 之后）：
- **运动基元生成**：每个离散朝向 × 多种转向角 → 弧线仿真 → MotionPrimitive
- **A\* 搜索**：优先队列 + g_scores 哈希表，加权启发式
- **碰撞检测**：对每条运动基元的中间点逐一检查
- **目标容差**：支持 `goalTolerance`，按栅格单元距离判定到达
- **路径重建**：包含运动基元中间点的密集轨迹输出
- **可配置项**：`xyResolution`、`numAngles`、`primitiveLength`、`maxSteerAngle`、`heuristicWeight`、`steerChangePenalty`

### 5. ARMv7 Profiling 基准 (~190 行)

`benchmarks/bench_profiling.cpp`

7 个基准套件，每个参数化运行：

| 基准 | 参数 | 计数器 |
|------|------|--------|
| BM_Profile_RRTStar | 1K/3K/5K 迭代 | nodes |
| BM_Profile_STOMP | 30/60/100 时间步 | — |
| BM_Profile_iLQR | 30/50/100 步长 | iters |
| BM_Profile_Lattice | 5K/20K/50K 展开 | expansions |
| BM_Profile_Dubins | — | queries |
| BM_Profile_QP | 10/50/100 变量 | iters |
| BM_Profile_SDF | 50/100/200 网格 | cells |

输出 JSON 格式，可直接对接 CI Dashboard。

### 6. 新增测试文件 (35 个新测试)

| 文件 | 测试数 | 覆盖模块 |
|------|--------|----------|
| `test_stomp.cpp` | 5 | STOMP 直线/对角/障碍/Reset/Name |
| `test_lattice.cpp` | 5 | Lattice 直线/障碍/基元/Name/Reset |
| `test_nlp.cpp` | 9 | NLP 变量/约束/Jacobian/Cost/Gradient/组合 |
| `test_json_export.cpp` | 6 | JSON Trajectory/Result/Environment/Polygon/HTML/Benchmark |
| `test_polygon_obstacles.cpp` | 6 | Polygon 栅格化/三角形/GJK分离/重叠/相同/包含 |
| `test_scenario_loader.cpp` | 4 | Scenario 加载保存/多障碍/目录扫描/异常 |

---

## 测试结果

```
104/104 tests passed, 0 tests failed
Total Test time (real) = 0.90 sec
```

从 69 个测试增长至 **104 个**，新增 35 个，0 回归。

---

## 文件变更汇总

### 新增文件
- `src/optimization/nlp_problem.cpp`
- `include/kinetra/planners/lattice.hpp`
- `src/planners/lattice.cpp`
- `benchmarks/bench_profiling.cpp`
- `tests/unit/test_stomp.cpp`
- `tests/unit/test_lattice.cpp`
- `tests/unit/test_nlp.cpp`
- `tests/unit/test_json_export.cpp`
- `tests/unit/test_polygon_obstacles.cpp`
- `tests/unit/test_scenario_loader.cpp`

### 修改文件
- `src/collision/polygon_obstacles.cpp` — 5 行 → ~185 行 (GJK)
- `src/io/scenario_loader.cpp` — 12 行 → ~270 行 (JSON parser)
- `include/kinetra/collision/occupancy_grid.hpp` — 新增 polygon 接口
- `CMakeLists.txt` — 新增 nlp_problem.cpp、lattice.cpp
- `benchmarks/CMakeLists.txt` — 新增 bench_profiling.cpp
- `tests/CMakeLists.txt` — 新增 6 个测试文件

---

## 下一步

根据路线图 (journey/000_genesis.md)，剩余关键项：

1. **NLP 求解器** — 有了 NLPProblem 组装框架，需要实际的 SQP/IPM 求解循环
2. **Interactive Canvas 增强** — 在 GitHub Pages 中可视化 Lattice 规划过程
3. **更多场景** — scenarios/ 目录填充典型测试用例 (parking, U-turn, narrow passage)
4. **文档完善** — Doxygen + API guide

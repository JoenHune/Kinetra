# 003 — SQP NLP 求解器 + Interactive Canvas 增强 + 规划场景

**日期**: 2026-02-28  
**里程碑**: NLP 求解器上线、Canvas 可视化增强 (Lattice A* + STOMP)、7 个规划场景、测试从 104 增至 111

---

## 背景

002 迭代完成了 NLPProblem 组装框架 (ifopt-style)、QPSolverADMM 以及 Lattice Planner,
其「下一步」中最关键的三项是：

1. **NLP 求解器** —— 有了 NLPProblem + QPSolverADMM，需要完整的 SQP 求解循环
2. **Interactive Canvas 增强** —— 在 GitHub Pages 中新增 Lattice A* + STOMP 可视化
3. **规划场景** —— scenarios/ 目录填充典型测试用例

---

## 完成内容

### 1. SQP NLP 求解器 (~360 行)

`include/kinetra/solvers/sqp.hpp` + `src/solvers/sqp.cpp`

基于 BFGS-SQP 架构的完整非线性规划求解器：

- **SQPSettings**：maxIterations、tolerance、constraintTolerance、merit 罚参数、Armijo 线搜索参数、时间限制
- **SQPResult**：最优 x、代价、约束违反量、迭代数、收敛状态
- **求解循环** (每次迭代):
  1. 在当前 x_k 上评估 NLP (f, ∇f, g, J)
  2. 构建 QP 子问题: `min ∇f'd + ½d'Hd  s.t. cl-g ≤ Jd ≤ cu-g, xl-x ≤ d ≤ xu-x`
  3. 调用 QPSolverADMM 求解 QP
  4. L1 merit 函数回溯线搜索 (Armijo 条件)
  5. **Powell 阻尼 BFGS 更新**: sᵀy ≥ 0.2·sᵀHs 时标准 BFGS，否则阻尼 r = θy+(1-θ)Hs
  6. 收敛判定: ‖step‖ < tol AND viol < tol, 或 ‖∇f‖ < tol AND 可行
  7. 自适应罚参数增长 (约束违反不降时 μ *= growth)

**关键技术决策**:
- 使用**代价梯度** (非 Lagrangian 梯度) 进行 BFGS 更新 —— ADMM 对偶变量噪声较大，
  Lagrangian 梯度会导致 Hessian 退化。纯代价梯度 + QP 约束处理更鲁棒。
- Powell 阻尼保证 BFGS 正定性
- 线搜索失败时采用缩小步长 (α·β) 而非盲目全步

### 2. 规划场景 (7 个 JSON 文件)

`scenarios/` 目录:

| 场景 | 文件 | 特征 |
|------|------|------|
| 窄通道 | `narrow_passage.json` | 垂直圆形墙壁, y≈0 处缺口 |
| 迷宫 | `maze.json` | 水平/垂直墙壁, 需多次转弯 |
| U 形转弯 | `u_turn.json` | 两个水平矩形墙壁 |
| 开阔场地 | `open_field.json` | 无障碍物基线 |
| 杂乱场地 | `cluttered_field.json` | 12 个散布圆形障碍物 |
| 发卡弯 | `hairpin.json` | 3 个大型障碍物, 需急转弯 |
| 平行泊车 | `parallel_parking.json` | 矩形车位墙壁 |

### 3. Interactive Canvas 增强 (~360 行新增)

`docs/index.html` + `docs/js/interactive_demo.js`

**新算法**:
- **Lattice A***: 运动原语生成 (8 角度 × 5 转向), A* 搜索 (最大展开 30000),
  路径重建含中间航点, 动画展示展开过程 (蓝色半透明矩形)
- **STOMP**: 直线初始化, 15 条噪声轨迹/迭代, exp-utility 加权 (温度 h=10),
  障碍物距离 + 平滑度代价, 动画迭代展示 (橙色半透明线条)

**新预设**:
- `Cluttered Field` —— 12 圆形障碍物散布
- `U-Turn` —— U 形通道

**UI 变更**:
- 算法选择器新增 "Lattice A*" 和 "STOMP" 选项
- STOMP Rollouts 图例条目 + CSS
- 两个新预设按钮

### 4. SQP 单元测试 (7 个 test case, ~290 行)

`tests/unit/test_sqp.cpp`

| 测试用例 | 描述 |
|----------|------|
| UnconstrainedQuadratic | min 0.5‖x‖²+c'x → x=[2,3] |
| BoxConstrainedQuadratic | 带变量下界 [0,0], 上界 [5,5] → x=[3,3] |
| LinearConstraint | x0+x1=1, min ‖x‖² → x=[0.5,0.5] |
| NonlinearConstraint | min ‖x‖²-2x0  s.t. x²+y²≤1 → x≈[1,0] |
| Rosenbrock | 经典 Rosenbrock (a=1, b=100) → x=[1,1] |
| EmptyProblem | 0 变量, 应直接返回 converged |
| FeasibilityFromInfeasibleStart | 从不可行点 (5,5) 出发, 验证 SQP 恢复可行性 |

**注意**: 所有测试辅助类使用匿名命名空间, 避免与 test_nlp.cpp 中同名类的 ODR 冲突。

---

## 调试纪实

### ODR (One Definition Rule) 冲突

`test_nlp.cpp` 和 `test_sqp.cpp` 各定义了一个 `QuadraticCost` 类 (同名但不同实现),
编译进同一个 `kinetra_tests` 二进制时触发了 ODR 冲突: 链接器选用了 `test_nlp.cpp`
的 vtable, 导致 `test_sqp.cpp` 中的 `fillGradientBlock` 调用被分派到错误的实现
(返回 `x` 而非 `Qx + c`), 梯度为零, SQP 认为已在最优点并报告 "Converged (zero step)"。

**解决**: 将 `test_sqp.cpp` 的辅助类封入 `namespace { }` (匿名命名空间)。

---

## 统计

| 指标 | 数值 |
|------|------|
| 新增文件 | 10 (sqp.hpp, sqp.cpp, test_sqp.cpp, 7 JSON) |
| 修改文件 | 4 (CMakeLists ×2, index.html, interactive_demo.js) |
| 新增行 | ~1800 |
| 测试 | 104 → 111 (+7), 100% 通过 |
| 算法 | +1 SQP NLP 求解器 |
| 可视化 | +2 算法 (Lattice A*, STOMP), +2 预设 |
| 场景 | +7 JSON 文件 |

---

## 下一步

1. **文档完善** —— Doxygen + API guide + README 更新
2. **IPM (内点法) 求解器** —— SQP 的替代方案, 适合大规模稀疏问题
3. **MPC 控制器** —— 将 SQP/iLQR 用于实时模型预测控制
4. **ROS 2 集成** —— nav2 plugin 接口
5. **更多测试场景** —— 动态障碍物、多车协同

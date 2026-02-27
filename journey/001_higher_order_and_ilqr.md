# 001 — 高阶模型与 iLQR 求解器

**日期**: 2026-02-28  
**里程碑**: Jerk/Snap 阶机器人模型 + 完整 iLQR 轨迹优化

---

## 背景

在 000 中我们搭建了项目骨架，包含 DiffDriveSimple/Accel、AckermannSimple/Accel、OmniSimple
五种速度/加速度阶模型，以及 RRT*、STOMP 两个规划器和一个 stub 状态的 iLQR。

本次迭代的目标：

1. **补全高阶模型**：为差速轮添加 Jerk（7 维）和 Snap（9 维），为阿克曼添加 Jerk（7 维）
2. **实现完整 iLQR 求解器**：从 stub 到可运行的迭代 LQR 轨迹优化
3. **清理 AI Agent 模块**：移除之前的 agent 相关代码，聚焦纯 C++ 库开发

## 新增模型

### DiffDriveJerk（7 维状态 + 2 维控制）

| 状态 | 含义 |
|------|------|
| x, y, θ | 位姿 |
| v, ω | 线速度、角速度 |
| a, α | 线加速度、角加速度 |

控制量 (j, jα) = 线性 jerk、角 jerk。Euler 积分后各阶逐级传导：

```
a' = a + j·dt,   α' = α + jα·dt
v' = v + a·dt,    ω' = ω + α·dt
θ' = θ + ω·dt
x' = x + v·cos(θ)·dt,  y' = y + v·sin(θ)·dt
```

### DiffDriveSnap（9 维状态 + 2 维控制）

在 Jerk 之上再加一阶：状态增加 (j, jα)，控制量为 (s, sα) = 线性 snap、角 snap。

### AckermannJerk（7 维状态 + 2 维控制）

| 状态 | 含义 |
|------|------|
| x, y, θ | 位姿 |
| v, φ | 线速度、转向角 |
| a, φ̇ | 线加速度、转向角速度 |

控制量 (j, φ̈) = 线性 jerk、转向角加速度。

所有新模型均满足 `RobotModel` 和 `LinearizableModel` concepts，提供完整的
`dynamics()`、`jacobianState()`、`jacobianControl()`、`stateLowerBound()`/`stateUpperBound()`。

## iLQR 实现

### 算法概述

iLQR (iterative Linear Quadratic Regulator) 是一种非线性轨迹优化方法：

1. **前向 rollout**：用当前控制序列 $u_{0..N-1}$ 和模型动力学 $f(x, u, dt)$ 前向展开得到轨迹 $x_{0..N}$
2. **代价展开**：在每个时间步对代价函数做二阶近似（$Q$, $R$, $Q_f$ 矩阵）
3. **后向传播**（Riccati 递推）：从终端代价开始后向，计算反馈增益 $K_k$ 和前馈修正 $d_k$
4. **前向通过 + 线搜索**：沿修正方向做带回溯的线搜索，找到一个降低总代价的步长 $\alpha$
5. **收敛判断**：期望改善量 $\Delta J$ 低于阈值则收敛

### 关键设计

- **模板化**：`iLQR<Model>` 对任意满足 `LinearizableModel` concept 的模型工作
- **Levenberg-Marquardt 正则化**：在 Riccati 递推中加 $\mu I$ 防止数值退化，自适应调整 $\mu$
- **回溯线搜索**：步长从 1.0 开始，以 $\beta=0.5$ 逐步缩减直到代价下降
- **控制钳位**：每步 rollout 后都 clamp 到 `[controlLowerBound, controlUpperBound]`
- **时间限制**：支持 `max_time_ms` 参数，在任意迭代结束时检查超时
- **header-only 实现**：`ilqr_impl.hpp` 包含全部模板实现，由 `ilqr.hpp` 末尾 include

### 接口

```cpp
// 直接求解（自定义代价矩阵）
Trajectory solveWithCost(
    const StateType& x0,
    const StateType& x_goal,
    const CostMatrices& cost,
    int horizon,
    Scalar dt
);

// 从 PlanningProblem 求解（自动构建代价矩阵）
Trajectory solve(const PlanningProblem& problem);
```

## 测试

### 新增模型测试（8 个）

| 测试 | 验证 |
|------|------|
| DiffDriveJerk.ConceptCheck | 满足 RobotModel + LinearizableModel |
| DiffDriveJerk.DynamicsPropagation | Euler 积分正确性 |
| DiffDriveJerk.JacobianFiniteDiff | 状态雅可比 vs 有限差分一致 |
| DiffDriveSnap.ConceptCheck | 满足 RobotModel + LinearizableModel |
| DiffDriveSnap.ChainedIntegration | 多步积分链式传导 |
| AckermannJerk.ConceptCheck | 满足 RobotModel + LinearizableModel |
| AckermannJerk.StraightLine | 直线行驶运动学 |
| AckermannJerk.JacobianFiniteDiff | 状态雅可比 vs 有限差分一致 |

### iLQR 测试（7 个）

| 测试 | 验证 |
|------|------|
| iLQR_DiffDriveSimple.SolveWithCost_StraightLine | 直线规划收敛 |
| iLQR_DiffDriveSimple.SolvePlanningProblem | PlanningProblem 接口 |
| iLQR_DiffDriveSimple.ResetClearsState | reset 清除内部状态 |
| iLQR_DiffDriveAccel.SolveWithCost | 加速度模型收敛 |
| iLQR_DiffDriveJerk.SolveWithCost | Jerk 模型收敛 |
| iLQR_AckermannSimple.SolveStraight | 阿克曼直线 |
| iLQR_OmniSimple.DiagonalMotion | 全向斜向运动 |

全部 **69 个测试通过**（54 原有 + 15 新增）。

## 清理

移除了项目中所有 AI Agent 相关代码：

- 删除 `agent/` 目录（Python 脚本、配置、orchestrator）
- 删除 `docs/js/agent_dashboard.js`
- 清理 `.gitignore`、`README.md`、`docs/index.html`、`journey/000_genesis.md` 中的 agent 引用

项目回归为纯 C++ 库 + 可视化仪表盘。

## 结果

| 指标 | 数值 |
|------|------|
| 新增 C++ header 代码 | ~600 行（3 模型 + iLQR 实现） |
| 新增测试 | 15 个（8 模型 + 7 iLQR） |
| 总测试数 | 69 |
| 测试通过率 | 100% |
| 构建时间 | < 10s（Release，M1 Mac）|

## 反思

### 学到了什么

1. **高阶模型的 clamp 边界交互需要注意**：Euler 积分中先 clamp 再使用，否则测试时初始值正好在边界上会导致混淆
2. **iLQR 的正则化至关重要**：不加 Levenberg-Marquardt 正则化，7 维 Jerk 模型的 Riccati 递推会数值退化
3. **header-only 模板实现**是 C++ 模板库的自然选择——避免了显式实例化的维护负担

### 下一步

- [ ] 添加 Lattice Planner 作为第四种规划器
- [ ] 性能 profiling：在 ARMv7 QEMU 上测量实际执行时间

---

> *"高阶模型让轨迹更丝滑，iLQR 让优化更优雅。Kinetra 开始有了灵魂。"*

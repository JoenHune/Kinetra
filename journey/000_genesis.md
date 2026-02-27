# 000 — 创世纪：从零到骨架

**日期**: 2026-02-28  
**里程碑**: 项目脚手架搭建完成

---

## 背景

我们的目标是打造世界上最好的轨迹规划算法库——**Kinetra**（Kinematic + Trajectory）。

核心约束：
- 基于 C++20，利用 concepts 实现零开销多态
- 完全 standalone，不依赖外部求解器，仅使用 Eigen（header-only）
- 最终产物可部署到 ARMv7 嵌入式芯片（如 Cortex-A7）
- 支持差速轮、阿克曼、全向三种机器人模型
- 支持多阶建模（速度、加速度、jerk、snap）
- 测试驱动开发 + AI Agent 迭代循环

## 调研

研究了 9 个现有库的架构与算法：

| 库 | 启发点 |
|----|--------|
| **OMPL** | 状态空间抽象、采样器架构 |
| **Drake** | MultibodyPlant 的建模思路 |
| **CasADi** | 符号微分与 NLP 公式化 |
| **ETH Control Toolbox** | iLQR/GNMS 实现模式 |
| **Crocoddyl** | Action Model 抽象 |
| **TrajOpt** | Sequential QP 轨迹优化 |
| **TEB** | 时间弹性带的实时规划 |
| **ifopt** | 可组合 NLP 问题公式化（VariableSet + ConstraintSet + CostTerm）|
| **MuJoCo MPC** | 嵌入式友好的 MPC 架构 |

## 决策

### 架构选型

1. **Concepts 替代虚函数** — 在 ARMv7 上避免 vtable 间接调用，编译期多态零开销
2. **ifopt 风格的 NLP 组合** — 用户可自由组合变量集、约束集、代价项来构建优化问题
3. **ADMM QP 求解器** — 参考 OSQP 的思路自研，完全无外部依赖
4. **Scalar 类型可切换** — 通过 `KINETRA_USE_FLOAT` 编译选项在 float/double 间切换，ARMv7 NEON 友好

### 模块划分

```
include/kinetra/
├── core/           # 基础类型、concepts、轨迹表示
├── spaces/         # SE(2)、Dubins、R^N 状态空间
├── robots/         # 差速轮、阿克曼、全向模型
├── optimization/   # 可组合 NLP 问题公式化
├── solvers/        # ADMM QP、LQR（Riccati）
├── planners/       # RRT*、STOMP、iLQR
├── collision/      # 占据栅格 + 有符号距离场
└── io/             # JSON 导出、HTML 可视化
```

### 机器人模型设计

每个模型满足 `RobotModel` 和 `LinearizableModel` concept：

| 模型 | 状态 | 控制 | 阶 |
|------|------|------|----|
| DiffDriveSimple | (x, y, θ) | (v, ω) | 速度 |
| DiffDriveAccel | (x, y, θ, v, ω) | (a, α) | 加速度 |
| AckermannSimple | (x, y, θ) | (v, δ) | 速度 |
| AckermannAccel | (x, y, θ, v, δ) | (a, δ̇) | 加速度 |
| OmniSimple | (x, y, θ) | (vx, vy, ω) | 速度 |

## 结果

本次迭代完成了完整的项目骨架：

- **80+ 文件** 创建完毕
- **C++ 库**: 17 个 header + 14 个 source file，含完整实现的：
  - Dubins 路径（6 种路径类型 LSL/LSR/RSL/RSR/RLR/LRL）
  - ADMM QP 求解器（自适应 ρ、warm start）
  - RRT*（带 rewire 的渐近最优采样规划）
  - STOMP（无梯度轨迹优化）
  - 占据栅格 + 有符号距离场（双向距离变换）
- **测试**: 8 个单元测试 + 1 个集成测试（GoogleTest）
- **基准测试**: 3 个 benchmark suite（Google Benchmark）
- **可视化**: GitHub Pages 暗色主题仪表盘（Plotly.js）+ 交互式 Canvas 规划 Demo
- **CI/CD**: GitHub Actions（原生 + 消毒器 + ARMv7 交叉编译 + Pages 部署）
- **交叉编译**: ARMv7 toolchain（NEON + hard-float）

## 反思

### 学到了什么

1. C++20 concepts 是 embedded-friendly 架构的杀手锏——编译期保证接口契约，零运行时开销
2. ifopt 风格的 NLP 组合模式非常优雅，但需要仔细设计索引管理
3. ADMM 求解器在中小规模 QP 上表现优秀，是 standalone 的最佳选择
4. Dubins 路径的 6 种类型虽然经典，但实现细节繁琐——trigonometric edge cases 很多

### 下一步

- [x] 编译通过：修复所有编译错误，确保 `cmake --build` 成功
- [x] 测试全绿：`ctest` 所有 54 个测试通过
- [x] 增加 jerk/snap 阶的机器人模型 → [001](001_higher_order_and_ilqr.md)
- [x] 实现 iLQR 求解器的完整版本 → [001](001_higher_order_and_ilqr.md)
- [ ] 添加 Lattice Planner 作为第四种规划器
- [ ] 性能 profiling：在 ARMv7 QEMU 上测量实际执行时间

---

> *"每一段旅程都始于第一步。Kinetra 的第一步，是让骨架站起来。"*

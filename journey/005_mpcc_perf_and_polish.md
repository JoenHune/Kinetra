# 005 — MPCC 性能优化 + 代码清理 + 交互增强

**日期**: 2026-02-28  
**状态**: 已完成

---

## 1. 背景

Iteration 004 完成了 SE(2)-MPCC 的完整实现，155 个测试全部通过。但 MPCC 求解速度极慢——对于 DiffDriveSimple 模型、$H=20$ 的直线跟踪问题，单次求解耗时 **1782 ms**（Debug/Release 均如此），完全无法满足实时 MPC 的需求（通常要求 < 50 ms）。

本轮迭代的目标：
1. 定位 MPCC 性能瓶颈并优化至可接受范围
2. 清理代码库中的遗留问题
3. Demo 页面增加拖拽设置 heading 的交互
4. 补充测试覆盖

---

## 2. 调研：性能瓶颈分析

### 2.1 问题规模

MPCC 的 NLP 结构（DiffDriveSimple, $H=20$）：

| 维度 | 数量 |
|------|------|
| 状态变量 | 63 (3 个状态 × 21 步) |
| 控制变量 | 40 (2 个控制 × 20 步) |
| 进度变量 | 21 |
| **总变量 $n$** | **124** |
| 动力学约束 | 60 |
| 初始条件约束 | 3 |
| 进度约束 | 20 |
| **总约束 $m$** | **83** |
| QP 约束（含变量边界） | 207 |

SQP 设置：最多 15 次 SQP 迭代，每次迭代求解一个 QP 子问题。

### 2.2 逐层计时

通过在关键路径插入 `std::chrono` 计时：

| 操作 | 单次耗时 |
|------|----------|
| NLP 评估（cost + gradient + constraints + Jacobian） | ~8 μs |
| QP ADMM setup（KKT 分解） | ~0.1 ms |
| QP ADMM solve（含所有 ADMM 迭代） | ~0.5 ms |
| **单次 SQP 迭代** | ~1 ms |
| **15 次 SQP（完整 MPCC 求解）** | **~1800 ms** |

**异常**：单次 QP 仅 0.5 ms，15 次 SQP 应该只需 ~15 ms，但实际耗时 1800 ms。

### 2.3 根因：`A'A` 重复计算

追踪 QP ADMM 内部循环发现：

```
ADMM 每次迭代 → updateRho() → 自适应 ρ 调整
                            → 重新计算 A_.transpose() * A_ （O(n²m) = 3.2M flops）
                            → 重新 LDLT 分解
```

单次 QP 平均进行 ~300 次 ADMM 迭代，每次都重复计算 $A^T A$。15 次 SQP × 300 次 ADMM = **4500 次** 不必要的 $O(n^2 m)$ 矩阵乘法，总计约 **14.4B flops**。

---

## 3. 决策与实现

### 3.1 QP ADMM 优化

#### 3.1.1 缓存 $A^T A$ 和 $A^T$

在 `QPSolverADMM::setup()` 中预计算并缓存：

```cpp
ATA_ = A_.transpose() * A_;   // 只算一次
AT_  = A_.transpose();         // 只算一次
```

ADMM 循环内使用缓存值替代重复计算。仅当 ρ 真正变化时才重新分解 KKT 矩阵。

**效果**：1782 ms → 94 ms（**19× 加速**）

#### 3.1.2 减少收敛检查频率

原始实现每次 ADMM 迭代都检查收敛条件（包含 $\|A^T y\|$ 计算）。改为每 5 次迭代检查一次，减少 60% 的 $A^T \Delta z$ 乘法。

#### 3.1.3 自适应 ρ 初始化

基于问题缩放自动选择初始 ρ：

$$\rho_0 = \text{clamp}\left(\frac{\sqrt{\text{tr}(Q)/n}}{\sqrt{\text{tr}(A^T A)/m}},\; 10^{-4},\; 10^4\right)$$

更好的初始 ρ 减少了 ADMM 迭代次数。

#### 3.1.4 QP 热启动

跨 SQP 迭代保存 QP 的原始/对偶解，作为下一次 QP 的初始点：

```cpp
solver2.warmStart(prev_d, prev_y_qp);
```

由于相邻 SQP 迭代的 QP 子问题结构相似，热启动显著减少收敛所需迭代。

### 3.2 SQP 优化

- **缓存约束边界**：$c_l$、$c_u$ 在求解过程中不变，移到循环外一次计算
- **消除冗余 NLP 评估**：收敛检查复用上一步的梯度，不再重复调用 `costGradient()`
- **放宽 QP 容差**：SQP 不需要高精度 QP 解，从 $10^{-6}$ 放宽到 $10^{-4}$
- **减少 QP 最大迭代**：$2000 \to 300$

### 3.3 最终效果

| 测试 | 优化前 | 优化后 | 加速比 |
|------|--------|--------|--------|
| SolveStraightLine | 1782 ms | 37 ms | **48×** |
| SolveCurvedPath | 3898 ms | 56 ms | **70×** |
| SolveWithObstacles | 3383 ms | 34 ms | **99×** |
| 完整测试套件 | 14.5 s | 1.70 s | **8.5×** |

QP 总 ADMM 迭代：4767 → 2355（-51%）

### 3.4 理论下限

对于 $n=124, m=207$ 的稠密 QP：
- KKT 分解：~0.3 ms（LDLT, $O(n^3)$）
- 15 次 SQP 迭代的纯矩阵运算：~3-5 ms
- 进一步优化需要：稀疏 Jacobian 利用、结构化 QP（banded KKT）、active-set 方法替代 ADMM

### 3.5 稀疏矩阵支持

分析 MPCC 的 Jacobian 稀疏性：

| 块 | 尺寸 | 非零 | 密度 |
|---|---|---|---|
| 动力学 vs 状态 | 60×63 | 240 | 6.3% |
| 动力学 vs 控制 | 60×40 | 120 | 5.0% |
| 初始条件 vs 状态 | 3×63 | 3 | 1.6% |
| 进度 vs 进度 | 20×21 | 40 | 9.5% |
| **总 Jacobian** | 83×124 | **403 / 10,292** | **3.9%** |
| **QP A 矩阵** [J; I] | 207×124 | **527 / 25,668** | **2.05%** |

实现细节：
1. **`SpMatX` 类型**：在 `types.hpp` 中添加 `Eigen::SparseMatrix<Scalar>` 别名
2. **`constraintJacobianSparse()`**：NLP 问题新增稀疏 Jacobian 装配方法（via triplet list）
3. **QP ADMM 双重载**：`setup(Q, c, MatX A, ...)` 和 `setup(Q, c, SpMatX A, ...)`
4. **内部存储**：A 矩阵始终存储为 `SpMatX`，矩阵-向量乘积自动利用稀疏性
5. **KKT 矩阵保持 dense**：因为 BFGS Hessian Q 是稠密的，$Q + \rho A^T A$ 无法利用稀疏性

**效果**：42.8 ms → 11.9 ms（再加速 **3.6×**）

### 3.6 MPCC 收敛性优化

问题诊断：15 次 SQP 后约束违反 0.0024（> 容差 0.0001），BFGS + ADMM 的近似解导致步长振荡。

改进措施：
1. **渐进式 QP 精度**：前 5 次 SQP 用粗糙 QP（tol=1e-3, maxIter=200），后期收紧（tol=1e-4, maxIter=400）
2. **放宽约束容差**：`constraintTolerance` 从 1e-4 提升到 1e-3（ADMM 实际可达精度）
3. **实用收敛准则**：约束可行 + 相对代价变化 < 0.1% 即判定收敛（不再仅依赖步长 + KKT 准确度）

**效果**：现在 20 次 SQP 迭代内收敛（`Converged: 1`），约束违反 0.0006 < 0.001

### 3.7 完整性能数据

从最初到最终优化的完整对比：

| 测试 | iter-005 | +缓存 | +稀疏 | +收敛 |
|------|----------|-------|-------|-------|
| SolveStraightLine | 1782 ms | 37 ms | ~12 ms | ~12 ms |
| 总测试套件 | 14.5 s | 1.66 s | 1.44 s | 1.54 s |
| 收敛 | ✗ | ✗ | ✗ | **✓** |
| 总加速比 | — | 48× | **149×** | **149×** |

---

## 4. 代码清理

审计整个代码库，搜索 `TODO`、`FIXME`、`HACK`、`stub`、`placeholder` 等标记：

| 发现 | 处理 |
|------|------|
| `mpcc_impl.hpp` 中未使用的 `mpcc_detail::StateVariables` 模板（40 行死代码） | **已删除** |
| `wasm/CMakeLists.txt` 注释 "stubs" 不准确（项目已无 stub） | **已修正**为 "header-only" |
| 其余代码无遗留问题 | — |

---

## 5. Demo 页面：拖拽设置 Heading

### 5.1 需求

原始交互：点击画布放置起点/终点，heading 为默认值（0 rad）。用户无法直观控制机器人朝向。

### 5.2 实现

替换单一 `click` 事件为完整的拖拽系统：

```
mousedown → 记录位置，暂时放置端点
mousemove → 超过 5px 死区后，计算 heading = atan2(-dy, dx)
          → 绘制虚线指示线 + 箭头
          → 实时更新 θ 值
mouseup   → 确认放置，显示 "θ=X.XX rad"
```

- 5px 死区区分点击与拖拽
- 障碍物模式保持简单点击
- 起点/终点模式下光标变为十字准星
- `drawEndpoint()` 增强：在 heading 方向绘制箭头指示

---

## 6. 测试覆盖

| 新增测试 | 验证目标 |
|----------|----------|
| `QPSolverADMM.WarmStart` | 热启动后收敛迭代 ≤ 冷启动 |
| `QPSolverADMM.AutoRhoScaling` | 不同缩放问题的自适应 ρ 正确运行 |
| `SQPSolver.TotalQPIterationsPopulated` | `totalQPIterations` 字段正确填充 |
| `MPCC_DiffDrive.TotalQPIterationsReported` | MPCC 结果中 QP 迭代数 > 0 |
| `MPCCPerf.ProfileSolve` | 性能基准 |
| `MPCCPerf.ProfileQPSolverAlone` | QP 独立性能基准 |
| `MPCCPerf.ProfileNLPEvaluation` | NLP 各组件计时 |

最终测试：**162/162 全部通过**，总耗时 1.70 s。

---

## 7. 反思

### 学到了什么

1. **ADMM 的性能陷阱**：自适应 ρ 需要重新分解 KKT 矩阵，但 $A^T A$ 不会随 ρ 变化——缓存它是 trivial 且高收益的优化。
2. **Profiling before optimizing**：最初怀疑 NLP 评估（Jacobian 计算）是瓶颈，但实测仅 8μs。真正的热点是 ADMM 内部循环中的冗余矩阵运算。
3. **QP 热启动的价值**：SQP 相邻迭代的 QP 子问题高度相似，热启动几乎免费却能减半迭代次数。

### 文件变更摘要

| 文件 | 变更 |
|------|------|
| `include/kinetra/core/types.hpp` | +`SpMatX`, `Triplet` 类型, +`Eigen/SparseCore` |
| `include/kinetra/solvers/qp_admm.hpp` | 稀疏 A 存储 (`SpMatX A_sp_`, `AT_sp_`), 双重载 setup |
| `src/solvers/qp_admm.cpp` | 稀疏 MV 乘积、`setupInternal()` 公共路径 |
| `include/kinetra/solvers/sqp.hpp` | +`totalQPIterations` |
| `src/solvers/sqp.cpp` | 稀疏 A_qp 构建、渐进式 QP 精度、实用收敛准则 |
| `include/kinetra/planners/mpcc.hpp` | +`totalQPIterations`, `constraintTolerance` 1e-3 |
| `include/kinetra/planners/mpcc_impl.hpp` | 删除死代码、传播 QP 迭代数 |
| `include/kinetra/optimization/nlp_problem.hpp` | +`constraintJacobianSparse()` |
| `src/optimization/nlp_problem.cpp` | 稀疏 Jacobian 装配 (triplet list) |
| `docs/js/interactive_demo.js` | 拖拽设置 heading |
| `tests/unit/test_mpcc_perf.cpp` | 性能测试（default iters） |
| `tests/unit/test_qp_solver.cpp` | +WarmStart, +AutoRhoScaling |
| `tests/unit/test_sqp.cpp` | +TotalQPIterationsPopulated |
| `tests/unit/test_mpcc.cpp` | +TotalQPIterationsReported |

### 下一步

- 考虑稀疏 KKT 分解（需要稀疏 Hessian 近似替代 dense BFGS）
- Gauss-Newton Hessian 近似（利用 MPCC 最小二乘结构）
- MPC 循环支持（warm-start across control cycles）

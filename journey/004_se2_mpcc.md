# 004 — SE(2)-MPCC：基于进度变量的路径跟踪轨迹优化

**日期**: 2026-02-28  
**状态**: 已实现 ✅

---

## 1. 问题动机

### 1.1 矛盾的根源

给定一条由全局规划器（RRT\*、Lattice A\* 等）生成的参考路径，局部轨迹优化器需要生成动力学可行的轨迹来跟踪该路径。在 Kinetra 的框架中，轨迹优化以固定时间步 $dt$ 和固定 horizon $N$ 构造 NLP——这意味着优化窗口覆盖的时间为 $T = N \cdot dt$，是一个常数。

对于差速机器人（DiffDriveSimple 到 DiffDriveSnap），不论模型阶数如何，在时间 $T$ 内的可达集（reachable set）是有限的。当参考路径长度 $L$ 远大于 $v_{\max} \cdot T$ 时，机器人不可能在一个优化窗口内到达路径终点。

### 1.2 传统方案的困境

| 方案 | 问题 |
|------|------|
| 固定终点约束 $\mathbf{x}_N = \mathbf{x}_{\text{goal}}$ | 路径过长时不可行（infeasible） |
| 启发式路径切分 | 切分点选取依赖速度估算，不精确；切分边界产生人为的子问题耦合 |
| 纯 tracking MPC（固定参考时间序列） | 需要预先做速度规划分配 $s(t)$；horizon 太短时参考点停滞，优化器失去前进动力 |

核心矛盾：**轨迹的合理终点是动力学、障碍约束与优化目标的耦合结果，无法在优化之前确定。**

### 1.3 MPCC 的核心洞察

> **把"终点在哪里"从问题参数变成决策变量。**

引入进度变量 $s_k$，表示第 $k$ 步沿参考路径的"任务进度"。优化器同时决定：
1. 机器人该怎么运动（状态/控制变量）
2. 此刻该跟踪路径的哪个位置（进度变量）

终端进度 $s_N$ 不是约束，而是目标函数的一部分——优化器在满足所有约束的前提下，自动找到可达的最大进度。

---

## 2. 相关工作

### 2.1 Contouring Control 的起源

MPCC 的思想源自 CNC 数控加工领域。Lam et al. [1] 首先提出将轮廓误差（contour error）和滞后误差（lag error）分离，用 MPC 框架同时最小化轮廓误差和最大化加工进度。这一思想后被推广到机器人领域。

### 2.2 Autonomous Racing

Liniger et al. [2] 将 MPCC 应用于自主赛车，建立了标准的 MPCC 公式：以弧长参数化赛道中心线，引入进度变量 $s$，以 $-\lambda \cdot (s_N - s_0)$ 作为进度奖励项。该工作展示了 MPCC 在高速、强非线性动力学下的有效性。

### 2.3 SE(2) 的推广

标准 MPCC 仅处理 $\mathbb{R}^2$ 中的位置跟踪。将其推广到 SE(2) 需要额外处理姿态（heading）一致性。Brito et al. [3] 在社会导航场景中引入了姿态误差项。Romero et al. [4] 在无人机竞速中将 MPCC 推广到 SE(3)。对于差速机器人，SE(2) 推广的关键在于参考路径包含 $\theta_r(s)$，进度 $s$ 可以在位置不变的情况下仅通过姿态变化推进——这自然支持原地旋转。

### 2.4 差速机器人轨迹优化

Rösmann et al. [5] 的 Timed Elastic Band (TEB) 通过可变时间间隔回避了固定 horizon 问题，但引入了时间间隔作为额外变量，增加了问题规模和非凸性。Mercy et al. [6] 使用 B-spline 参数化在弧长域做轨迹优化。Verscheure et al. [7] 证明了弧长参数化下的时间最优路径跟踪可以转化为凸优化，但要求精确跟踪（零误差），不适合需要绕障的场景。

### 2.5 Safe Corridor 方法

Liu et al. [8] 提出 Safe Flight Corridor，从参考路径生成一系列凸自由空间区域，轨迹约束在走廊内。Wang et al. [9] 的 MINCO 框架在走廊内做最小控制量轨迹优化。这些方法与 MPCC 互补：MPCC 处理路径跟踪 + 进度问题，Safe Corridor 处理障碍约束的凸化。

---

## 3. SE(2)-MPCC 完整公式

### 3.1 系统动力学

以 DiffDriveAccel（加速度控制）为例。状态：

$$\mathbf{x}_k = (x_k, y_k, \theta_k, v_k, \omega_k)^T \in \mathbb{R}^5$$

控制：

$$\mathbf{u}_k = (a_k, \alpha_k)^T \in \mathbb{R}^2$$

离散动力学（Euler 积分）：

$$\mathbf{x}_{k+1} = f(\mathbf{x}_k, \mathbf{u}_k, dt) = \begin{pmatrix} x_k + v_k \cos\theta_k \cdot dt \\ y_k + v_k \sin\theta_k \cdot dt \\ \theta_k + \omega_k \cdot dt \\ v_k + a_k \cdot dt \\ \omega_k + \alpha_k \cdot dt \end{pmatrix}$$

对于更高阶模型（DiffDriveJerk、DiffDriveSnap），状态维度增加但结构相同。

### 3.2 参考路径的 SE(2) 参数化

给定参考路径为有序 waypoint 序列 $\{(\bar{x}_i, \bar{y}_i, \bar{\theta}_i)\}_{i=0}^{M}$，构造连续参数化：

$$\mathbf{P}(s) = \big(x_r(s),\; y_r(s),\; \theta_r(s)\big), \quad s \in [0, S]$$

其中 $S$ 为路径总参数长度。参数 $s$ 的定义采用 **SE(2) 加权弧长**：

$$ds = \sqrt{dx_r^2 + dy_r^2 + \ell_\theta^2 \, d\theta_r^2}$$

其中 $\ell_\theta > 0$（单位 m/rad）将角度变化转换为等效位移。这确保：

- **纯平移段**：$d\theta_r = 0 \Rightarrow ds = \sqrt{dx_r^2 + dy_r^2}$（退化为几何弧长）
- **原地旋转段**：$dx_r = dy_r = 0 \Rightarrow ds = \ell_\theta \, |d\theta_r|$（$s$ 仍然递增）

实现上，$\mathbf{P}(s)$ 通过对 waypoint 间的线性插值（位置）和 SLERP（角度）构造，预计算累积弧长表以支持 $O(\log M)$ 的 $s \to \mathbf{P}(s)$ 查询。

### 3.3 Frenet-Serret 标架

在参考路径上 $s$ 处定义切向和法向：

$$\mathbf{t}(s) = \frac{1}{\|\dot{\mathbf{p}}_r(s)\|} \begin{pmatrix} \dot{x}_r(s) \\ \dot{y}_r(s) \end{pmatrix}, \quad \mathbf{n}(s) = \begin{pmatrix} -t_y(s) \\ t_x(s) \end{pmatrix}$$

其中 $\dot{\mathbf{p}}_r(s) = d\mathbf{p}_r / ds$。对于原地旋转段（$\|\dot{\mathbf{p}}_r\| = 0$），切向/法向退化，此时误差直接在笛卡尔坐标下计算（见 §3.4 退化处理）。

### 3.4 误差定义

定义三个误差分量：

**横向误差（Contour Error）**——轨迹点到参考路径的法向偏差：

$$e_{c,k} = \mathbf{n}(s_k)^T \begin{pmatrix} x_k - x_r(s_k) \\ y_k - y_r(s_k) \end{pmatrix}$$

**纵向误差（Lag Error）**——轨迹点沿切向的偏差：

$$e_{l,k} = \mathbf{t}(s_k)^T \begin{pmatrix} x_k - x_r(s_k) \\ y_k - y_r(s_k) \end{pmatrix}$$

**姿态误差（Heading Error）**——朝向偏差（wrapped to $[-\pi, \pi]$）：

$$e_{\theta,k} = \text{wrap}(\theta_k - \theta_r(s_k))$$

**退化处理**：当 $\|\dot{\mathbf{p}}_r(s_k)\| < \epsilon$（原地旋转段），设 $\mathbf{t} = (\cos\theta_r, \sin\theta_r)^T$，$\mathbf{n} = (-\sin\theta_r, \cos\theta_r)^T$，利用参考朝向代替路径切向。此时位置误差项自然为零（参考点不动），优化器主要通过姿态误差驱动 $\omega$ 控制。

### 3.5 决策变量

在标准动力学 NLP 的变量基础上，增加进度变量：

$$\text{决策变量} = \underbrace{\{\mathbf{x}_0, \dots, \mathbf{x}_N\}}_{\text{状态}} \cup \underbrace{\{\mathbf{u}_0, \dots, \mathbf{u}_{N-1}\}}_{\text{控制}} \cup \underbrace{\{s_0, \dots, s_N\}}_{\text{进度}}$$

总变量数：$(n_x + n_u) \cdot N + n_x + (N + 1)$，其中 $n_x$ 为状态维度，$n_u$ 为控制维度。

### 3.6 目标函数

$$J = \underbrace{\sum_{k=0}^{N} \left( w_c \, e_{c,k}^2 + w_l \, e_{l,k}^2 + w_\theta \, e_{\theta,k}^2 \right)}_{\text{SE(2) 跟踪代价}} + \underbrace{\sum_{k=0}^{N-1} \left( w_a \, a_k^2 + w_\alpha \, \alpha_k^2 \right)}_{\text{控制代价}} - \underbrace{\lambda \, s_N}_{\text{进度奖励}}$$

各项的作用：

| 项 | 作用 | 缺失后果 |
|----|------|----------|
| $w_c e_c^2$ | 保持轨迹贴近参考路径 | 轨迹大幅偏离路径 |
| $w_l e_l^2$ | 限制进度变量 $s_k$ 不能远超实际位置 | $s$ 脱缰跑到路径远处 |
| $w_\theta e_\theta^2$ | 保持朝向一致性 | 姿态振荡，差速机器人不可执行 |
| $w_a a_k^2 + w_\alpha \alpha_k^2$ | 平滑控制输入 | 剧烈加减速，执行器饱和 |
| $-\lambda s_N$ | 驱动沿路径前进 | 优化器无动力，轨迹停滞 |

对于更高阶模型（jerk/snap 控制），控制代价项替换为相应控制量的二次惩罚。

### 3.7 约束

**（a）动力学约束**（等式约束）：

$$\mathbf{x}_{k+1} = f(\mathbf{x}_k, \mathbf{u}_k, dt), \quad k = 0, \dots, N-1$$

**（b）初始状态约束**（等式约束）：

$$\mathbf{x}_0 = \mathbf{x}_{\text{init}}$$

**（c）状态界约束**（不等式约束）：

$$\mathbf{x}_{\min} \leq \mathbf{x}_k \leq \mathbf{x}_{\max}, \quad k = 0, \dots, N$$

包括速度限制 $|v_k| \leq v_{\max}$、角速度限制 $|\omega_k| \leq \omega_{\max}$ 等。

**（d）控制界约束**（不等式约束）：

$$\mathbf{u}_{\min} \leq \mathbf{u}_k \leq \mathbf{u}_{\max}, \quad k = 0, \dots, N-1$$

**（e）进度单调约束**（不等式约束）：

$$s_{k+1} \geq s_k, \quad k = 0, \dots, N-1$$

**（f）进度速率上界约束**（不等式约束）——**关键工程护栏**：

$$s_{k+1} - s_k \leq \bar{v}_s \cdot dt, \quad k = 0, \dots, N-1$$

其中 $\bar{v}_s$ 为进度的最大推进速率。若参考路径是纯位移段，$\bar{v}_s \approx v_{\max}$；若包含旋转段，$\bar{v}_s$ 需根据 $\ell_\theta$ 和 $\omega_{\max}$ 联合确定：

$$\bar{v}_s = \sqrt{v_{\max}^2 + \ell_\theta^2 \, \omega_{\max}^2}$$

此约束防止 $s$ 跳跃到路径远处——这是 MPCC 实现中 **最常见的失败模式**。

**（g）进度范围约束**：

$$s_{\text{init}} \leq s_k \leq S, \quad k = 0, \dots, N$$

其中 $s_{\text{init}}$ 为当前已达进度（MPC 滚动时更新），$S$ 为路径总长。

**（h）纵向误差约束**（可选，推荐）——**第二道工程护栏**：

$$|e_{l,k}| \leq \bar{e}_l, \quad k = 0, \dots, N$$

防止 cost 中 lag 项的权重不足以阻止 $s$ 与实际位置脱耦。建议 $\bar{e}_l \in [0.5, 2.0]$ m，视应用精度而定。

**（i）避障约束**（不等式约束）：

$$d(\mathbf{p}_k, \mathcal{O}_j) \geq d_{\text{safe}}, \quad \forall k, \forall j$$

其中 $d(\cdot, \mathcal{O}_j)$ 为到第 $j$ 个障碍物的有符号距离。Kinetra 已有 SDF 基础设施可直接复用。

### 3.8 完整 NLP 汇总

$$\boxed{\begin{aligned}
\min_{\mathbf{x}, \mathbf{u}, \mathbf{s}} \quad & \sum_{k=0}^{N} \left( w_c e_{c,k}^2 + w_l e_{l,k}^2 + w_\theta e_\theta^2 \right) + \sum_{k=0}^{N-1} \mathbf{u}_k^T W_u \, \mathbf{u}_k - \lambda \, s_N \\[6pt]
\text{s.t.} \quad & \mathbf{x}_{k+1} = f(\mathbf{x}_k, \mathbf{u}_k, dt) & \forall k \\
& \mathbf{x}_0 = \mathbf{x}_{\text{init}} \\
& \mathbf{x}_{\min} \leq \mathbf{x}_k \leq \mathbf{x}_{\max} & \forall k \\
& \mathbf{u}_{\min} \leq \mathbf{u}_k \leq \mathbf{u}_{\max} & \forall k \\
& 0 \leq s_{k+1} - s_k \leq \bar{v}_s \cdot dt & \forall k \\
& s_{\text{init}} \leq s_k \leq S & \forall k \\
& d(\mathbf{p}_k, \mathcal{O}_j) \geq d_{\text{safe}} & \forall k, j
\end{aligned}}$$

---

## 4. 梯度结构与求解

### 4.1 误差对决策变量的梯度

误差项 $e_c$、$e_l$、$e_\theta$ 对状态变量 $(x_k, y_k, \theta_k)$ 的梯度较直接：

$$\frac{\partial e_{c,k}}{\partial x_k} = n_x(s_k), \quad \frac{\partial e_{c,k}}{\partial y_k} = n_y(s_k)$$

$$\frac{\partial e_{l,k}}{\partial x_k} = t_x(s_k), \quad \frac{\partial e_{l,k}}{\partial y_k} = t_y(s_k)$$

$$\frac{\partial e_{\theta,k}}{\partial \theta_k} = 1$$

关键的复杂性在于 **误差对进度变量 $s_k$ 的梯度**，它涉及参考路径的曲率 $\kappa(s)$：

$$\frac{\partial e_{c,k}}{\partial s_k} = -\mathbf{n}^T \dot{\mathbf{p}}_r - \kappa(s_k) \, \mathbf{t}^T \Delta\mathbf{p}_k$$

$$\frac{\partial e_{l,k}}{\partial s_k} = -\mathbf{t}^T \dot{\mathbf{p}}_r + \kappa(s_k) \, \mathbf{n}^T \Delta\mathbf{p}_k$$

$$\frac{\partial e_{\theta,k}}{\partial s_k} = -\dot{\theta}_r(s_k)$$

其中 $\Delta\mathbf{p}_k = (x_k - x_r(s_k), \; y_k - y_r(s_k))^T$，$\kappa(s)$ 为参考路径曲率。

### 4.2 曲率奇异性与路径预光滑

当参考路径包含尖角（$\kappa \to \infty$）或曲率突变时，上述梯度可能爆炸或不连续。解决方案：

1. **路径预光滑**：对参考 waypoint 做 B-spline 或 cubic spline 拟合，保证 $C^2$ 连续性
2. **曲率截断**：$\kappa_{\text{eff}} = \text{clamp}(\kappa, -\bar{\kappa}, \bar{\kappa})$
3. **有限差分梯度**：对 $\partial e / \partial s$ 用数值差分代替解析式，牺牲精度换鲁棒性

推荐在 Kinetra 中采用方案 1 + 2：B-spline 路径参数化 + 曲率截断。

### 4.3 与 Kinetra SQP 的集成

Kinetra 已有 BFGS-SQP 求解器（Powell 阻尼 + ADMM QP 子问题）。SE(2)-MPCC NLP 通过 `NLPProblem` 组装：

| NLP 组件 | Kinetra 类 | 内容 |
|----------|-----------|------|
| 状态变量 | `VariableSet` | $\mathbf{x}_0, \dots, \mathbf{x}_N$，带状态界 |
| 控制变量 | `VariableSet` | $\mathbf{u}_0, \dots, \mathbf{u}_{N-1}$，带控制界 |
| 进度变量 | `VariableSet` | $s_0, \dots, s_N$，带 $[s_{\text{init}}, S]$ 界 |
| 动力学约束 | `ConstraintSet` | $\mathbf{x}_{k+1} - f(\mathbf{x}_k, \mathbf{u}_k, dt) = 0$ |
| 进度单调+速率 | `ConstraintSet` | $0 \leq s_{k+1} - s_k \leq \bar{v}_s \cdot dt$ |
| 避障约束 | `ConstraintSet` | $d(\mathbf{p}_k, \mathcal{O}_j) \geq d_{\text{safe}}$ |
| SE(2) 跟踪代价 | `CostTerm` | $\sum w_c e_c^2 + w_l e_l^2 + w_\theta e_\theta^2$ |
| 控制代价 | `CostTerm` | $\sum \mathbf{u}^T W_u \mathbf{u}$ |
| 进度奖励 | `CostTerm` | $-\lambda \, s_N$ |

SQP 每次迭代中，cost gradient 和 constraint Jacobian 按 §4.1 的解析式填充各 block。

### 4.4 NLP 规模

| 模型 | $n_x$ | $n_u$ | 每 horizon 步变量数 | N=50 总变量 | N=50 总约束（估）|
|------|--------|--------|-------------------|------------|----------------|
| DiffDriveSimple | 3 | 2 | 6 | 306 | ~500 |
| DiffDriveAccel | 5 | 2 | 8 | 408 | ~650 |
| DiffDriveJerk | 7 | 2 | 10 | 510 | ~800 |
| DiffDriveSnap | 9 | 2 | 12 | 612 | ~950 |

注：每步增加 1 个进度变量。ADMM QP 子问题在此规模下可在 ms 量级求解。

---

## 5. MPC 滚动执行

SE(2)-MPCC 的自然执行模式是 Receding Horizon MPC：

```
算法: SE(2)-MPCC MPC Loop
────────────────────────────────
输入: 参考路径 P(s), 初始状态 x_init, 初始进度 s_init
参数: N, dt, 所有权重和约束参数

s_current ← s_init
x_current ← x_init

循环:
  1. 构造 NLP:
     - x_0 = x_current
     - s 的活跃窗口 = [s_current, min(s_current + N·v̄_s·dt, S)]
     - warm start: 上一次解平移一步

  2. 调用 SQP 求解

  3. 提取前 K 步控制: u_0, ..., u_{K-1}  (K ≪ N, 通常 K=1~5)

  4. 执行控制，更新:
     - x_current ← 实际状态（闭环）
     - s_current ← s*_K（求解得到的第 K 步进度）

  5. 若 s_current ≈ S 且 ‖x_current - P(S)‖ < ε: 完成
```

### 5.1 Warm Start 策略

每次 MPC 迭代的初始猜测对 SQP 收敛速度至关重要：

- **状态和控制**：将上一次的解平移 $K$ 步，尾部用零控制前向传播补齐
- **进度变量**：平移 $K$ 步，尾部按 $\bar{v}_s \cdot dt$ 线性外推
- **Hessian 近似**：保留上一次的 BFGS Hessian（warm BFGS）

### 5.2 活跃窗口

参考路径 $\mathbf{P}(s)$ 可能含数千个 waypoint，但每次 NLP 只需查询 $s \in [s_{\text{current}}, s_{\text{current}} + N \bar{v}_s \, dt]$ 范围内的子段。预计算累积弧长表后，通过二分查找定位活跃 waypoint 范围，避免全路径遍历。

---

## 6. 与其他方案的比较

### 6.1 vs. 两阶段解耦（速度规划 + Tracking MPC）

两阶段方法 [7] 先在参考路径上求解时间最优速度 profile $s^*(t)$（凸优化），再以 $\mathbf{P}(s^*(t))$ 为参考做标准 tracking MPC。

| 维度 | SE(2)-MPCC | 两阶段解耦 |
|------|-----------|-----------|
| 速度-跟踪耦合 | 联合优化（更优） | 解耦（次优但更快） |
| 计算量 | 较高（多 $N+1$ 变量） | 较低 |
| 速度层全局最优性 | ❌ 局部 | ✅ 凸优化保证 |
| 绕障灵活性 | ✅ 自然支持 | ⚠️ 速度层不感知障碍 |

在计算资源极度受限的场景（ARMv7），两阶段解耦可能是更务实的选择。

### 6.2 vs. Timed Elastic Band (TEB) [5]

TEB 将时间间隔 $\Delta t_k$ 也作为优化变量，从根本上消除固定 $dt$ 的限制。

| 维度 | SE(2)-MPCC | TEB |
|------|-----------|-----|
| 时间参数化 | 固定 $dt$ | 可变 $\Delta t_k$ |
| 下游兼容性 | 直接输出等间隔轨迹 | 需 resample |
| 多拓扑探索 | ❌ | ✅（多 TEB 并行） |
| 高阶动力学 | ✅ 自然支持 | ⚠️ 变 $\Delta t$ 使动力学约束复杂化 |

### 6.3 vs. Safe Flight Corridor [8, 9]

Safe Corridor 适合障碍密集、需要大幅偏离参考路径的场景。可以与 MPCC **结合使用**：用走廊约束替换点对点避障约束 (§3.7 (i))，将非凸避障转化为线性约束。

### 6.4 vs. Projection-Based Tracking（无进度变量）

最简约的方案：不引入 $s_k$，每个轨迹点在线投影到参考路径最近点，最小化投影距离。

| 维度 | SE(2)-MPCC | Projection-Based |
|------|-----------|-----------------|
| 额外变量 | $N+1$ 个 $s_k$ | 零 |
| 前进驱动力 | $-\lambda s_N$（显式） | 隐式（靠 terminal cost 或参考点前移启发式）|
| 路径自交叉鲁棒性 | ✅（$s$ 单调） | ❌（可能跳到错误的最近点） |
| 实现复杂度 | 中 | 低 |

---

## 7. 已知局限性与开放问题

### 7.1 局部最优性

MPCC NLP 是非凸的（非线性动力学 + 非凸避障约束）。SQP 只能保证局部最优。在有多个拓扑可选的环境中（如障碍物可左绕或右绕），需要外层规划器（如 RRT\*、Lattice A\*）提供拓扑正确的参考路径。

### 7.2 进度-位置脱耦风险

即使有 lag 约束 (§3.7 (h)) 和进度速率上界 (§3.7 (f))，在极端场景（如急转弯后的直道）中，$s$ 仍可能超前于实际位置。实践中需要调参：$w_l$ 和 $\bar{e}_l$ 需要与路径曲率匹配。

### 7.3 原地旋转段的数值问题

在原地旋转段，$\dot{\mathbf{p}}_r(s) \approx 0$，切向/法向退化。§3.4 给出了退化处理策略，但在旋转段与平移段的过渡区域，梯度可能出现跳变。建议在路径参数化阶段对过渡区域做 $C^1$ 光滑处理。

### 7.4 参数选择

权重 $(w_c, w_l, w_\theta, w_a, w_\alpha, \lambda)$ 和约束参数 $(\bar{v}_s, \bar{e}_l, \ell_\theta)$ 的选取对性能影响显著，目前没有自适应调参的理论保证。实践中需要针对场景类型（开阔 / 狭窄 / 旋转密集）维护不同的参数集。

---

## 8. 参考文献

[1] D. Lam, C. Manzie, and M. Good, "Model predictive contouring control," in _Proc. 49th IEEE Conf. Decision and Control (CDC)_, 2010, pp. 6137–6142.

[2] A. Liniger, A. Domahidi, and M. Morari, "Optimization-based autonomous racing of 1:43 scale RC cars," _Optimal Control Applications and Methods_, vol. 36, no. 5, pp. 628–647, 2015.

[3] B. Brito, B. Floor, L. Ferranti, and J. Alonso-Mora, "Model predictive contouring control for collision avoidance in unstructured dynamic environments," _IEEE Robotics and Automation Letters_, vol. 4, no. 4, pp. 4459–4466, 2019.

[4] J. Romero, S. Sun, P. Foehn, and D. Scaramuzza, "Model predictive contouring control for time-optimal quadrotor flight," _IEEE Trans. Robotics_, vol. 38, no. 6, pp. 3340–3356, 2022.

[5] C. Rösmann, W. Feiten, T. Wösch, F. Hoffmann, and T. Bertram, "Trajectory modification considering dynamic constraints of autonomous robots," in _Proc. German Conf. Robotics (ROBOTIK)_, 2012, pp. 74–79.

[6] T. Mercy, R. Van Parys, and G. Pipeleers, "Spline-based motion planning for autonomous guided vehicles in a dynamic environment," _IEEE Trans. Control Systems Technology_, vol. 26, no. 6, pp. 2182–2189, 2018.

[7] D. Verscheure, B. Demeulenaere, J. Swevers, J. De Schutter, and M. Diehl, "Time-optimal path tracking for robots: A convex optimization approach," _IEEE Trans. Automatic Control_, vol. 54, no. 10, pp. 2318–2327, 2009.

[8] S. Liu, M. Watterson, K. Mohta, K. Sun, S. Bhatt, C. J. Taylor, and V. Kumar, "Planning dynamically feasible trajectories for quadrotors using safe flight corridors in 3-D complex environments," _IEEE Robotics and Automation Letters_, vol. 2, no. 3, pp. 1688–1695, 2017.

[9] Z. Wang, X. Zhou, C. Xu, and F. Gao, "Geometrically constrained trajectory optimization for multicopters," _IEEE Trans. Robotics_, vol. 38, no. 5, pp. 3259–3278, 2022.

---

## 附录 A：推荐参数范围

以下参数基于差速机器人（$v_{\max} = 1.0$ m/s, $\omega_{\max} = 1.5$ rad/s, $dt = 0.1$ s）的经验值：

| 参数 | 符号 | 推荐范围 | 说明 |
|------|------|----------|------|
| 横向误差权重 | $w_c$ | 10 – 100 | 值越大轨迹越贴路径 |
| 纵向误差权重 | $w_l$ | 5 – 50 | 防止 $s$ 脱耦 |
| 姿态误差权重 | $w_\theta$ | 5 – 50 | 差速机器人建议偏高 |
| 控制权重 | $w_a, w_\alpha$ | 0.1 – 10 | 根据执行器能力调整 |
| 进度奖励 | $\lambda$ | 1 – 20 | 过高导致贪进度、偏路径 |
| 角度等效长度 | $\ell_\theta$ | 0.3 – 1.0 m/rad | 越大越重视旋转 |
| 进度速率上界 | $\bar{v}_s$ | $1.2 \times v_{\max}$ | 留 20% 余量 |
| 纵向误差上界 | $\bar{e}_l$ | 0.5 – 2.0 m | 窄通道取小值 |
| Horizon | $N$ | 30 – 80 | 平衡计算量与前瞻性 |

## 附录 B：与 Kinetra 现有组件的映射

```
SE(2)-MPCC 模块                       Kinetra 现有组件
─────────────────────                 ──────────────────
差速动力学 f(x,u,dt)           ←→     DiffDriveAccel::dynamics()
状态/控制雅可比                ←→     DiffDriveAccel::jacobianState/Control()
NLP 变量、约束、代价组装       ←→     NLPProblem + VariableSet + ConstraintSet + CostTerm
SQP 求解                      ←→     solvers/sqp.hpp (BFGS-SQP + ADMM QP)
避障有符号距离                 ←→     collision/ (OccupancyGrid + SDF)
参考路径表示                   ←→     Trajectory2D (需扩展弧长参数化)
MPC 外层循环                   ←→     待实现
参考路径光滑化                 ←→     待实现 (B-spline fitting)
```

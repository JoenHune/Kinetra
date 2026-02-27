# 旅程 / Journey

本目录记录 Kinetra 仓库的进化历程。每一篇日志对应一个里程碑或重要决策。

This directory chronicles the evolution of the Kinetra repository.
Each entry corresponds to a milestone or key architectural decision.

## 目录 / Index

| # | 日期 | 标题 | 文件 |
|---|------|------|------|
| 0 | 2026-02-28 | 创世纪：从零到骨架 | [000_genesis.md](000_genesis.md) |
| 1 | 2026-02-28 | 高阶模型与 iLQR 求解器 | [001_higher_order_and_ilqr.md](001_higher_order_and_ilqr.md) |
| 2 | 2026-02-28 | 消灭 Stubs + Lattice Planner + Profiling | [002_stubs_lattice_profiling.md](002_stubs_lattice_profiling.md) |
| 3 | 2026-02-28 | SQP 求解器 + Canvas 增强 + 规划场景 | [003_sqp_canvas_scenarios.md](003_sqp_canvas_scenarios.md) |
| 4 | 2026-02-28 | SE(2)-MPCC：基于进度变量的路径跟踪轨迹优化 | [004_se2_mpcc.md](004_se2_mpcc.md) |

---

### 如何撰写旅程日志

每篇日志应包含：

1. **背景** — 当时面临的问题或目标
2. **调研** — 参考了什么论文、库、方法
3. **决策** — 做了什么决定，为什么
4. **结果** — 实现了什么，跑通了哪些测试
5. **反思** — 学到了什么，下一步是什么

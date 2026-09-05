# OpenVSP/VSPAERO 气动数据库工作流：详细说明

## 1. 目标与责任边界

项目从 OpenVSP `.vsp3` 生成 uniform/adaptive GRID、纵向 TRIM、23 项 production 气动导数，并导出 JSON、CSV 和 MATLAB `.mat`。工作流验证 VSPAERO 完整性、Wake、FD 步长、TRIM、导数来源和数据结构；Tessellation/mesh 由用户负责，脚本不修改也不认证网格独立性。

完整数据流为：

```text
模型与配置 identity
  -> 几何集、参考量、舵面和当前网格可求解性
  -> 基础系数 Wake 3/5/8/12（必要时 Wake16 verification）
  -> alpha/beta/control 的真实 centered FD 与逐项步长选择
  -> steady .stab 的经典 p/q/r rate derivatives
  -> Solver Gate + Derivative Gate -> Production Gate
  -> uniform/adaptive GRID + 两阶段 TRIM
  -> validation -> JSON/CSV/MAT
```

## 2. 23 项 production derivative

`config/required_derivatives.yaml` 是唯一清单。最终权威集合是：

```text
derivatives.production_derivatives
```

`production_derivatives` 是唯一正式 production 集合：其中 15 项的 `method` 为
`centered_finite_difference`，8 项为 `vspaero_steady_rate_derivative`。FD 步长、
rate 来源与原始字段均保存在各自记录内，不再输出第二套重复 production 集合。

每条记录至少包含 `name, value, units, method, source, source_field, wake_iterations, validation_status, coordinate_sign_convention, production_included`。JSON、CSV、MAT、validation 和 regression 均以该集合为正式来源。

### 2.1 steady p/q/r 的真实语义

OpenVSP 3.51.3 的 `VSPAEROSweep` 实际输入使用 `UnsteadyType`。枚举为 steady default 与 P/Q/R analysis；steady 会生成 `.stab`。VSPAERO 3.51.3 求解器实现对 rate perturbation 使用：

```text
dC/dp_hat: ΔC / [Δp * bref/(2Vinf)]
dC/dq_hat: ΔC / [Δq * cref/(2Vinf)]
dC/dr_hat: ΔC / [Δr * bref/(2Vinf)]
```

所以 steady `.stab` 的 `CL_q, CMm_q, CFy_p, CMl_p, CMn_p, CFy_r, CMl_r, CMn_r` 是本项目需要的经典无量纲速率导数。它们的方法标为 `vspaero_steady_rate_derivative`，而非 centered FD，但方法差异本身不构成警告。

### 2.2 steady 与 P/Q/R unsteady 的区别

`STABILITY_DEFAULT` 用小稳态扰动形成包括 wrt p/q/r 的稳定性表。`STABILITY_P/Q/R_ANALYSIS` 则运行时间相关 damping analysis，生成 `.pstab/.qstab/.rstab`。后者可包含：

- P：p damping 诊断；
- Q：`q + alpha_dot` 组合量；
- R：`r - beta_dot` 组合量。

组合量不是独立的 `q` 或 `r` 偏导；除非求解器同时给出足够的独立方程并经过验证，否则不能拆分。因此 `q+alpha_dot`、`r-beta_dot` 只保存在 `unsteady_derivative_diagnostics`，永不写入 required `CL_q/Cm_q/CY_r/Cl_r/Cn_r`。

OpenVSP 3.51.3 在 P/Q/R 分支内部把 `NumberOfTimeSteps` 设为 128，外部 sweep 的较小 `NumTimeSteps` 不会缩短它。生产 8 项无需运行这些昂贵诊断，默认 `solver.run_unsteady_diagnostics: false`；显式开启后脚本会自动运行和解析，用户无需手工操作。

### 2.3 单位与坐标符号

项目标准体轴为 `+X forward, +Y right, +Z down`，右手系；OpenVSP 几何/分量字段为 `+X aft, +Y right, +Z up`。唯一映射位于 `src/coordinate_system.py`：

```text
CX = -CFx
CY =  CFy
CZ = -CFz
Cl = -CMx = CMl
Cm =  CMy = CMm
Cn = -CMz = CMn
```

因此 production rate 使用 OpenVSP 已写出的标准 `CFy/CL/CMl/CMm/CMn` stability 字段，不再二次翻转。单位分别为 `1/p_hat`、`1/q_hat`、`1/r_hat`，并把原字段名保存在 `source_field`。

## 3. Centered FD 与 TRIM

alpha、beta、elevator、aileron、rudder 使用相同的公共扰动与 degree-to-radian 实现。每个变量按配置计算多个真实正负 case；同一变量的 solver pair 可被多个系数复用，但每个导数独立选择稳定步长。没有 PASS 稳定区但在 WARN 门限内时为 `WARN_NUMERICAL`；缺失、非有限或超过 WARN 为 `FAIL`。

TRIM 求解：

```text
qbar * Sref * CL - mass*g = 0
qbar * Sref * cref * Cm = 0
```

先以低成本 Wake pre-trim，再查询完整 derivative bundle 的最大生产 Wake并重新配平。Jacobian 使用真实 alpha/elevator centered FD；Newton 步受界限约束并按 `1, 0.5, 0.25, 0.125` 回溯。15 次仅是停止上限，必须同时满足 ±1 N 和 ±1 N·m 才成功。

## 4. 三层 Gate

`production_numerical_settings.yaml` 明确保存：

1. `solver_gate`：model/OpenVSP/config identity、基础六系数 Wake convergence、离散 Wake schedule、与基础 GRID 数值直接相关的 boundary continuity。
2. `derivative_gate`：derivative Wake/FD step convergence、23 项 source/method/unit/finite 完整性及导数数值检查。
3. `production_gate`：以上两者的最坏状态，用于最终数据库和 AUTOTUNE 交付。

`adaptive_grid_eligible` 仅由 `solver_gate == PASS` 决定。这让 `solver PASS + derivative WARN` 可以运行 Adaptive GRID，同时最终 dataset 仍诚实保留 WARN。Solver Gate 为 WARN/FAIL 时 Adaptive GRID 被拒绝，且 `--force` 无效。Uniform GRID/TRIM 保留 PASS/WARN policy；production FAIL 只有显式审计性 `--force` 可运行，但不会改变真实状态。

Gate 不包含 Tessellation convergence。状态体系为 `PASS/WARN_NUMERICAL/FAIL`；不存在为已支持方法保留的永久限制状态。

## 5. Wake 与缓存

正式 Wake 候选为 3/5/8/12。只有从某等级到所有后续等级都 PASS 才选择较低值；8→12 不通过时才运行 verification-only Wake16。12→16 PASS 也只验证 Wake12，不把16加入 production schedule。

Numerical cache signature 包含模型哈希、OpenVSP/算法版本、状态、舵偏、Wake、分析类型和厚体选择。只有完整成功的真实 case 才命中；旧配置仅改变 validation/manifest bookkeeping 而物理求解请求完全等价时，可复用其 solver payload。正式 GRID/TRIM 还签入 production settings identity。

Adaptive GRID 的 evaluator 始终走统一 `_grid_case`，所以 seed、corner、midpoint 均先查持久 cache；同一次 refinement 中几何坐标相同的点再由内存 key 去重。报告分别记录 persistent cache hits、deduplicated reuses 和 new solver runs。

## 6. Adaptive GRID 算法

### 6.1 Seed 与插值

`operating_conditions.speed/alpha/beta` 定义 seed axes。长度大于1的轴为 active；其余轴固定，算法自然退化为 1D/2D。每个初始 cell：

1. 获取全部真实 corner；
2. 在几何中心计算真实 VSPAERO case；
3. 由 corner 在中心做 linear/bilinear/trilinear 插值；
4. 对 `grid.adaptive.quantities` 逐项使用独立绝对+相对 tolerance；
5. PASS 接受，WARN/FAIL 局部 refine。

中心恰好是每个 active 轴的 0.5 位置，所以 multilinear 中心预测等于所有去重 corner 的等权平均。

### 6.2 确定性局部二分

只选择一个 active axis：

```text
normalized_span = current_cell_span / whole_seed_domain_span
```

取最大者；相同按 V、alpha、beta。生成两个 child，继续独立中心检查，不会一次把3D cell切成8份。队列、cell ID、排序和 tie-break 固定，因此相同输入与 solver cache 得到相同 refinement history。

### 6.3 停止状态

- 所有最终 cell PASS：整体 PASS；
- `max_depth/min_spacing/max_cases` 阻止继续细分且遗留 WARN：整体 WARN；
- 遗留 FAIL 或 solver/插值数据失败：整体 FAIL。

`max_cases` 对全部去重真实点生效；`min_spacing` 判断二分后的半跨度；达到任何上限都不会自动成功。报告包含 active dimensions、seed/final point counts、cache/new runs、accepted/refined cells、最大深度、总体和逐系数最坏误差、终止原因、final bounds/status 与完整 history。

## 7. 配置新飞机

在 OpenVSP 中先完成几何、参考量、重心、三组 Control Surface Group、薄面/厚体 Set 和用户认可的 Tessellation。然后修改：

- `aircraft.model`、`geometry_sets`、`reference`；
- `controls` 的组名、中立位和限位；
- `trim.mass_kg`、alpha/elevator 范围和工况；
- `operating_conditions` 作为 uniform axes 或 adaptive seed；
- `grid.adaptive` 的量、容差和资源上限；
- Wake representative states 与 FD candidates。

更换模型或网格会改变模型哈希，旧 cache 不会误命中。不要通过删导数、改符号或放宽门限消除真实 WARN/FAIL。

## 8. 输出结构

### 8.1 JSON

`results/latest/aero_database.json` 包含：

```text
grid.mode
grid.results                 # 所有真实 seed + adaptive midpoint
grid.adaptive_summary
grid.cells
grid.refinement_history
trim.results[].derivatives.production_derivatives
trim.results[].derivatives.unsteady_derivative_diagnostics
trim.results[].derivatives.native_derivative_diagnostics
trim.results[].derivatives.required_derivatives_manifest
```

### 8.2 CSV

`aero_database.csv` 一行一个真实 solver point，`grid_source` 标记 `seed/adaptive_midpoint`；TRIM 列以 `production_<name>` 读取统一23项集合。`trim_derivatives.csv` 提供逐项 value/source/source_field/method/unit/wake/status。

Adaptive 独立报告位于 `results/adaptive_grid/`：JSON 保存汇总、cells 与 history；points/cells CSV 方便检查不规则点集。

### 8.3 MATLAB

`results/autotune/aircraft_aero.mat` 的 `AERO.longitudinal/lateral/controls` 从 `production_derivatives` 建立，故包含 `CL_q, Cm_q, CY_p, Cl_p, Cn_p, CY_r, Cl_r, Cn_r`。`AERO.grid` 保存不规则点的 V/alpha/beta、`grid_source` 和六系数，不强制规则矩阵。native 与 unsteady diagnostic 不会覆盖 production。

## 9. 主要代码职责

| 文件 | 作用 |
|---|---|
| `src/main.py` | CLI 与 workflow orchestration |
| `src/vspaero_runner.py` | steady/P/Q/R analysis 设置、执行和原始文件完整性 |
| `src/coordinate_system.py` | 唯一坐标、符号、单位与 steady/unsteady 字段映射 |
| `src/finite_difference.py` | centered FD、rate production 合并与 manifest |
| `src/numerical_convergence.py` | Wake/FD 验证、三层 Gate、cache 与报告 |
| `src/adaptive_grid.py` | 1D/2D/3D midpoint 插值、局部二分、去重和报告 |
| `src/validation.py` | TRIM、23项来源/数值/物理完整性检查 |
| `src/export_results.py` | JSON/CSV/MAT 导出及 MAT 回读检查 |
| `src/trim_solver.py` | 有界 centered-FD Newton TRIM 与回溯 |

## 10. 排查顺序

1. 运行 `run_aero.bat check`。
2. 查看 numerical report 中 solver/derivative/production 三层状态。
3. Solver FAIL 按 signature 查看 `raw/<signature>/case_result.json` 与 `vspaero_console.txt`。
4. Derivative WARN/FAIL 检查对应 source field、unit、FD samples 和 selected step。
5. Adaptive 非 PASS 查看 final cell 的 bounds、逐系数误差和 termination reason。
6. 修改模型/网格后重新运行 numerical-convergence，避免 identity 不匹配。

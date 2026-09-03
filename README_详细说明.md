# OpenVSP/VSPAERO 气动数据库工作流：详细说明

## 1. 目标和责任边界

本项目从 OpenVSP `.vsp3` 模型生成 GRID 气动数据、纵向 TRIM 工况、经数值稳定性检查的 centered finite-difference derivatives，并导出 JSON、CSV 和 MATLAB `.mat`。

工作流认证的是 Wake Iteration、FD 步长稳定性、TRIM、数据完整性和 required derivative 方法状态。它不认证 OpenVSP Tessellation 或网格质量。

> Tessellation / mesh quality is user responsibility and is not numerically certified by this workflow.

用户必须在 OpenVSP 建模阶段设置足够的 Tess_U/Tess_W、截面离散、前后缘聚类和舵面铰链分辨率。脚本始终使用当前保存在 `.vsp3` 中的网格，不修改网格参数、不比较 COARSE/MEDIUM/FINE、不把网格收敛放入 Production Gate。

仍保留的网格相关有效性检查只有：VSPAEROComputeGeometry 能成功、VSPAEROSweep 能完成、`.vspgeom/.vspaero/.history/.adb` 等必要文件存在，以及所需系数为有限数。这些检查不等于网格独立性证明。

## 2. 整体流程

```text
读取配置与 .vsp3
  -> 验证几何集、参考量、舵面映射和当前模型网格可求解性
  -> 解析 Wake 代表状态（必要时真实 Pre-Trim）
  -> 在代表点选择逐导数 FD step
  -> Wake 3/5/8/12 的六系数 + required FD derivative Gate
  -> 仅对 8->12 未 PASS 的代表点运行 Wake=16 verification
  -> 离散 Wake Schedule 和边界连续性检查
  -> Required Derivatives Manifest + Production Gate
  -> GRID / 两阶段 TRIM / Production centered FD derivatives
  -> 验证、CSV/JSON/MAT 导出
```

Numerical Validation 只在少量代表状态上运行。FD step 诊断不扩展到全 GRID。逐 case signature cache 使已成功的真实求解可以断点续跑。

## 3. 配置结构

`config/aircraft.yaml` 是主配置，`config/required_derivatives.yaml` 是唯一 required derivative manifest。

### 3.1 飞机与几何

- `aircraft.model`：`.vsp3` 文件。
- `geometry_sets.thin`：主翼、平尾、垂尾等 VLM 薄面。
- `geometry_sets.thick`：机身等厚体。
- `thin_set_index/thick_set_index`：写入临时 case 模型的 OpenVSP Set。
- `reference.source/cg_source`：决定参考量和重心从模型还是 YAML 读取。

### 3.2 状态、舵面与 TRIM

- `operating_conditions`：GRID 的 V/alpha/beta。
- `controls`：OpenVSP Control Surface Group 名、中立值和限位。
- `trim.mass_kg/gravity_m_s2`：用于 Lift=Weight。
- `trim.alpha/elevator`：初值和搜索界限。
- `trim.force_tolerance_n=1` 和 `moment_tolerance_nm=1`：必须同时满足。
- `trim.max_iterations=15`：仅是停止上限，不是收敛判据。

### 3.3 Wake

- `wake.candidates=[3,5,8,12]`：唯一正式候选。
- `wake.verification_only_level=16`：只用于 12→16 外推验证。
- `representative_states`：应覆盖线性低迎角、巡航 Trim、中迎角、较高迎角和必要 beta。
- `tolerance`：六系数和 required FD derivatives 共用的绝对+相对判据。
- `neighbor_count/boundary_buffer_normalized/safety_margin_levels_for_untested`：非实测点的保守离散查询。

### 3.4 FD step

`derivatives.fd_step_candidates_deg` 对 alpha、beta、elevator、aileron、rudder 分别配置候选。当前默认为：

- alpha/beta：0.25°、0.5°、1°；
- elevator/aileron/rudder：0.5°、1°、2°。

`trim_jacobian_steps_deg` 是 TRIM 专用步长，保持 alpha=0.5°、elevator=2°。它与生产导数的逐导数自动选择目的不同。

## 4. TRIM 和 FD Jacobian

TRIM 解两个方程：

```text
qbar * Sref * CL - mass * g = 0
qbar * Sref * cref * Cm = 0
```

每轮在当前 alpha/elevator 周围运行真实正负扰动，调用 `finite_difference.py` 的统一 degree-to-radian 中心差分，构造 CL/Cm 对 alpha/elevator 的 Jacobian。VSPAERO native derivatives 可写入迭代历史供人工比较，但不进入 Jacobian。

Newton 步先受 `max_step_deg` 限制，再按 `1 / 0.5 / 0.25 / 0.125` 做真实回溯。选择第一个降低归一化残差的候选；所有候选均不改善时保留原状态并 FAIL。

两阶段流程是：

1. Pre-Trim 用配置的低成本 Wake 找到近似 alpha/elevator。
2. 查询 Pre-Trim 基点和完整导数 bundle 的 Wake，以最大值作为固定 production Wake，从近似解重新 TRIM。
3. 正式 TRIM 后再查询 bundle；如果进入更高区域，只在完整 TRIM 之间升级 Wake 并重新配平。

Bundle 中 base、alpha±、beta±、三组舵面±全部使用同一 production Wake，避免差分两侧使用不同数值设置。

## 5. Wake 收敛与 Wake=16 verification-only

每个代表状态在 3/5/8/12 运行：

- 六个基础系数 `CL, CD, CY, Cl, Cm, Cn`；
- 全部 15 项正式 centered FD：alpha 的 `CL/CD/Cm`，beta 的 `CY/Cl/Cn`，以及 elevator、aileron、rudder 各自对应的三项 required control derivatives。

所有后续相邻转换均 PASS 时，才可选择较低 Wake。Native derivatives 只在代表基准 stability case 中保存供诊断，不扩展成 Wake 等级研究，也不参与正式 Wake Gate。

如果最后的 8→12 不是 PASS，工作流仅对该 representative state 运行 Wake=16：

- 12→16 PASS：该状态判为已验证，`required_wake=12`；
- 12→16 仍不 PASS：保持 WARN，`required_wake=12`；
- 16 输出缺失或非有限：这是真实数据缺失，状态 FAIL；
- 8→12 已 PASS 的状态不运行 16。

Wake=16 不在 `wake_schedule.candidates`中，不在 GRID/TRIM 查询中返回，不会把 production Wake 提高到 16。

## 6. Wake Schedule 和边界连续性

Schedule 保存每个实测 `(V,alpha,beta,required_wake,status)`。实测点直接使用该点 Wake；其他点使用归一化 V/alpha/beta 距离的有限邻域，在边界缓冲区取较高离散等级，并按配置增加安全等级。没有 alpha 阈值硬编码，也不对 Wake 做线性插值。

当相邻代表区域 Wake 不同时，在中点比较低/高 Wake 的同一套六系数和 required FD derivatives。如果不连续，低 Wake 端点升级到高 Wake，并保留 WARN 审计信息。

## 7. FD step 自动选择

所有角度和舵面导数共用 `finite_difference.py` 的公共扰动、限位、单位换算和中心差分功能。同一变量的正负 case 可被多个系数导数共享，但每个导数独立判断稳定区和选择步长。

选择逻辑为：

1. 按步长从小到大比较相邻导数，使用配置中不变的绝对+相对容差。
2. 找到第一个 PASS 稳定区，选择该区的较大端点，避免使用最小、最易受求解器噪声影响的步长。
3. 无 PASS 但有 WARN 稳定区时，状态为 `WARN_NUMERICAL`。
4. 没有任何相邻区达到 WARN 时为 `FAIL`。
5. Numerical Validation 的推荐步长在正式 TRIM 点仍要通过本地候选步长稳定性检查；如果局部行为不同，可独立改选。

因此 `CY_delta_r` 可自动选择约 1°的局部线性步长，不受 2°/4° 较大偏角非线性强制。诊断仅在必要代表点上执行，不将多步长扩展到 GRID。

每个正式 FD record 至少包含 `selected_fd_step`、`convergence_status`、`derivative_value`、`method`、完整正负 samples、Wake 和坐标/符号约定。

## 8. Production FD 与 Native diagnostic

输出严格分为：

- `production_fd_derivatives`：真实正负 centered FD，使用选定步长，经数值稳定性检查；
- `native_derivative_diagnostics`：VSPAERO stability/control native 值，`diagnostic_only=true`，不进入 Production Gate，不作 TRIM Jacobian，不覆盖正式 FD；
- `required_derivatives_manifest`：统一的 required 逐项状态和 Gate 处理。

GRID 的 stability sweep 仍保存 native 值供诊断，但 CSV 字段使用 `native_diagnostic_` 前缀。TRIM 正式数据字段使用 `production_fd_` 前缀。MAT 中 production 结构只放正式 FD，原生值放在独立 `native_derivative_diagnostics`。

## 9. Required Derivatives Manifest

`config/required_derivatives.yaml` 定义 23 项 required 导数。每个输出 item 至少包含：

- `name`、`required`、`value`；
- `source`、`method`、`selected_fd_step`、`units`；
- `coordinate_sign_convention`、`wake_level`；
- `validation_status`、`production_included`、`gate_action`、`reason`。

状态只有：

- `PASS`：正式 centered FD 通过；
- `WARN_NUMERICAL`：有可接受但需审阅的步长敏感性；
- `METHOD_LIMITATION`：所需方法在当前 API 不可实现；
- `FAIL`：真正缺失、非有限或数值/方法不可接受。

Manifest 顶层 `status_policy` 显式定义 Gate action，当前 `PASS=ACCEPT`、`WARN_NUMERICAL/METHOD_LIMITATION=ACCEPT_WITH_WARNING`、`FAIL=REJECT`。这些规则不在代码中隐藏绕过。

OpenVSP/VSPAERO 3.51.3 公开 Sweep API 不能提供真实负 steady p/q/r，因此不能构造标准中心差分。`CL_q, Cm_q, CY_p, Cl_p, Cn_p, CY_r, Cl_r, Cn_r` 按 manifest 的 `steady_rate_centered_difference` 规则标为 `METHOD_LIMITATION`，`production_included=false`。VSPAERO native 值可作 reference，但不被冒充为 centered FD。

汇总固定报告 `Required / PASS / WARN_NUMERICAL / METHOD_LIMITATION / FAIL / production_included`。Production Gate 只因 manifest 规则中的 `REJECT`、缺失求解数据或其他真实失败而 FAIL。

## 10. Cache / Resume

Numerical Validation 缓存位于 `results/numerical_convergence/raw/<signature>/`。Signature 包含：

- `.vsp3` 哈希（因此包含用户当前网格身份）；
- OpenVSP 版本和验证算法版本；
- V/alpha/beta 及所有舵偏；
- Wake，包括 verification-only 16；
- polar/stability 分析类型和厚体参与。

诊断用途名不影响求解器输出，因此条件完全相同的 Wake Gate/FD step case 共享缓存。只有 `status=SUCCESS`且必要系数/诊断结构完整的 case 才可命中；损坏、失败或不完整 case 重算。启动 Numerical Validation 不删除整个 raw。

正式 GRID/TRIM 也使用签名恢复已完成 case；签名覆盖模型、配置、manifest、Wake Schedule 和 selected FD steps。

## 11. Production Gate

Gate 组合：

- Wake 六系数收敛；
- required centered-FD Wake derivatives 收敛；
- 必要的 Wake=16 验证；
- Wake Schedule 边界连续性；
- FD step 稳定性与 required manifest status policy。

Gate 不包含 Tessellation convergence，native derivatives 不参与 Gate。

`PASS` 允许正式运行；`WARN` 按 manifest 中显式的带警告接受规则允许普通 GRID/TRIM；`FAIL` 默认阻止。Adaptive GRID 要求 Gate PASS。`--force` 只许录审计用试跑，不修改真实 Gate 状态。

## 12. 输出结构

### 12.1 Numerical Validation

- `numerical_convergence_report.md/.json`：Wake/Wake16、FD step、manifest、Gate、cache 汇总。
- `production_numerical_settings.yaml`：模型身份、网格责任声明、Wake Schedule、selected FD steps、manifest 和 Gate。
- `wake_convergence_map.csv/.png`：代表点 production Wake 及 Wake16 状态。
- `fd_step_selection.csv`、`fd_step_convergence.png`：逐导数步长选择。
- `required_derivatives_manifest.csv`：逐项状态。
- `raw/<signature>/case_result.json`：缓存请求、映射输出和运行时间。

### 12.2 正式数据库

- `results/latest/aero_database.json`：保留完整嵌套数据。
- `results/latest/aero_database.csv`：GRID/TRIM 平铺表，使用 `production_fd_` 和 `native_diagnostic_` 前缀。
- `results/latest/trim_derivatives.csv`：每项 required 导数的 value/source/method/step/status/wake/production flag。
- `results/validation/validation_report.csv`：Solver/TRIM/Numerical/Derivative/Physics/Dataset 多层检查。
- `results/autotune/aircraft_aero.mat`：`AERO.longitudinal/lateral/controls` 只包含正式 FD；`AERO.native_derivative_diagnostics` 为独立参考区。

JSON 中 `derivatives.production_fd_derivatives` 与 `derivatives.native_derivative_diagnostics` 永不共用同一个模糊 `value` 字段：前者是 `derivative_value`，后者是 `diagnostic_value`。Manifest 中的 `value` 必须结合 `source` 和 `production_included` 解读。

## 13. 主要代码文件

| 文件 | 作用 |
|---|---|
| `src/main.py` | CLI、启动验证、GRID/TRIM 编排、Gate 和导出 |
| `src/config_loader.py` | YAML/manifest 读取和强约束校验 |
| `src/openvsp_interface.py` | 模型、几何集、参考量和舵面接口；不修改网格 |
| `src/vspaero_runner.py` | 使用当前 `.vsp3` 网格生成并验证单个 VSPAERO case |
| `src/numerical_convergence.py` | Wake/Wake16、Schedule、边界、FD 代表点、Gate、cache 和报告 |
| `src/finite_difference.py` | 公共扰动、centered FD、步长选择、production/native 分离和 manifest 组装 |
| `src/trim_solver.py` | 15 次上限、真实 FD Jacobian、有界 Newton 和回溯 |
| `src/coordinate_system.py` | 坐标、力/力矩符号、角度和速率导数单位的唯一映射 |
| `src/validation.py` | 输出完整性、TRIM、manifest、数值和物理验证 |
| `src/export_results.py` | JSON/CSV/MAT 结构化导出和回读校验 |
| `src/case_generator.py` | GRID/TRIM case ID、范围展开和完成 case 恢复 |
| `src/regression.py` | 固定飞机回归基线比较 |

## 14. 失败排查

1. 先运行 `run_aero.bat check`。
2. 查看 `numerical_convergence_report.md` 的 Wake16、FD step 和 manifest 汇总。
3. 对真实 solver FAIL，按 signature 查看 `raw/<signature>/case_result.json` 和 `vspaero_console.txt`。
4. `WARN_NUMERICAL` 不应通过放宽容差直接消除；先检查步长区间、模型网格、几何和舵面。
5. `METHOD_LIMITATION` 要按 manifest 策略审阅，不应改名为 PASS 或用 native 值覆盖 production。
6. 更换模型或修改 OpenVSP 网格后，模型哈希改变，旧 case 不会误命中。

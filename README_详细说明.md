# OpenVSP/VSPAERO 气动数据库工作流：详细说明

## 1. 目标数据模型

本工程面向 ArduPilot SITL -> 飞控/PID/AutoTune -> 舵机 -> Simulink 非线性 6DOF 闭环。当前气动模型为：

```text
C(V,alpha,beta,p,q,r,delta_a,delta_e,delta_r)
  ~= C0(V,alpha,beta)
   + delta_Cp + delta_Cq + delta_Cr
   + delta_Caileron + delta_Celevator + delta_Crudder
```

`C` 表示 `CL, CD, CY, Cl, Cm, Cn`。`additive_response_assumption = true` 在 JSON 和 MAT metadata 中明示保存。当前忽略 `q*elevator`、`p*aileron`、`elevator*aileron`、`q*r` 等交叉项；不建立九维完整笛卡尔网格。以后只为有证据的强耦合添加二维修正。

## 2. GRID 与 TRIM 的物理关系

GRID 是飞行包线中的一般气动状态。基准设定为 `p=q=r=0`，三舵面均在 neutral。它不需要满足力和力矩平衡，所以 `Cm != 0`、`Cl != 0`、`Cn != 0` 都合法。局部气动导数和局部响应的定义也不要求 baseline 是 TRIM 点。

TRIM 是满足指定平衡条件的特殊 operating-point 子集。纵向平飞 TRIM 求解：

```text
qbar*Sref*CL - mass*g = 0
qbar*Sref*cref*Cm = 0
```

它使用有界 Newton、真实 alpha/elevator centered-difference Jacobian、回溯和 Wake upgrade。TRIM 仍用于初始状态、模型健康检查、局部线性化和 AutoTune 辅助分析，但不是 GRID 的前置。

## 3. 唯一模式选择逻辑

```yaml
analysis:
  grid_enabled: true
  trim_enabled: false
```

`analysis.modes` 和 `trim.enabled` 不再是可用配置，避免真值来源冲突。`all` 严格按上述开关；`grid` 和 `trim` 显式只运行对应模块。`all` 且两开关都关闭时，在启动求解器前报错。

## 4. 基础 GRID：uniform 与 adaptive

Uniform GRID 计算 `operating_conditions` 的规则 `(V,alpha,beta)` 笛卡尔积。Adaptive GRID 从同一 seed axes 开始，用 corner 插值的中心预测与真实 VSPAERO 中点对比；非线性超限时按归一化跨度只分裂一个轴。

Adaptive 最终数据是不规则点集，`grid_source` 为 `seed` 或 `adaptive_midpoint`。Response `full_grid` 直接覆盖这些实际点，不补成规则矩阵。

## 5. 控制面非线性响应

`response_scan.control_levels_deg` 是各控制面的绝对 DeflectionAngle，正向表示增加 OpenVSP Control Surface Group 设置；真实后缘方向由 `.vsp3` 的 gain 和铰链定义。

对每个选定 baseline，其他舵面固定 neutral，只改变一个舵面并运行真实 VSPAERO polar。每个 sample 保存六系数绝对值以及相对中立 baseline 的 `delta_CL ... delta_Cn`。中立点复用基础 GRID steady baseline，不重复求解。即使线性审计推荐 derivative，原始 response samples 也不删除。

## 6. 线性 / 非线性审计

对每个 base point、control variable 和 coefficient 组合计算：

- neutral 两侧最近样本的 centered slope（每 rad）；
- 全范围最小二乘一阶拟合 slope/intercept；
- 最大绝对残差与 normalized maximum deviation；
- 相邻区间 slope variation；
- `R²`；
- 正负等幅 sample 相对 baseline 的不对称度。

阈值全部位于 `response_scan.linearity`。分类为 `LINEAR`、`WEAKLY_NONLINEAR`、`NONLINEAR`或 `INSUFFICIENT_DATA`。只有 `LINEAR` 推荐 `derivative`，其他推荐 `response_table`。分类仅是建议，不自动改 production schema。

## 7. p/q/r 的实现和限制

OpenVSP 3.51.3 steady `VSPAERO_Stab` 已验证提供经典无量纲 rate derivatives：

```text
p_hat = p*bref/(2V)
q_hat = q*cref/(2V)
r_hat = r*bref/(2V)
```

每个基础 GRID case 已运行 steady stability，response 层直接将六系数 `C_p/C_q/C_r` 整理为随 `(V,alpha,beta)` 变化的局部导数场，`representation = local_linear_derivative`，不增加 rate solver cases。

P/Q/R damping analysis 是不同的时间相关诊断；Q/R 可输出 `q+alpha_dot`和 `r-beta_dot`，不能拆成独立 q/alpha_dot 或 r/beta_dot。当前 runner 没有已验证的任意独立 body-rate 输入，因此绝不生成伪 rate curves。

## 8. scope: audit / full_grid

`audit` 使用少量代表 GRID 点。用户可配置 `representative_states`；留空则从成功 GRID 点确定性选低 V/低 alpha、中 V/中 alpha、高 V/中 alpha、高 alpha 和最大 `|beta|` 附近点，并去重。所有点均不依赖 TRIM。

`full_grid` 使用当次全部成功基础 GRID 点。求解量约为 `base_points * sum(non-neutral control levels)`，应先 audit 再升级。

## 9. Gate 职责

1. Solver / GRID Gate：基础六系数 Wake 收敛、离散 Wake schedule 和 boundary continuity。Uniform 接受 PASS/WARN；Adaptive 必须 PASS。
2. Response Gate：控制面 samples 数量/求解完整性，以及 p/q/r 六系数 steady rate fields 完整性。响应非线性本身不是失败。
3. Derivative Gate：Wake/FD step 稳定性、23 项 source/method/unit/finite 完整性，继续为 TRIM 局部导数包服务。
4. TRIM validation：仅当 TRIM 运行时检查收敛、残差、边界、Jacobian/derivatives 和物理合理性。
5. Simulink delivery：GRID+response 模型不因 TRIM 未运行而失败；选择 TRIM 交付时，完整 TRIM derivative package 仍必须通过。

Gate 门限未因 GRID-only 而放宽，也没有关闭旧检查。

## 10. 缓存与 resume

基础 GRID 仍用现有 case cache。Response signature 包含 model hash/OpenVSP 版本、base GRID signature、`(V,alpha,beta)`、response enabled/scope/variables/全部 levels、variable/value/unit、neutral/实际三舵偏、Wake、reference、rate source、几何集和坐标约定。改变 scan levels 使 response cache 失效，但不会使未变的基础 GRID cache 失效。中立点和共用 baseline 不重复求解。

## 11. 坐标和符号

内部体轴为 `+X forward, +Y right, +Z down`，右手系；OpenVSP 字段为 `+X aft, +Y right, +Z up`。唯一映射位于 `src/coordinate_system.py`：

```text
CX=-CFx, CY=CFy, CZ=-CFz
Cl=-CMx=CMl, Cm=CMy=CMm, Cn=-CMz=CMn
```

状态角和舵偏 samples 使用 deg，角度导数使用 `1/rad`，rate 导数使用 `1/p_hat`、`1/q_hat`、`1/r_hat`。

## 12. JSON / CSV / MAT schema

JSON 核心路径：

```text
metadata.analysis_selection / additive_response_assumption
grid.results / adaptive_summary / cells / refinement_history
responses.controls.elevator|aileron|rudder
responses.rates.p|q|r
responses.linearity / gate / cache / assumptions
trim.results[].derivatives.production_derivatives
validation / summary
```

CSV 分层：`aero_database.csv` 仅基础 GRID，`grid_response_samples.csv` 仅控制面 samples，`grid_response_summary.csv` 保存线性审计和 rate derivatives，`trim_database.csv` 仅 TRIM，`trim_derivatives.csv` 仅 TRIM 23 项导数。

MATLAB `AERO` schema 2.0：

```text
AERO.meta / reference / grid
AERO.responses.controls.elevator|aileron|rudder
AERO.responses.rates.p|q|r
AERO.linearity / assumptions
```

Control arrays 包含 base ID、V/alpha/beta、绝对 deflection、neutral offset、六系数绝对值和增量。Rate arrays 包含 base ID、状态、normalization、source、representation 和六系数局部导数。TRIM 通过时，兼容保留 `flight_points/trim/longitudinal/lateral/controls/native_derivative_diagnostics`。

## 13. 代码职责

| 文件 | 职责 |
|---|---|
| `src/main.py` | CLI、模式解析和流程编排 |
| `src/response_scan.py` | response cases、baseline 选择/复用、增量、线性审计和 rate fields |
| `src/vspaero_runner.py` | 唯一 VSPAERO runner |
| `src/coordinate_system.py` | 唯一坐标、符号和字段映射 |
| `src/adaptive_grid.py` | Adaptive GRID 中点验证和细分 |
| `src/finite_difference.py` | centered FD 和 TRIM production derivatives |
| `src/numerical_convergence.py` | Wake/FD、schedule 和旧三层 Gate |
| `src/validation.py` | TRIM/导数/物理/portability 检查 |
| `src/export_results.py` | 分层 CSV/JSON/MAT 与 MAT 回读验证 |

Response 模块不复制 runner、Wake schedule、坐标映射、centered FD 或缓存系统。

## 14. 已知限制和扩展

- 控制面响应为独立一维表，高阶交叉项未建模。
- p/q/r 是局部线性导数场，不是任意 rate response table。
- Linearity audit 不自动丢弃 samples，也不自动将 production 永久压缩成导数。
- Tessellation/mesh 由 `.vsp3` 管理，脚本不覆盖 Tess_U/Tess_W，Gate 不宣称 mesh independence。
- 从 audit 扩展到 full_grid 前，应根据曲率、不对称和飞行范围调整 levels；然后设置 `response_scan.scope: full_grid` 并运行 `run_aero.bat grid` 或 `run_aero.bat all`。

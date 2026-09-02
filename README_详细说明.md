# OpenVSP/VSPAERO 气动数据工具——详细说明

## 1. 工程目标与设计边界

本工程把 OpenVSP/VSPAERO 的几何与求解能力封装成一条可重复的数据生产链：读取 `.vsp3`，生成 GRID 数据；在指定速度求纵向配平；围绕配平点计算完整稳定/控制导数；执行数值、物理和数据集验证；最终输出版本化的 MATLAB `AERO` 接口。CSV 面向人工审查，MAT 面向调参模型。

本实现只维护一条调用链、一个坐标转换模块、一个有限差分引擎、一个 required derivative 清单和一个入口。当前交付配置是小规模 smoke test，不是高精度 CFD 或生产级网格收敛证明。

## 2. 架构与数据流

```text
aircraft.yaml + required_derivatives.yaml + .vsp3
                 │
                 ▼
       配置/环境/几何/控制面校验
                 │
          ┌──────┴──────┐
          ▼             ▼
     GRID Sweep      TRIM Newton
          │             │
          │       TRIM 点三尺度扰动
          │             │
          └──────┬──────┘
                 ▼
       坐标、符号、单位统一
                 ▼
       六级 PASS/WARN/FAIL
                 ▼
    JSON + CSV + AERO schema MAT
```

每个 case 使用输入、模型哈希、OpenVSP 版本、几何/求解/控制/导数/验证配置生成稳定签名。签名一致且结果为可用状态时断点续算；任何相关输入变化都会重新计算。

## 3. 文件夹结构

```text
E:/aerodynamic/
├─ aircraft/                     .vsp3 飞机模型
├─ config/
│  ├─ aircraft.yaml              唯一主配置
│  ├─ openvsp.yaml               OpenVSP 路径与期望版本
│  └─ required_derivatives.yaml   唯一导数清单
├─ src/                           核心 Python 模块
├─ tests/
│  ├─ test_core.py               配置、坐标、数值与交付结果测试
│  └─ regression/
│     ├─ regression.yaml          固定飞机/求解/扰动配置
│     └─ baseline.json            固定飞机真实结果基线
├─ results/
│  ├─ cases/                      case 结果和原始求解器文件
│  ├─ latest/                     人工查看的最新汇总
│  ├─ validation/                 验证报告
│  ├─ regression/                 回归比较
│  └─ autotune/aircraft_aero.mat  下游标准接口
├─ run.py                         唯一 Python 入口
├─ setup.bat                      首次环境安装
└─ run_aero.bat                   Windows 一键 all
```

## 4. 主要模块

| 模块 | 职责 |
|---|---|
| `run.py` | 将 `src` 加入路径并进入统一 CLI |
| `src/main.py` | 编排 check/grid/trim/all/regression，组织缓存、验证与导出 |
| `src/config_loader.py` | 读取并严格校验配置和 manifest；解析相对路径 |
| `src/case_generator.py` | 展开 V/alpha/beta、生成安全 case id 与缓存签名 |
| `src/openvsp_interface.py` | 加载 OpenVSP API、模型、geometry sets、参考量和控制面组 |
| `src/vspaero_runner.py` | 配置并运行同一套 VSPAERO 分析，解析真实输出 |
| `src/trim_solver.py` | 受范围约束的纵向 Newton 配平 |
| `src/finite_difference.py` | 唯一三尺度导数引擎与收敛度量 |
| `src/coordinate_system.py` | 唯一坐标、符号、单位、字段与无量纲速率定义 |
| `src/validation.py` | SOLVER 至 DATASET 的分级验证与完整性汇总 |
| `src/export_results.py` | 一次生成 JSON、CSV、validation 与 `AERO` MAT |
| `src/regression.py` | 固定基线的绝对+相对容差比较 |

## 5. 配置结构与优先级

`config/aircraft.yaml` 集中飞机、工况、大气、几何集合、求解器、参考量、控制面、配平、扰动、验证、回归、缓存和导出参数。文件路径相对各自 YAML 解析，不依赖启动目录。用户参数及单位表见快速说明。

参考量有两种来源：

- `reference.source: model`：从 `.vsp3` 的 VSPAEROSettings 读取 Sref、bref、cref。
- `reference.source: config`：使用配置中的 `sref_m2/bref_m/cref_m`。

CG 同理由 `cg_source` 单独选择。程序会拒绝非正的参考面积/长度和非有限输入。

`config/openvsp.yaml` 的显式路径优先；为 `null` 时再检查 `OPENVSP_ROOT` 和常见安装位置。版本不匹配会在启动检查中给出清楚错误。

## 6. OpenVSP/VSPAERO 调用流程

启动时依次完成：导入与当前 Python 匹配的 OpenVSP binding、验证 VSPAERO 可执行文件、读取模型、列出 geometry、验证 thin/thick 选择、验证三个 Control Surface Group、读取参考量、应用 tessellation。每次分析复用同一个 runner，写入独立 raw 子目录并解析 OpenVSP Result 数据；正常终端只显示每个 case 的摘要，原始 solver 输出保存在 case 目录。

GRID 和 TRIM 均使用 VSPAERO Sweep/稳定性分析已有链路。导数中心差分中的 alpha/beta/control 样本使用相同几何、求解器和输出映射，避免平行实现。

## 7. Thin surface 与 thick body

机翼、平尾和垂尾放入 `thin` 集合，机身放入 `thick` 集合。runner 同时把 `ThinGeomSet` 和 `GeomSet` 传给 VSPAERO，使升力面和厚体共同参与名义分析。

数据集还运行一次相同状态的 thin-only 比较；只要 thin+thick 与 thin-only 的有限系数变化超过配置的极小阈值，即证明机身确实进入求解。结果写入 `results/validation/fuselage_effect_validation.csv`。这项检查只证明参与，不证明机身网格已经收敛。

## 8. 控制面与正方向

模型必须存在恰好可识别的 Aileron、Elevator、Rudder 三个 Control Surface Group，并含活动表面。程序记录组内表面名和 gain。配置正偏角的唯一含义是：增加该组的 OpenVSP `DeflectionAngle`。几何上的后缘运动方向由模型中的 gain、表面朝向和铰链方向共同决定，因此迁移飞机时必须通过小偏角结果核对物理方向，不能仅凭“正升降舵”等自然语言猜测。

所有控制导数分母以 rad 计，虽然配置和 OpenVSP 偏角输入以 deg 计。例如 `Cm_delta_e = dCm/d(delta_e_rad)`。

## 9. GRID 模式

`grid` 将 `operating_conditions` 中 V × alpha × beta 做笛卡尔积。轴既可写 `values`，也可写含端点的 `start/step/end`，因此架构支持 `V=8:2:22 m/s`，而当前 smoke 仅为 V=[8,9]、alpha=[0,1]、beta=[0] 共 4 点。

每点运行 VSPAERO stability sweep，保留基准气动系数、原生稳定导数、控制导数和 raw 结果。GRID 用于查表数据库与链路验证；三尺度敏感性不扩展到全部 GRID 点，避免无意义地成倍增加算例。

## 10. TRIM 模式与保存字段

TRIM 是后续线性化/调参数据的主模式。对每个 `trim.operating_conditions` 速度和 beta，程序求 `alpha_trim` 和 `elevator_trim`，使：

```text
qbar * Sref * CL - mass * g = 0
qbar * Sref * cref * Cm      = 0
```

每点保存 V、alpha/beta/elevator、CL/CD/CY/Cl/Cm/Cn、力与俯仰矩残差、迭代历史、收敛标记、容限、solver 时间及错误原因。

## 11. Trim 算法

`trim_solver.py` 使用局部 VSPAERO stability/control Jacobian 构造二维 Newton 更新，并对 alpha/elevator 搜索范围和单步大小施加约束。每次迭代均为真实 VSPAERO 求解；达到力、矩两个容限才算收敛。若雅可比不可用、达到最大迭代数、越界或 solver 失败，TRIM 为 FAIL，导数集不会假装成功。

当前 smoke 点 V=8 m/s 的实跑结果是 alpha≈4.26891°、elevator≈-5.95249°，力残差约 -0.0183 N、俯仰矩残差约 0.1419 N·m，均小于 1.0 的配置容限。

## 12. 统一中心有限差分

对于状态角和控制变量，`finite_difference.py` 以当前 TRIM 点 `x0` 计算：

```text
dC/dx = [C(x0 + h) - C(x0 - h)] / (2h)
```

alpha、beta、aileron、elevator、rudder 都运行真实 plus/minus 两侧，且控制扰动会先检查配置偏角范围。每项记录 base state、base coefficient、两侧完整 state/coefficients、步长、分母、单位、导数、方法和 validation status。相同扰动变量的一次求解会同时供该变量相关的多个系数使用，不重复调用求解器。

### p/q/r 的真实接口限制

OpenVSP 3.51.3 的公开 `VSPAEROSweep` API 没有负 p/q/r 稳态单点输入。其 VSPAERO 源码内部用固定正速率差分产生 `Roll__Rate/Pitch_Rate/Yaw___Rate`，所以本工程不能在不伪造数据或维护修改版求解器的前提下提供真实 ±p/±q/±r。

本工程的处理是：保留 VSPAERO 原生真实正向 rate case，在三个有效无量纲步长执行求解，把 unavailable minus 明确保存为 null，方法名为 `vspaero_native_forward_rate`，并强制产生配置中的 `rate_derivative_method_status: WARN`。这项限制不会被伪装成中心差分；如果 `autotune_allow_warn` 未显式开启，这些点不会进入 MAT。

## 13. 0.5Δ / Δ / 2Δ 收敛检查

基础扰动全部集中在配置。每个导数计算三个值 `D0.5`、`D1`、`D2`，输出采用名义值 `D1`。度量为：

```text
max_change = max(|D0.5-D1|, |D2-D1|)
reference  = max(|D1|, near_zero_reference)
variation  = max_change / reference
```

状态判定采用“绝对 + 相对”容差：

```text
PASS limit = pass_absolute + pass_relative * reference
WARN limit = warn_absolute + warn_relative * reference
```

小于 PASS limit 为 PASS；介于二者为 WARN；超过 WARN limit 为 FAIL。`near_zero_reference` 防止近零导数的相对误差发散。报告同时保留 variation 百分比和实际 limit，便于区分数值噪声、非线性区、步长过大/过小、TRIM 质量或 solver 问题。

## 14. Required derivatives manifest

`config/required_derivatives.yaml` 是唯一清单。每项包含 name、category、required、coefficient、perturbation、unit、definition；适合常规固定翼的条目还带 expected_sign。本工程当前 23 项为：

- 纵向 5：`CL_alpha, CD_alpha, Cm_alpha, CL_q, Cm_q`
- 横航向 9：`CY_beta, Cl_beta, Cn_beta, CY_p, Cl_p, Cn_p, CY_r, Cl_r, Cn_r`
- 升降舵 3：`CL_delta_e, CD_delta_e, Cm_delta_e`
- 副翼 3：`CY_delta_a, Cl_delta_a, Cn_delta_a`
- 方向舵 3：`CY_delta_r, Cl_delta_r, Cn_delta_r`

计算后自动统计 required、calculated、missing、invalid（NaN/Inf）、validation failed 和 warned。任何 required 缺失、无效或 FAIL 都使 derivative set 为 FAIL。

## 15. 坐标系、力和力矩正方向

`coordinate_system.py` 是唯一转换权威。内部/下游采用右手体轴：+X 向前、+Y 向右、+Z 向下；Cl/Cm/Cn 分别是绕内部 +X/+Y/+Z 的右手矩。CL 向上为正，CD 向后为正。

VSPAERO 气动分量与内部字段转换为：

| 内部量 | VSPAERO 字段 | 转换 |
|---|---|---|
| CX | CFx | `CX=-CFx` |
| CY | CFy | `CY=CFy` |
| CZ | CFz | `CZ=-CFz` |
| Cl | CMx / CMl | `Cl=-CMx=CMl` |
| Cm | CMy / CMm | `Cm=CMy=CMm` |
| Cn | CMz / CMn | `Cn=-CMz=CMn` |
| CL/CD | CL/CD | 保持 VSPAERO 标准升力/阻力方向 |

OpenVSP 3.51.3 源码的稳定性输出明确写入 `CMl=-CMx`、`CMm=CMy`、`CMn=-CMz`。转换既保留 raw field/value，又保留 standard value、unit 和 conversion，避免导出时再次猜符号。

## 16. 角度、alpha 与 beta

配置、case state、OpenVSP 输入和 TRIM 输出使用 deg。任何 `dC/dalpha`、`dC/dbeta`、`dC/ddelta_*` 的有限差分分母都先转为 rad，因此单位为 `1/rad`。+alpha 与 +beta 的明确定义保存在每份数据的 coordinate metadata；不能把 deg 输入值直接当成导数分母。

## 17. p/q/r 与无量纲角速度

p、q、r 的输入参考量单位是 rad/s，但 VSPAERO 原生动态导数的分母是无量纲速率：

```text
p_hat = p * bref / (2V)
q_hat = q * cref / (2V)
r_hat = r * bref / (2V)
```

因此 `Cl_p` 等实际表示 `dCl/dp_hat`，`Cm_q` 表示 `dCm/dq_hat`，不是每 rad/s。该定义来自对 OpenVSP 3.51.3 `CalculateStabilityDerivatives` 源码分母的审计，且写入 JSON、CSV limitation 和 `AERO.meta.coordinate_system`。

三个 rate 尺度通过改变 VSPAERO 内部 Vinf 形成实际 0.5/1/2 的归一化步长，同时保持目标 flight point 的物理 speed、Mach 和 Reynolds metadata 不变；这是对内部固定正 rate delta 的显式步长检查。

## 18. 稳定导数和控制导数定义

所有导数都是对应气动力/力矩系数对 manifest 中 perturbation 的局部斜率。alpha/beta/control 是配平点中心差分、每 rad；p/q/r 是 VSPAERO 原生每 p_hat/q_hat/r_hat。输出名区分大小写：`CL` 是升力系数，`Cl` 是滚转力矩系数，`Cm` 是俯仰力矩系数，`Cn` 是偏航力矩系数。

控制导数正号依赖“增加 group DeflectionAngle”这一软件定义。若下游采用相反舵角定义，应在模型/接口边界统一改变约定，不能只对单个 CSV 列临时翻转。

## 19. 六级 validation 系统

验证不等同于“进程退出码为零”。每个 TRIM 点形成这些层级：

1. `SOLVER`：求解器完成、结果字段存在、数值有限。
2. `TRIM`：配平收敛，力/矩残差在限值内，状态和控制未越界。
3. `NUMERICAL`：三个步长完整，plus/minus 方法满足能力声明，导数有限且尺度变化过关。
4. `DERIVATIVE`：23 项 required 无缺失、无效或 failed。
5. `PHYSICS`：可配置的常规固定翼符号/大小检查与 beta=0 对称性。
6. `DATASET`：综合所有点、机身参与和路径可移植性。

报告行带 speed、level、check、status、value、limit 和 message，保存到 `results/validation/validation_report.csv`。

## 20. PASS / WARN / FAIL 传播规则

- PASS：满足 PASS 容限或检查无异常。
- WARN：完整、有限且未触发关键失败，但超出 PASS 阈值或使用已声明的受限方法。
- FAIL：solver/TRIM 失败，required 缺失/NaN/Inf/导数验证失败，或物理/数值量超过 FAIL 阈值。

组合状态取最严重者。FAIL 永不进入调参 MAT。默认接口只接受 PASS；只有配置明确设置 `validation.autotune_allow_warn: true` 才接受 WARN。当前样例显式允许 WARN，是因为 p/q/r 公共接口限制已知且所有 23 项仍完整、有限、可追溯；用户应根据自己的认证要求决定是否关闭。

物理规则只在 `conventional_fixed_wing` 和 `physics.enabled` 开启时应用；expected sign 来源于 manifest，可修改或移除，避免把所有飞机强制套用同一经验规则。对称性检查也可关闭或调阈值。

## 21. MAT 标准接口 schema 1.0

`results/autotune/aircraft_aero.mat` 只暴露一个顶层变量 `AERO`：

```text
AERO.meta
AERO.reference
AERO.flight_points
AERO.trim
AERO.longitudinal
AERO.lateral
AERO.controls.aileron
AERO.controls.elevator
AERO.controls.rudder
AERO.validation
```

`meta` 保存 schema_version、飞机名、创建时间、OpenVSP 版本、solver、模型 SHA-256、坐标约定和每项导数定义。`reference` 保存 Sref/bref/cref/CG。`flight_points` 保存 V、rho、qbar、Mach、Reynolds、三个配平角。`trim` 保存六分量系数、残差和迭代数。稳定与控制分组按 manifest 输出 23 项数组。`validation` 保存各速度点 overall/trim/convergence/derivative/physics status 及 rate limitation。

所有数组先按 `(V, beta)` 排序，并由同一 accepted flight-point 列表构造，因此第 i 个元素始终对应同一个 `V(i)`。导出后程序立即用 SciPy 重新读取并检查 `AERO.meta.schema_version`、flight_points 和关键导数字段；回读失败即导出失败。接口变更必须升级 schema version，不能静默改变字段。

## 22. CSV 与 JSON

- `aero_database.csv`：GRID/TRIM 展平汇总。
- `trim_derivatives.csv`：每速度、每导数的名义值、0.5/1/2 值、单位、定义、方法、变化率、状态、是否具备 minus sample 和限制说明。
- `validation_report.csv`：六级验证明细。
- `fuselage_effect_validation.csv`：thin-only 与 thin+thick 比较。
- `regression_report.csv`：基线/current/差值/阈值/状态。
- `aero_database.json`：完整内部追溯数据，包括每个扰动的 state 和系数。

CSV 供人审查，JSON 供调试与追溯，MAT 是稳定机器接口；三者不互相替代。

## 23. 固定飞机 regression test

`tests/regression/regression.yaml` 固定飞机、几何选择、求解设置、TRIM 和扰动；默认 `python run.py regression` 总是使用它，不受用户随后修改主配置的影响。显式传 `--config` 才会改用另一回归配置。`tests/regression/baseline.json` 来自 2026-09-01 对 `test_aircraft.vsp3`、OpenVSP 3.51.3、V=8 m/s smoke 结果的人工 sanity check。基线包含 alpha/elevator trim 及全部 23 项 required derivatives，并记录模型 SHA、OpenVSP 版本和已知 rate WARN；回归先要求两个身份字段精确匹配，再比较数值。

`python run.py regression` 计算或复用同签名 TRIM 点，逐量应用：

```text
PASS limit = pass_absolute + pass_relative * |baseline|
WARN limit = warn_absolute + warn_relative * |baseline|
```

缺失或非有限 current 直接 FAIL。回归的目的不是证明绝对物理正确，而是发现代码、版本或模型变化造成的非预期漂移。只有审阅过的新基准才能替换 baseline。

## 24. Cache / resume

结果缓存位于 `results/cases/<case_id>/result.json`。签名覆盖内部 schema/tool 版本、OpenVSP 版本、模型哈希、参考量、geometry 集、solver、大气、控制、manifest、扰动、validation 和工况。`resume.enabled: true` 且签名、模式和可用状态一致时显示 `CACHED`；失败结果和不完整 TRIM 导数不会被复用。

修改模型或相关配置会自动失效。若只想强制重算，可临时设 `resume.enabled: false`；不要手工改 result.json 冒充成功。

## 25. Solver 异常与错误处理

用户级错误被转换为 `RUN FAILED / Reason / Suggestion`，`--debug` 才显示 traceback。单 case 异常会写 result.json 的 error、solver FAIL 和输入状态，方便断点后定位。程序显式检查 OpenVSP/binding/VSPAERO、模型、geometry、控制面、参考量、输出字段、finite values、配平、required 集和 MAT 回读。

原始 `vspaero_console.txt`、polar/stab/history/geometry 文件留在各 case raw 目录；终端不回显数千行 solver 日志。

## 26. 新飞机迁移方法

1. 复制 `.vsp3` 到 aircraft，保留固定 regression 飞机。
2. 在 OpenVSP 校正 VSPAERO Reference 与 CG。
3. 建立 Aileron/Elevator/Rudder groups，逐一核对 surfaces/gains/正偏角。
4. 在主配置更新模型路径、质量、工况、geometry selectors、控制限制和必要的参考来源。
5. 先 `check`，再运行单个 GRID 与单个 TRIM。
6. 审查 alpha/elevator、残差、β=0 对称性、预期符号、三尺度值和机身差异。
7. 提高 tessellation/wake settings，单独完成生产级网格/尾迹收敛，再扩大 V/alpha/beta。
8. 只有下游状态空间需求改变时才更新 manifest；同时升级消费端映射与测试。
9. 决定 WARN 是否可进入 MAT；认证或自动飞行用途通常应更严格。

## 27. 常见故障排查

| 现象 | 常见原因 | 处理 |
|---|---|---|
| OpenVSP 未找到 | 路径/环境变量错误 | 修改 `config/openvsp.yaml` 或 `OPENVSP_ROOT` |
| binding import fail | Python 位数/版本不匹配 | 使用 Python 3.11 64 位并重建 `.venv` |
| model contains no geometry | 文件损坏或路径错误 | 在 OpenVSP GUI 打开并保存，再检查路径 |
| geometry matched 0/多个 | name/type 不一致或重名 | 用唯一 name+type 或 id selector |
| control group missing | 组未建、名称不一致 | 在 VSPAERO Settings 建组并核对大小写 |
| TRIM 不收敛 | 重量/CG/参考量错误、范围小、舵效方向错 | 检查输入，扩大范围，查看迭代 history |
| lift residual 大 | CL 目标不合理或网格/状态异常 | 检查单位、Sref、rho、mass 和 raw polar |
| moment residual 大 | CG 或 elevator gain 有误 | 核对 Xcg/Zcg、尾翼、舵面 gain |
| 三尺度 WARN/FAIL | 噪声、非线性、步长不合适 | 对照三个值与 plus/minus，调整统一配置后重算 |
| beta=0 不对称 | 几何/网格/控制中立不对称 | 检查左右部件、gain、neutral 和面板 |
| required missing | manifest 与解析字段/求解方法不匹配 | 查看 result.json 和 manifest，不要放宽为成功 |
| MAT 未生成 | TRIM FAIL 或 WARN 未被允许 | 查看 validation summary；修复或显式评估 WARN |
| regression FAIL | 模型、版本、配置或代码改变 | 查看逐量报告；确认预期后才更新基线 |
| 运行很慢 | TRIM + 30 个中心差分 polar + rate runs | 先用 smoke；缓存后重复运行会快速复用 |

## 28. 当前已验证状态与限制

本轮固定 smoke 配置已经真实运行 OpenVSP/VSPAERO。TRIM 点收敛；23/23 required derivatives 均生成且有限；常规固定翼符号和 beta=0 对称性通过；机身参与检查通过；MAT schema 1.0 可回读；固定基线回归通过。

真实限制必须保留：

- p/q/r 没有公共负 rate 输入，因此不是严格中心差分并标记 WARN；更严格需求需修改/扩展 VSPAERO 本体或升级到公开该能力的版本后重新审计。
- 当前 `wake_iterations: 3` 和低 tessellation 是 smoke 设置，不能当作生产气动精度证据。
- 经验物理符号只适用于配置声明的常规固定翼；特殊布局需修改 manifest/validation 规则。
- VSPAERO 低速线性导数不覆盖分离流、失速、强非线性、推进器耦合或地面效应，使用范围必须由项目另行确认。

OpenVSP/VSPAERO 原始定义的审计依据为 OpenVSP 3.51.3 的 VSPAERO 源码实现；升级求解器版本后应重新核对 CMl/CMm/CMn 和 p/q/r 分母，再决定是否更新坐标 metadata 与 regression baseline。

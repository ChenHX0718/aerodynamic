# OpenVSP/VSPAERO 气动数据工具——详细说明

## 1. 目标与数值策略

工程从 `.vsp3` 生成 GRID 气动数据库、纵向 TRIM、required stability/control derivatives、分级验证和 MATLAB `AERO` 数据。Production 数值策略由三部分组成：

1. 全飞行包线共用一套已验证的 tessellation preset；
2. Wake Iteration 由实测代表工况形成的 `required_wake=f(V, alpha, beta)` 离散 Schedule 决定；
3. 每个 Trim 基准点和全部导数扰动组成 derivative bundle，bundle 内统一采用所需 Wake 的最大值。

数值收敛只在少量气动代表状态运行，不把 3/5/8/12 或 COARSE/MEDIUM/FINE 铺到整个 GRID。Adaptive GRID 本版只保留受门禁保护的接口和 midpoint error evaluator。

## 2. 完整工作流

```text
配置与 .vsp3 检查
        |
一点式真实 smoke
        |
Wake 代表状态 × [3,5,8,12]：六系数 + 关键真实中心差分导数
        |
最小连续稳定等级 + native derivative 诊断 + Wake Map + 边界连续性修正
        |
代表状态 × [COARSE,MEDIUM,FINE]
        |
CY_delta_r / Cm_q 专项诊断
        |
production_numerical_settings.yaml + Production Gate
        |
正式 GRID + Pre-Trim -> Production Trim
        |
统一 bundle Wake 的 0.5Δ/Δ/2Δ 导数
        |
validation + regression + CSV/JSON/MAT
```

Numerical Convergence 的每个真实 VSPAERO case 独立签名、独立保存和独立失败。某一代表点失败时，其他固定状态继续；仅真正依赖失败状态的后续步骤标为 `SKIPPED_DEPENDENCY`。无论 Overall 是否 FAIL 都生成正式 JSON/Markdown 报告。

唯一 Python 入口是 `run.py`。正式命令是 `grid`、`trim`、`all`；收敛命令是 `numerical-convergence`；`smoke` 只运行一个真实 VSPAERO 状态。

## 3. 配置职责

`config/aircraft.yaml` 是飞机与数值研究的统一配置；`config/openvsp.yaml` 是唯一允许保存 OpenVSP 根路径的位置；`config/required_derivatives.yaml` 是唯一权威导数清单。

主要配置区：

- `operating_conditions`：GRID 的 V/alpha/beta，可用显式 `values` 表示人工非均匀 GRID，也可用 `start/step/end`。
- `grid.mode`：`uniform` 或预留的 `adaptive`。
- `trim`：质量、重力、V/beta、alpha/elevator 初值、边界、残差容限和迭代限制。
- `derivatives`：0.5Δ/Δ/2Δ、各变量基准步长及导数步长收敛容差。
- `numerical_convergence.wake`：唯一 Wake 候选、代表状态、双容差、邻域、边界缓冲和安全裕度。
- `numerical_convergence.tessellation`：三档按几何职责设计的剖分和代表状态。
- `numerical_convergence.trim`：Pre-Trim Wake 与跨区升级上限。
- `numerical_convergence.diagnostics`：少量诊断状态、rudder 步长和容差。
- `numerical_convergence.production_gate`：是否要求正式运行先通过数值门禁。

`solver.wake_iterations` 和 `solver.tessellation_overrides` 是 smoke/强制回退设置。Production 计算优先读取收敛输出，不能把它们误认为全包线统一的正式设置。

## 4. GRID

GRID 对配置轴做笛卡尔积，每点执行 VSPAERO stability sweep，保存六个基准气动力/力矩系数、OpenVSP 原生稳定导数、控制导数和求解证据。每个点使用统一 Production tessellation，但单独查询 Wake Schedule。

人工非均匀 GRID 保持不变：任何轴都可写 `values: [...]`。当前 `adaptive` 接口只提供：

- `grid.mode=uniform/adaptive`；
- Production Convergence 必须为 PASS 的硬约束；
- `midpoint_interpolation_error()`，比较两端线性插值与真实中点 VSPAERO 值，并使用统一双容差。

本版没有按 alpha 人工加密，也没有自动递归扩展 GRID。

## 5. TRIM 与两阶段算法

纵向水平直飞配平目标为：

```text
qbar*S*CL - mass*g = 0
qbar*S*cref*Cm      = 0
```

`trim_solver.py` 不再用 VSPAERO native derivative 作为正式 Newton Jacobian。每轮在当前点执行 alpha `±Δalpha` 和 elevator `±Δelevator` 的真实气动计算，调用 `finite_difference.py` 的统一 degree→radian 中心差分，构造：

```text
J = [dF/dalpha     dF/delevator
     dM/dalpha     dM/delevator]
```

native `CL_alpha/Cm_alpha/CL_delta_e/Cm_delta_e` 仍从基准 stability 结果读取，并与 FD 值一起写入迭代历史，但只作诊断。Newton 原始步先经过原有最大角度限制，再按 `lambda=1, 0.5, 0.25, 0.125` 做真实回溯：完整步改善就立即接受，只有未改善才缩步。历史记录原始步、受限步、每次尝试、最终 lambda、实际步长和新旧归一化残差；所有候选均不改善时保留当前状态并明确 FAIL。

Pre-Trim 与正式 TRIM 的 `max_iterations` 均为 15。成功条件始终是同一次迭代同时满足 `|Force residual|<=1 N` 和 `|Moment residual|<=1 N·m`；第 15 次只是停止点，未同时满足时必须 FAIL，不能因达到上限而成功。

Production TRIM 分两阶段：

1. Pre-Trim 使用配置的低成本安全 Wake 和 COARSE tessellation，目标仅是得到近似 `alpha_trim/elevator_trim`。
2. 根据 Pre-Trim 的 `(V,alpha,beta)` 以及整个 derivative bundle 查询 Schedule，选择固定 Production Wake，在统一 Production tessellation 下从头完成正式 TRIM。

正式 TRIM 完成后再次查询 bundle Wake。若进入更高区域，则只在完整 TRIM 之间升级 Wake，并用上次解作新初值重新正式配平；不在单轮 Newton 迭代中来回切换。达到升级上限仍不一致时明确 FAIL。最终导数只围绕最后一次 Production Trim 计算。

## 6. Required derivatives

`config/required_derivatives.yaml` 定义每项导数的名称、类别、系数、扰动变量、单位、定义和可选预期符号。当前 23 项覆盖：

- 纵向：`CL_alpha, CD_alpha, Cm_alpha, CL_q, Cm_q`；
- 侧向：`CY/Cl/Cn_beta`、`CY/Cl/Cn_p`、`CY/Cl/Cn_r`；
- 升降舵：`CL/CD/Cm_delta_e`；
- 副翼：`CY/Cl/Cn_delta_a`；
- 方向舵：`CY/Cl/Cn_delta_r`。

正式 23 项导数输出和 tessellation 监控仍由该 manifest 驱动。Wake 的二级气动 Gate 按本版明确要求监控八项关键真实 FD 导数：`CL_alpha, Cm_alpha, Cm_delta_e, CY_beta, Cl_beta, Cl_delta_a, Cn_beta, Cn_delta_r`；manifest 本身没有删改。

## 7. 中心差分与 derivative bundle

alpha、beta、aileron、elevator、rudder 使用真实中心差分：

```text
dC/dx = [C(x0+h)-C(x0-h)]/(2h)
```

角度输入为 deg，分母转换为 rad，导数单位为 `1/rad`。每个变量计算 `h=0.5Δ, Δ, 2Δ`。基准点、所有正负扰动和 rate 敏感性运行构成同一个 derivative bundle。

Schedule 首先对 bundle 中 base、`alpha±2Δalpha`、`beta±2Δbeta` 查询。控制偏转不改变 Schedule 的三个自变量，但其所有 ± 样本仍被固定到同一个 `bundle_wake=max(required_wake_of_all_bundle_states)`。因此不会出现中心差分两侧分别使用 Wake 5 和 8 的情况。

结果的 `derivatives.bundle_wake_iterations` 和 `bundle_rule` 明确记录该约束。

TRIM Jacobian、Wake 关键导数 Gate 和正式 0.5Δ/Δ/2Δ 导数都复用 `finite_difference.py` 中同一套状态扰动、舵面限位、degree/radian 分母与符号映射，不存在第二套角度或符号实现。

## 8. 0.5Δ/Δ/2Δ 收敛

公共判据为 absolute tolerance + relative tolerance：

```text
scale      = max(|a|, |b|, near_zero_reference)
limit      = absolute + relative*scale
difference = |a-b|
```

PASS 和 WARN 各有一组 limit，超过 WARN 为 FAIL。`near_zero_reference` 防止接近零的量因相对变化比例过大而误判。导数记录保留三组中心差分原始正负样本、导数值、变化、limit 和状态。

## 9. Wake Iteration 收敛

候选 `[3,5,8,12]` 只来自配置。每个代表状态、每个 Wake 执行一级六系数检查，并以配置的正式扰动步长执行二级真实中心差分检查，至少包含 `CL_alpha, Cm_alpha, Cm_delta_e, CY_beta, Cl_beta, Cl_delta_a, Cn_beta, Cn_delta_r`。一级和二级从同一候选开始的全部后续相邻转换都 PASS，Wake aerodynamic convergence 才 PASS。

同一基准 stability case 的 VSPAERO native derivatives 继续完整保存在单 case 缓存，报告对关键 native derivatives 做相邻 Wake 比较，但该结果是 `Native derivative diagnostic`，不再进入 Wake aerodynamic Gate。FD PASS 而 native FAIL 时，Wake 仍可 PASS，native diagnostic 降为 WARN；不会为了 native 跳动增加 Wake 16/20。

最小值选择不是只看一次相邻变化。选择某等级的条件是：从该等级开始的所有后续相邻转换都必须为 PASS。例如 3→5 PASS、5→8 FAIL、8→12 PASS 时只能推荐 8，不能推荐 5。若 8→12 为 WARN，只能把 12 标为“保守但未经验证”的 WARN；若为 FAIL，状态为 FAIL。最高候选从不自动等于正确答案。

代表状态覆盖低迎角线性区、真实巡航 Trim、中迎角、较大迎角和必要 beta。状态本身可以由用户按飞机任务选择，但 `required_wake` 必须由真实求解结果生成，不能按 alpha 阈值手写。

## 10. Wake Convergence Map / Schedule

输出 Schedule 保存每个实测 `(V,alpha,beta,required_wake,status)`。查询规则是：

1. 实测点直接使用其实测 required Wake；
2. 未实测点在归一化 V/alpha/beta 空间寻找局部邻域；
3. 位于区域边界缓冲范围时合并相邻区域并取较高离散 Wake；
4. 对未实测点再施加配置的离散 safety margin；
5. 结果始终属于候选集合，绝不线性插值。

轴归一化尺度从实测状态跨度生成，避免速度量纲支配 alpha/beta。

## 11. Wake 边界连续性

若相邻代表区域 required Wake 不同，程序按归一化距离选择有限个最近的异等级区域对，在二者中点分别运行低/高 Wake，并比较同一套六系数和八项关键真实 FD derivatives。

边界比较超出 PASS 时，低 Wake 端点自动升级到高 Wake，并在报告中记录 action。修正后的边界状态至少为 WARN，提醒审阅发生过区域收缩/升级；没有跨等级边界时状态为 PASS。该机制避免 Schedule 本身引入数据库折点。

## 12. Tessellation convergence

流程严格位于 Wake 收敛之后。COARSE/MEDIUM/FINE 对主翼、平尾、垂尾、机身分别配置 `Tess_U/Tess_W`；升力面还可配置 OpenVSP `WingGeom/LECluster` 和 `TECluster`。主翼/尾翼控制面附近通过弦向离散和前后缘聚类获得更高分辨率，机身则分别控制长度和周向离散。三个 preset 不是统一倍增。

在配置的少量代表状态、Schedule 和 bundle Wake 下比较全部监控量。只有所有后续转换稳定才推荐更低档：

- COARSE→MEDIUM 与 MEDIUM→FINE 均 PASS：可推荐 COARSE；
- COARSE→MEDIUM 较大、MEDIUM→FINE PASS：推荐 MEDIUM；
- MEDIUM→FINE 非 PASS：状态 FAIL，FINE 仅表示最高已算网格，不能宣称收敛。

所有 Production 状态统一使用最终推荐 preset，本版不做分区 tessellation。

## 13. CY_delta_r 专项诊断

只在配置的少量代表 Trim 状态运行方向舵 `±0.5°/±1°/±2°/±4°`，保存原始 `CY/Cl/Cn` 和三项中心导数。分类逻辑为：

- `LOCAL_LINEAR_VALID`：小/中步长连续稳定；
- `NUMERICAL_NOISE`：最小步长不稳定，但较大一组相邻步长稳定；
- `LARGE_DEFLECTION_NONLINEAR`：小步长稳定而较大步长偏离；
- `NONLINEAR_OR_NUMERICALLY_UNRESOLVED`：无法找到明确稳定的局部区间。

只有前三种存在明确稳定区间时才输出 `recommended_delta_r_deg`。额外舵偏仅用于诊断，不扩展到整个 GRID。

## 14. Cm_q 限制与数值敏感性

OpenVSP 3.51.3 公开 `VSPAEROSweep` API 没有负 p/q/r 稳态单点输入。工程不会伪造 `-q`；p/q/r 保留 VSPAERO 原生正向归一化差分，负样本为 `null`，方法状态保持 WARN。

Numerical Convergence 从 Wake 和 tessellation 两组已有 stability 结果提取 `Cm_q`，逐级应用双容差。即使数值敏感性为 PASS，总方法仍为 WARN；若提高 Wake/网格后仍变化明显，`numerical_status` 继续 WARN/FAIL，供人工决定是否接受。该 native rate 限制是诊断信息，不替代能够真实计算的 alpha/beta/control 中心差分 Gate。

## 15. Production Numerical Settings 与 Gate

`results/numerical_convergence/production_numerical_settings.yaml` 是正式计算唯一数值产物，包含：

- Production tessellation 名称、完整 overrides、推荐理由和状态；
- Wake Schedule、候选、归一化尺度、边界缓冲、安全裕度和实测点；
- derivative bundle 最大 Wake 规则；
- 两阶段 Trim 与最大跨区升级次数；
- 稳定时才给出的控制扰动建议；
- CY_delta_r、Cm_q 诊断；
- Production Gate 状态和原因。

文件通过模型 SHA-256、OpenVSP 版本和收敛配置 SHA-256 绑定生成环境；身份包含 TRIM、solver、derivative、geometry、atmosphere、control 和 numerical-convergence 关键设置，任一相关内容不匹配都会使 Gate 变为 FAIL，防止新飞机或新算法误用旧 Schedule。

Gate 组合 Wake、Wake boundary 和 tessellation：PASS 允许；WARN 允许但进入最终数据库状态；FAIL 默认在任何正式 case 前阻止。`--force` 只允许显式试跑并写 `production_force_override.json`，不会洗掉 FAIL。Adaptive GRID 不接受强制绕过，必须真实 PASS。

## 16. Validation

正式结果仍执行原有多层验证：

- `SOLVER`：结果存在且有限；
- `TRIM`：残差、范围和收敛；
- `NUMERICAL`：0.5Δ/Δ/2Δ；
- `DERIVATIVE`：manifest 完整性与方法状态；
- `PHYSICS`：常规固定翼符号、对称性和幅值；
- `DATASET`：机身参与、路径可移植性和 Production Gate。

状态按最严重项传播。WARN 可按配置决定是否允许 MAT；FAIL 不会生成完整可接受的调参数据。

## 17. 坐标系与单位

内部机体系：`+X` 向前、`+Y` 向右、`+Z` 向下；`Cl/Cm/Cn` 为绕相应正轴的右手力矩。转换集中在 `coordinate_system.py`：

```text
CX=-CFx, CY=CFy, CZ=-CFz
Cl=-CMx=CMl, Cm=CMy=CMm, Cn=-CMz=CMn
```

无量纲角速度：

```text
p_hat=p*bref/(2V)
q_hat=q*cref/(2V)
r_hat=r*bref/(2V)
```

配置/状态角为 deg；角度与舵偏导数分母为 rad。

## 18. 输出文件

数值收敛目录只保留统一报告、三张核心表、Production YAML、少量图及必要 raw：

- `numerical_convergence_report.md/.json`；
- `wake_convergence_map.csv/.png`；
- `tessellation_convergence.csv/.png`；
- `derivative_diagnostics.csv`；
- `CY_vs_delta_r.png`；
- 有异等级边界时的 `wake_boundary_check.png`；
- `production_numerical_settings.yaml`。

`raw/<signature>/` 是简单文件缓存而不是数据库。signature 至少覆盖模型哈希、V/alpha/beta、全部舵偏、Wake、tessellation、analysis type、perturbation、OpenVSP 版本和影响结果的配置。`case_result.json` 只有 `status=SUCCESS` 且六系数/映射结果完整时才可命中；FAIL、损坏或不完整记录会重算。启动 numerical-convergence 不再删除全部 raw，配置或模型变化只会产生新 signature；失败目录同时保留失败原因和已有 VSPAERO 控制台日志。

正式数据库位于 `results/latest/`；验证在 `results/validation/`；固定回归在 `results/regression/`；最终 MATLAB 文件为 `results/autotune/aircraft_aero.mat`。

## 19. MAT Schema

`.mat` 唯一顶层变量为 `AERO`，schema 1.0：

- `AERO.meta`：飞机、模型哈希、OpenVSP、坐标约定、导数元数据和 Production Gate/tessellation/Wake rule 摘要；
- `AERO.reference`：Sref/bref/cref 与 CG；
- `AERO.flight_points`：V、rho、qbar、Mach、Re、Trim 状态；
- `AERO.trim`：六系数和残差；
- `AERO.longitudinal/lateral`：稳定导数数组；
- `AERO.controls.aileron/elevator/rudder`：控制导数数组；
- `AERO.validation`：逐点状态和 rate limitation。

导出后立即用 SciPy 回读并检查关键字段，避免生成不可读 MAT。

## 20. Cache、失败隔离与 regression test

正式 case 的 `result.json` 签名覆盖模型哈希、OpenVSP 版本、参考量、geometry、solver、manifest、导数配置、validation、Numerical Convergence 配置和实际 Production Numerical Settings。签名不一致不会误用旧缓存。

Numerical Convergence 使用上述逐 VSPAERO case 缓存。代表点、各 Wake、各 FD 正负扰动、各 tessellation 和诊断 case 分别捕获异常并写入报告；一个 case FAIL 不会抛弃其他独立任务。Tessellation 或专项诊断若确实需要失败的 cruise trim，则明确写 `SKIPPED_DEPENDENCY`。最终报告的 `cache.hits/misses/failed_cases/solver_duration_sec/wall_duration_sec` 可用于确认断点续算是否生效。

`python run.py regression` 默认固定使用 `tests/regression/regression.yaml` 与 `tests/regression/baseline.json`，不受主配置变化影响。回归先比较模型 SHA/OpenVSP 身份，再对 Trim 与全部 23 项 required derivatives 使用固定双容差。

普通 `unittest` 使用 synthetic/mock 数据，不调用长时间 VSPAERO，覆盖 Wake 候选、关键 FD Gate、连续稳定、双容差、安全裕度、Map/边界查询、bundle、15 次 TRIM 上限、中心差分 Jacobian、回溯、失败隔离、逐 case cache/resume、tessellation、CY_delta_r、Cm_q、Gate 和 midpoint evaluator。真实 OpenVSP smoke 由 `python run.py smoke` 单独运行。

## 21. 主要模块职责

| 模块 | 职责 |
|---|---|
| `run.py` | 唯一启动入口 |
| `src/main.py` | 命令编排、GRID、两阶段 TRIM、门禁和导出协调 |
| `src/numerical_convergence.py` | 双容差、Wake Map/查询/bundle/边界、tessellation、诊断、Gate、报告和少量图 |
| `src/vspaero_runner.py` | 单次求解，接收显式 Wake 与 tessellation override |
| `src/openvsp_interface.py` | 模型、几何 set、控制组、Tess_U/W 和聚类参数 |
| `src/trim_solver.py` | 15 次上限、中心差分 Jacobian、有界 Newton 与回溯 line search |
| `src/finite_difference.py` | 统一状态扰动、degree/radian 中心差分和 manifest 三尺度导数引擎 |
| `src/coordinate_system.py` | 唯一坐标、符号和 rate 定义 |
| `src/validation.py` | case/导数/物理/数据集验证 |
| `src/export_results.py` | CSV、JSON、MAT schema 与回读验证 |
| `src/case_generator.py` | GRID/TRIM case 生成和稳定 ID |
| `src/regression.py` | 固定基线数值比较 |
| `src/config_loader.py` | 配置、路径和完整性校验 |

## 22. 后续 Adaptive GRID 设计

后续自动加密必须在 Numerical Convergence PASS 后进行。对现有相邻 GRID 单元：先用端点数据库插值得到中点预测，再运行真实 VSPAERO 中点；把六系数及所需导数交给公共 midpoint error evaluator。只有误差超过容差才插入中点并继续局部检查。

该策略由真实插值误差驱动，不按 alpha 大小或手写区域加密；新点仍使用 Production tessellation、Wake Schedule 和 derivative bundle 规则。

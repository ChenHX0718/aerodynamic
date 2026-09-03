# OpenVSP 无人机气动数据库：快速使用

本工具使用 OpenVSP/VSPAERO 3.51.3 生成 GRID 气动数据、真实中心差分导数和纵向 TRIM 数据。正式导数以 centered finite difference（FD）为准；VSPAERO native derivatives 仅作诊断参考。

## 1. 首次准备

1. 安装 OpenVSP 3.51.3，并确认目录中有 `vsp.exe` 和 `vspaero.exe`。
2. 把 OpenVSP 路径写入 `config/openvsp.yaml`，或设置 `OPENVSP_ROOT`。
3. 运行 `setup.bat`创建项目环境。
4. 运行 `run_aero.bat check`检查模型、几何集、参考量和舵面映射。

## 2. 准备新的 OpenVSP 无人机模型

在 OpenVSP 中完成以下内容后再保存 `.vsp3`：

- 设置机翼、平尾、垂尾和机身几何；
- 在 VSPAERO Reference 中确认 `Sref`/`bref`/`cref` 和重心 `Xcg`/`Ycg`/`Zcg`；
- 建立 Aileron、Elevator、Rudder Control Surface Groups，检查 gain、铰链和正偏角方向；
- 把薄面升力体和厚体机身放入不同 Set；
- 由用户根据几何曲率、前后缘、舵面铰链和计算成本设置 Tess_U、Tess_W 及聚类参数。

网格现在完全由用户在 OpenVSP 建模阶段负责。脚本不覆盖模型网格，不运行 COARSE/MEDIUM/FINE，不认证 Tessellation convergence，也不因网格“未收敛”导致 Production Gate FAIL。脚本仍会检查当前网格能否生成完整 VSPAERO case 及有限数值输出。

## 3. 更换飞机时修改哪里

主配置是 `config/aircraft.yaml`，导数清单是 `config/required_derivatives.yaml`。

| 配置 | 含义 |
|---|---|
| `aircraft.model` | 新 `.vsp3` 模型路径 |
| `geometry_sets` | 薄面/厚体集合和几何选择器 |
| `reference` | 参考面积、展长、弦长和重心是从模型还是配置读取 |
| `controls` | 副翼/升降舵/方向舵组名、中立位和限位 |
| `trim.mass_kg` | 飞机质量，决定配平升力 |
| `operating_conditions` | GRID 的速度、迎角和侧滑角范围 |
| `trim.operating_conditions` | TRIM 速度和侧滑角 |
| `numerical_convergence.wake.representative_states` | Wake 验证的少量代表工况 |
| `numerical_convergence.wake.candidates` | 正式 Wake 候选，默认 3/5/8/12 |
| `numerical_convergence.wake.verification_only_level` | 仅用于验证 12 的 Wake=16 |
| `derivatives.fd_step_candidates_deg` | alpha/beta/各舵面分别使用的 FD 步长候选 |
| `derivatives.trim_jacobian_steps_deg` | TRIM Jacobian 的 alpha/elevator 中心差分步长 |
| `derivatives.convergence` | FD 步长稳定性的绝对+相对容差 |
| `required_derivatives.yaml` | 23 项 required 导数和 METHOD_LIMITATION 接受策略 |

不要为了获得 PASS 放宽容差、删除 required 导数或改动符号。

## 4. 运行 Numerical Validation

```bat
run_aero.bat numerical-convergence
```

此命令会：

- 在代表工况比较 Wake 3/5/8/12；
- 以六个基础系数和 required Wake FD derivatives 作为正式 Wake Gate；
- 当 8→12 未 PASS 时，仅对该代表工况运行 Wake=16；
- 在一个代表点为 alpha、beta、elevator、aileron、rudder 导数独立选择 FD step；
- 生成 required derivatives manifest 和 Production Gate。

如果 12→16 PASS，状态可判为已验证，但 Production Wake 仍是 12。如果 12→16 仍未 PASS，保持 WARN；Wake=16 不加入 GRID 或 Schedule。Native derivatives 只记录 diagnostic，不参与 Wake Gate。

## 5. 运行正式数据库

Numerical Validation 完成后：

```bat
run_aero.bat all
```

也可分开运行：

```bat
run_aero.bat grid
run_aero.bat trim
```

GRID 每点查询离散 Wake Schedule。TRIM 先用低成本 Wake 得到初值，再在固定 production bundle Wake 下重新配平和计算导数。每个 required alpha/beta/control 导数使用自己的 selected FD step。

TRIM 保持以下硬性规则：

- 最多 15 次迭代；
- 仅当力残差不超过 ±1 N 且俯仰力矩残差不超过 ±1 N·m 才 PASS；
- Jacobian 使用真实 alpha/elevator centered FD；
- 回溯步长为 1 / 0.5 / 0.25 / 0.125；
- native derivatives 不作 Jacobian。

## 6. 如何理解状态

| 状态 | 含义 |
|---|---|
| `PASS` | 数值、方法和完整性满足正式规则 |
| `WARN_NUMERICAL` | 导数有有限的步长敏感性，依 manifest 规则带警告接受 |
| `METHOD_LIMITATION` | 工具/API 无法实现所要求的方法；例如 3.51.3 无真实负 steady-q，`Cm_q` 不能伪装为 centered FD |
| `FAIL` | required 正式数据缺失、非有限、方法或数值检查不可接受 |

Manifest 在 `status_policy` 中显式规定各状态对 Gate 的处理。`METHOD_LIMITATION` 不会被偷偷跳过；报告会列出数量、原因和是否进入 production。

## 7. 输出和缓存

| 路径 | 内容 |
|---|---|
| `results/numerical_convergence/numerical_convergence_report.md` | 人可读 Numerical Convergence / Validation Report |
| `results/numerical_convergence/numerical_convergence_report.json` | 完整机读报告 |
| `results/numerical_convergence/production_numerical_settings.yaml` | 正式 Wake Schedule、selected FD steps、manifest 和 Gate |
| `results/numerical_convergence/wake_convergence_map.csv` | 代表点、Production Wake 和 Wake=16 验证状态 |
| `results/numerical_convergence/fd_step_selection.csv` | 逐导数 FD step 选择 |
| `results/numerical_convergence/required_derivatives_manifest.csv` | required 导数逐项状态 |
| `results/latest/aero_database.json/.csv` | 最新气动数据库 |
| `results/latest/trim_derivatives.csv` | 正式 FD/native diagnostic 明确分列的导数表 |
| `results/autotune/aircraft_aero.mat` | MATLAB/Simulink 数据；正式 FD 与 native diagnostic 分区 |
| `results/validation/` | 验证汇总和机身参与检查 |

`results/numerical_convergence/raw/<signature>/` 保存逐 VSPAERO case 缓存。Signature 区分模型哈希、状态、舵偏、Wake（包括 16）、当前模型网格身份和分析类型。重跑相同命令会复用完整成功的真实 case，不在启动时整体清空 raw。

如果 Gate FAIL，先查看报告中的 Wake、FD step 和 manifest 汇总，再按 signature 查看对应 `case_result.json` 和 `vspaero_console.txt`。

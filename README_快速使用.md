# OpenVSP/VSPAERO 气动数据工具——快速使用

本工具从 `.vsp3` 模型自动生成 GRID 气动数据、TRIM 点、23 项稳定/控制导数、分级验证报告，以及可直接供 MATLAB/Simulink 调参模型读取的 `AERO` 结构体。第一次使用请按本页顺序操作。

## 1. 软件要求

| 软件 | 本工程已验证版本 | 用途 |
|---|---:|---|
| Windows 64 位 | Windows 10/11 | OpenVSP 运行环境 |
| Python | 3.11 64 位 | 主程序；须与 OpenVSP Python binding 匹配 |
| OpenVSP/VSPAERO | 3.51.3 | 几何、面元/厚体和气动求解 |
| MATLAB | 非必需 | 只在下游读取 `.mat` 时需要 |

Python 依赖列在 `requirements.txt`，包括 NumPy、SciPy、PyYAML 和 six。

## 2. 首次安装与检查

1. 安装 OpenVSP 3.51.3，并确认安装目录中存在 `vspaero.exe` 和 `python` 目录。
2. 双击 `setup.bat`。脚本创建 `.venv`、安装依赖并执行环境检查。
3. 如果 OpenVSP 未被自动找到，打开 `config/openvsp.yaml`，设置：

   ```yaml
   openvsp:
     root: "D:/OpenVSP/OpenVSP-3.51.3-win64"
     expected_version: "3.51.3"
   ```

   也可以设置环境变量 `OPENVSP_ROOT`。路径优先级为配置、环境变量、常见安装位置。
4. 在工程根目录运行：

   ```powershell
   .\.venv\Scripts\python.exe run.py check
   ```

看到 OpenVSP、geometry、control mapping 和 required derivatives 均通过后再运行计算。

## 3. 新飞机的 11 个步骤

1. 将新的 `.vsp3` 文件放入 `aircraft/`。
2. 在 `config/aircraft.yaml` 修改 `aircraft.name` 和 `aircraft.model`。
3. 修改 `trim.mass_kg`；程序用 `mass × gravity` 得到重量。
4. 选择 CG 来源。默认 `reference.cg_source: model`，此时在 OpenVSP 的 VSPAERO Reference 中设置 Xcg/Ycg/Zcg；若改成 `config`，在 `reference` 下填写 `xcg_m/ycg_m/zcg_m`。
5. 选择参考量来源。默认从模型读取 Sref/bref/cref；只有改成 `reference.source: config` 时才在配置中填写 `sref_m2/bref_m/cref_m`。
6. 在 OpenVSP 建立 Aileron、Elevator、Rudder 三个 Control Surface Group，并在 `controls` 中填写完全一致的组名和偏角限制。
7. 在 `geometry_sets` 中列出 thin surface（机翼、平尾、垂尾）和 thick body（机身），名称和类型必须唯一匹配模型。
8. 在 `operating_conditions` 设置 GRID 的 V/alpha/beta；在 `trim.operating_conditions` 设置 TRIM 的 V/beta。
9. 先运行小 GRID：`.\.venv\Scripts\python.exe run.py grid`。
10. 再运行一个 TRIM：`.\.venv\Scripts\python.exe run.py trim`；确认没有 FAIL 后，可运行 `all`。
11. 运行固定飞机回归：`.\.venv\Scripts\python.exe run.py regression`。不带 `--config` 时它固定使用 `tests/regression/regression.yaml`，因此修改主配置不会悄悄改变基准飞机；不要用新飞机结果覆盖随工程交付的基线。

## 4. 运行命令

所有功能共用一个 Python 入口：

```powershell
.\.venv\Scripts\python.exe run.py check
.\.venv\Scripts\python.exe run.py grid
.\.venv\Scripts\python.exe run.py trim
.\.venv\Scripts\python.exe run.py all
.\.venv\Scripts\python.exe run.py regression
```

`run_aero.bat` 等价于先 `check` 再 `all`。指定另一份主配置可加 `--config D:\path\aircraft.yaml`；排错时可加 `--debug`。

## 5. 主要配置参数

除路径外，所有角度状态用 deg，角速度用 rad/s，速度用 m/s，质量用 kg。

| 配置键 | 单位 | 含义 | 示例 |
|---|---:|---|---|
| `openvsp.root` | 路径 | OpenVSP 安装根目录；`null` 为自动查找 | `D:/OpenVSP/OpenVSP-3.51.3-win64` |
| `aircraft.name` | — | 数据集中的飞机名称 | `test_aircraft` |
| `aircraft.model` | 路径 | 相对主配置文件的 `.vsp3` 路径 | `../aircraft/test_aircraft.vsp3` |
| `analysis.modes` | — | `all` 默认包含的模式 | `[GRID_DATABASE, TRIM_DATABASE]` |
| `operating_conditions.speed` | m/s | GRID 速度轴，可用 `values` 或 start/step/end | `{start: 8, step: 1, end: 9}` |
| `operating_conditions.alpha` | deg | GRID 迎角轴 | `{start: 0, step: 1, end: 1}` |
| `operating_conditions.beta` | deg | GRID 侧滑角轴 | `{values: [0]}` |
| `atmosphere.rho_kg_m3` | kg/m³ | 空气密度 | `1.225` |
| `atmosphere.dynamic_viscosity_pa_s` | Pa·s | 动力黏度 | `1.7894e-5` |
| `atmosphere.speed_of_sound_mps` | m/s | 音速，用于 Mach | `340.294` |
| `geometry_sets.thin_set_index` | — | 薄面集合编号 | `1` |
| `geometry_sets.thick_set_index` | — | 厚体集合编号 | `2` |
| `geometry_sets.thin/thick` | — | 按 name/type 选择几何 | `{name: Wing, type: Wing}` |
| `solver.wake_iterations` | 次 | 尾迹迭代；示例值仅供 smoke | `3` |
| `solver.ncpu` | 核 | VSPAERO 使用的 CPU 数 | `4` |
| `solver.tessellation_overrides` | 面板参数 | 各几何 Tess_U/Tess_W | `tess_u: 8, tess_w: 17` |
| `reference.source` | `model/config` | Sref、bref、cref 来源 | `model` |
| `reference.cg_source` | `model/config` | Xcg、Ycg、Zcg 来源 | `model` |
| `controls.*.group` | — | OpenVSP 控制面组名 | `Elevator` |
| `controls.*.neutral_deg` | deg | 中立偏角 | `0` |
| `controls.*.min_deg/max_deg` | deg | 允许的控制偏角范围 | `-20 / 20` |
| `trim.mass_kg` | kg | 飞机质量 | `25` |
| `trim.gravity_m_s2` | m/s² | 重力加速度 | `9.80665` |
| `trim.operating_conditions.speed` | m/s | TRIM 速度点 | `{values: [8]}` |
| `trim.operating_conditions.beta` | deg | TRIM 侧滑角 | `{values: [0]}` |
| `trim.alpha` | deg | 初值和搜索范围 | `initial 2, min -5, max 12` |
| `trim.elevator` | deg | 初值和搜索范围 | `initial 0, min -20, max 20` |
| `trim.force_tolerance_n` | N | 升力减重量残差容限 | `1.0` |
| `trim.moment_tolerance_nm` | N·m | 俯仰力矩残差容限 | `1.0` |
| `derivatives.manifest` | 路径 | 唯一 required derivative 清单 | `required_derivatives.yaml` |
| `derivatives.scales` | 倍数 | 步长敏感性检查 | `[0.5, 1.0, 2.0]` |
| `derivatives.perturbations.alpha_deg/beta_deg` | deg | 基础角度扰动 Δ | `0.5` |
| `derivatives.perturbations.p_rad_s/q_rad_s/r_rad_s` | rad/s | 速率参考扰动 | `0.01` |
| `derivatives.perturbations.*_deg` | deg | 三个舵面的基础扰动 | `2.0` |
| `derivatives.convergence.*` | 导数单位/比例 | PASS/WARN 绝对与相对容限 | `pass_relative: 0.10` |
| `validation.symmetry.*` | 系数 | β=0 对称性阈值 | `warn 0.02, fail 0.05` |
| `validation.physics.derivative_absolute_limit` | 导数单位 | 异常大导数上限 | `100` |
| `validation.autotune_allow_warn` | bool | 是否明确允许 WARN 进入 MAT | `true` |
| `regression.*` | 混合 | 自定义配置下的回归点、基线和比较容限；默认回归使用固定 fixture | `speed_mps: 8` |
| `resume.enabled` | bool | 签名一致时复用已完成结果 | `true` |
| `export.csv/json/mat` | bool | 输出开关 | `true` |

`config/required_derivatives.yaml` 是 23 项导数唯一权威定义，通常只在下游动力学接口改变时修改，不应在 Python 中另写列表。

## 6. 结果在哪里

| 路径 | 内容 |
|---|---|
| `results/autotune/aircraft_aero.mat` | MATLAB/Simulink 标准接口，唯一顶层变量 `AERO` |
| `results/latest/aero_database.csv` | GRID 与 TRIM 汇总，供人工查看 |
| `results/latest/trim_derivatives.csv` | 23 项导数、三尺度值、方法、单位和状态 |
| `results/latest/aero_database.json` | 完整可追溯内部结果 |
| `results/latest/run_summary.txt` | 本次命令简明总结 |
| `results/validation/validation_report.csv` | SOLVER/TRIM/NUMERICAL/DERIVATIVE/PHYSICS/DATASET 分级报告 |
| `results/validation/validation_summary.json` | 验证状态摘要 |
| `results/regression/` | 固定飞机回归 CSV/JSON |
| `results/cases/<case>/raw/` | 各真实求解器输出与控制台日志 |

MATLAB 示例：

```matlab
load('results/autotune/aircraft_aero.mat', 'AERO');
V = AERO.flight_points.V_mps;
Cm_alpha = AERO.longitudinal.Cm_alpha;
```

所有向量按同一 flight-point 顺序排列。FAIL 数据不会进入 MAT；WARN 只有在 `validation.autotune_allow_warn: true` 时才可进入。

## 7. 如何看 PASS / WARN / FAIL

- `PASS`：求解、配平、导数完整性、数值或物理检查达到 PASS 阈值。
- `WARN`：数据完整且可追溯，但存在已记录的非关键风险；先查看 validation report。
- `FAIL`：关键步骤失败、required 导数缺失/无效、配平失败或超过 FAIL 阈值；不得用于调参 MAT。

当前随工程提供的 smoke 配置预期整体为 `WARN`：OpenVSP 3.51.3 的公开 Sweep API 不提供负 p/q/r 稳态单点输入，因此 p/q/r 导数采用 VSPAERO 原生正向归一化差分并明确标记 WARN。alpha、beta 和三个控制面仍使用以 TRIM 点为中心的真实 ± 扰动。另请注意当前网格和 3 次尾迹迭代只验证流程，不代表生产级网格收敛。

## 8. 常见首次运行错误

- 找不到 OpenVSP：检查 `config/openvsp.yaml`、`OPENVSP_ROOT` 和版本目录。
- Python binding 导入失败：使用 64 位 Python 3.11，并重跑 `setup.bat`。
- 找不到 geometry：配置中的 name/type 必须与 `.vsp3` 完全对应且只匹配一个对象。
- 找不到控制面组：在 OpenVSP VSPAERO 设置中建立并命名 Control Surface Group。
- TRIM FAIL：检查重量/CG/参考量，扩大 alpha/elevator 搜索范围，并检查舵面 gain 方向。
- 导数 FAIL：查看 `trim_derivatives.csv` 的三个步长及原始目录，判断噪声、非线性或求解失败。

算法、坐标、MAT schema 和验证细节见 `README_详细说明.md`。

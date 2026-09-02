# OpenVSP/VSPAERO 气动数据工具——快速使用

本工具自动完成小规模数值收敛、按飞行状态选择 Wake、两阶段 TRIM、23 项稳定/控制导数、验证以及 MATLAB `.mat` 导出。正式大规模 GRID/TRIM 前必须先生成 Production Numerical Settings。

## 1. 首次安装

要求：64 位 Python 3.11、OpenVSP 3.51.3（含 VSPAERO）。在项目根目录运行：

```powershell
.\setup.bat
.\.venv\Scripts\python.exe run.py check
```

OpenVSP 路径只在 `config/openvsp.yaml` 设置：

```yaml
openvsp:
  root: "D:/OpenVSP/OpenVSP-3.51.3-win64"
  expected_version: "3.51.3"
```

`root: null` 会自动搜索；也可设置环境变量 `OPENVSP_ROOT`。不要把绝对路径写到其他代码或配置中。

## 2. 更换新飞机

1. 把新 `.vsp3` 放入 `aircraft/`。
2. 修改 `config/aircraft.yaml` 的 `aircraft.name` 与 `aircraft.model`。
3. 核对 `geometry_sets.thin/thick`；机翼、平尾、垂尾通常是 thin，机身是 thick。
4. 核对 `.vsp3` 内三个 Control Surface Group 名称，并修改 `controls.aileron/elevator/rudder.group`。
5. 核对模型内或配置内的 `Sref/bref/cref` 与 CG。
6. 修改大气、质量、重力、TRIM 搜索范围与舵面上下限。
7. 修改 GRID 的 V/alpha/beta 和 TRIM 的 V/beta。
8. 修改 `numerical_convergence.wake.representative_states`，覆盖线性区、巡航/Trim、中迎角、较大迎角和必要的 beta；不要按 alpha 人工写 Wake 分区。
9. 按新模型的几何名称调整 COARSE/MEDIUM/FINE 三套剖分。主翼、尾翼、机身和聚类参数应分别设计，不要简单统一乘倍数。
10. 运行 `check`，再运行一点式 `smoke`。
11. 运行完整 Numerical Convergence，通过门禁后再扩大正式 GRID/TRIM。

## 3. 必须修改或复核的配置

| 配置 | 作用 |
|---|---|
| `aircraft.model` | 当前 `.vsp3` 文件 |
| `geometry_sets` | thin/thick 几何选择及 set index |
| `controls` | 控制组名称、中立位和限位 |
| `reference` | 参考面积、展长、弦长和 CG 来源 |
| `trim.mass_kg`、`trim.alpha/elevator` | 重量和配平搜索范围 |
| `operating_conditions` | 人工非均匀或均匀 GRID 的 V/alpha/beta |
| `trim.operating_conditions` | 正式 TRIM 的 V/beta |
| `derivatives.perturbations` | 初始导数扰动；rudder 可由专项诊断更新 |
| `numerical_convergence.wake.representative_states` | Wake 收敛代表状态 |
| `numerical_convergence.*.tolerance` | 固定的绝对+相对容差，不得为求 PASS 临时放宽 |
| `numerical_convergence.tessellation.presets` | COARSE/MEDIUM/FINE 定制剖分 |

Wake 候选 `[3, 5, 8, 12]` 只在配置中定义。`solver.wake_iterations` 和 `solver.tessellation_overrides` 仅作为 check/smoke 或显式 `--force` 时的回退，不是正式 Production 设置。

## 4. 推荐运行顺序

```powershell
# 只检查环境、模型、几何和舵面映射
.\.venv\Scripts\python.exe run.py check

# 一个真实 OpenVSP/VSPAERO 工况，验证调用链和 COARSE 预设
.\.venv\Scripts\python.exe run.py smoke

# 小规模代表状态：Wake -> 边界 -> tessellation -> 两项专项诊断
.\.venv\Scripts\python.exe run.py numerical-convergence

# 门禁允许后运行正式数据
.\.venv\Scripts\python.exe run.py grid
.\.venv\Scripts\python.exe run.py trim
.\.venv\Scripts\python.exe run.py all

# 固定交付飞机的回归
.\.venv\Scripts\python.exe run.py regression

# 纯逻辑与交付结果测试
.\.venv\Scripts\python.exe -m unittest tests.test_core -v
```

Numerical Convergence 只跑配置中的少量代表状态，不会对整个飞行包线重复计算 3/5/8/12 或 COARSE/MEDIUM/FINE。

## 5. 如何查看 Wake Map

打开：

- `results/numerical_convergence/wake_convergence_map.csv`：每个实测代表工况的最小合格 Wake、状态与推荐理由。
- `results/numerical_convergence/wake_convergence_map.png`：代表状态的 Wake 分布概览。
- `results/numerical_convergence/numerical_convergence_report.md`：相邻级别、边界连续性和自动升级说明。

未实测状态采用归一化 V/alpha/beta 距离的保守离散邻域查询，边界附近取较高 Wake，并对未实测点增加安全等级。Wake 永不做普通线性插值，也没有 `alpha < 某值` 的硬编码规则。

## 6. 如何查看 tessellation 收敛

打开：

- `results/numerical_convergence/tessellation_convergence.csv`
- `results/numerical_convergence/tessellation_convergence.png`

程序先完成 Wake 研究，再在选定 Wake 下比较 COARSE/MEDIUM/FINE。只有 MEDIUM→FINE 稳定时才可推荐 MEDIUM；若该转换仍不合格，FINE 只代表最高已算等级，不会被冒充为已收敛。

## 7. Production Numerical Settings

最终统一文件是：

`results/numerical_convergence/production_numerical_settings.yaml`

其中包含：

- 一套 Production tessellation preset 及完整 overrides；
- 从实测状态生成的 Wake Schedule；
- 未实测点的离散查询、boundary buffer 和 safety margin；
- derivative bundle 最大 Wake 规则；
- Pre-Trim→Production Trim 规则与跨区升级上限；
- 推荐的控制扰动步长；
- CY_delta_r、Cm_q 与 Production Gate 状态。

不要手写或线性插值 Wake。正式计算会自动读取此文件。
文件还绑定模型哈希、OpenVSP 版本和收敛配置哈希；更换飞机或修改收敛设置后，旧文件会被门禁拒绝，必须重跑。

## 8. 正式 GRID/TRIM 的数值规则

- 每个 GRID 状态单独查询 Wake Schedule，使用统一 Production tessellation。
- TRIM 先用安全低成本 Wake 做 Pre-Trim，只定位近似解。
- 根据 Pre-Trim 状态和所有 ± 导数扰动状态查询 Wake；整个 derivative bundle 取其中最大 Wake。
- Production Trim 在这个固定 Wake 下完整重算。若最终 Trim 进入更高 Wake 区域，整轮升级后重算，不在单轮 Trim 中频繁切换。
- base、alpha±、beta±、aileron±、elevator±、rudder± 以及 rate 检查全部使用同一 bundle Wake。

人工非均匀 GRID 仍由 `values` 或 `start/step/end` 定义。`grid.mode: adaptive` 目前只开放门禁和 midpoint interpolation error 接口；只有 Numerical Convergence 为 PASS 才能启用，自动加密尚未展开。

## 9. PASS / WARN / FAIL

- `PASS`：Wake、边界连续性和 tessellation 均满足 PASS 容差，可正式运行。
- `WARN`：允许正式运行，但报告中存在必须审阅的数值或 API 限制。OpenVSP 3.51.3 无法提供真正的 `+q/-q` 稳态中心差分，因此 Cm_q 方法至少保持 WARN。
- `FAIL`：默认禁止正式 GRID/TRIM；最高 Wake 或 FINE 不会被自动视为正确。

确需在 FAIL 下试跑，可显式使用：

```powershell
.\.venv\Scripts\python.exe run.py grid --force
```

这会在 `results/numerical_convergence/production_force_override.json` 留下时间、命令和原因。`--force` 不会把 FAIL 改成 PASS；Adaptive GRID 仍必须真实 PASS。

## 10. 结果位置

| 文件 | 内容 |
|---|---|
| `results/numerical_convergence/numerical_convergence_report.md` | 人工审阅总报告 |
| `results/numerical_convergence/numerical_convergence_report.json` | 完整机器可读证据 |
| `results/numerical_convergence/wake_convergence_map.csv` | Wake 实测地图 |
| `results/numerical_convergence/tessellation_convergence.csv` | 三档剖分比较 |
| `results/numerical_convergence/derivative_diagnostics.csv` | CY_delta_r 与 Cm_q 诊断 |
| `results/numerical_convergence/production_numerical_settings.yaml` | 正式计算唯一数值设置产物 |
| `results/latest/aero_database.json/.csv` | GRID/TRIM 总数据库 |
| `results/latest/trim_derivatives.csv` | 导数及 0.5Δ/Δ/2Δ 证据 |
| `results/validation/` | 分级验证与机身参与检查 |
| `results/autotune/aircraft_aero.mat` | MATLAB/Simulink 最终文件；唯一顶层变量 `AERO` |
| `results/regression/` | 固定飞机回归结果 |

MATLAB 读取：

```matlab
load('results/autotune/aircraft_aero.mat', 'AERO');
```

若 `.mat` 未生成，先检查 Production Gate、TRIM、required derivatives 和 validation 是否存在 FAIL。

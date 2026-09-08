# OpenVSP 无人机气动数据库：快速使用

本工程使用 OpenVSP/VSPAERO 3.51.3 为 Simulink 非线性 6DOF 和 ArduPilot SITL 生成气动数据。

- GRID 保存 `C0(V, alpha, beta)` 非线性基础气动。普通 GRID 点不需要配平，`Cm/Cl/Cn != 0` 合法。
- GRID response 保存各控制面的真实多级响应，以及 GRID-local p/q/r 导数。
- TRIM 保留为初始状态、健康检查、局部线性化和 AutoTune 辅助分析，不再是 GRID 数据库的必需前置。

## 1. 推荐开关

`config/aircraft.yaml` 中只有这一处模式真值来源：

```yaml
analysis:
  grid_enabled: true
  trim_enabled: false
```

`all` 按这两个开关运行。两者都为 `false` 时会明确报错。显式 `grid` 和 `trim` 命令仍可分别单独运行。

## 2. uniform / adaptive GRID

```yaml
grid:
  mode: uniform       # 或 adaptive
```

- `uniform`：计算 `operating_conditions.speed/alpha/beta` 的规则笛卡尔积。
- `adaptive`：轴值是 seed grid，将真实 VSPAERO 中点与 corner 插值对比并局部细分。保留实际 seed/adaptive midpoint，不强制转成规则矩阵。

Adaptive GRID 需要 `solver_gate == PASS`，`--force` 不能绕过。

## 3. GRID response scan

小规模审计的推荐配置：

```yaml
response_scan:
  enabled: true
  scope: audit
  variables: [p, q, r, elevator, aileron, rudder]
  representative_states:
    - {speed_mps: 8, alpha_deg: 0, beta_deg: 0}
  control_levels_deg:
    elevator: [-5, 0, 5]
    aileron: [-5, 0, 5]
    rudder: [-5, 0, 5]
  rates:
    representation: local_linear_derivative
    source: vspaero_steady_stab
```

- `audit`：只扫描少量代表性非配平 GRID 点。`representative_states` 留空时自动选低/中/高速、高 alpha 和带 beta 点。
- `full_grid`：对 uniform 全部规则点，或 adaptive 实际接受的全部点扫描。

舵面值必须在 `controls.<name>.min_deg/max_deg` 内，必须包含并夹住 neutral。中立 sample 直接复用 GRID baseline。每个 sample 保存六系数绝对值和 `delta_CL ... delta_Cn`。

OpenVSP 3.51.3 当前工作流没有验证可靠的任意独立 body-rate sweep，因此不伪造 p/q/r 多级曲线。工程从每个 GRID 点已有的 steady `.stab` 读取六系数局部导数，标记 `local_linear_derivative`。

## 4. 运行

```bat
run_aero.bat check
run_aero.bat numerical-convergence
run_aero.bat all
run_aero.bat grid
run_aero.bat trim
```

`run_aero.bat` 会传递子命令及其他参数。首次运行前用 `setup.bat` 创建环境，并在 `config/openvsp.yaml` 设置 OpenVSP 目录或设置 `OPENVSP_ROOT`。

## 5. Gate 与交付

- Solver / GRID Gate：基础六系数 Wake 收敛、GRID 求解和 Adaptive 资格。
- Response Gate：控制面 sample 以及 GRID-local rate fields 完整性。
- Derivative Gate：centered FD、steady rate 与 23 项导数包的数值质量。
- TRIM validation：仅在 TRIM 实际运行时评估配平、Jacobian 和 TRIM derivatives。

GRID + response 通过时，即使 TRIM 关闭也会生成 Simulink MAT。线性分类仅是建议，不删除原始 samples。

## 6. 主要输出

| 路径 | 内容 |
|---|---|
| `results/latest/aero_database.csv` | 仅基础 GRID 点 |
| `results/latest/grid_response_samples.csv` | 控制面绝对/增量六系数 |
| `results/latest/grid_response_summary.csv` | 线性审计与 p/q/r 局部导数 |
| `results/latest/trim_database.csv` | 独立 TRIM 点 |
| `results/latest/trim_derivatives.csv` | TRIM 23 项 production derivative |
| `results/latest/aero_database.json` | 完整 GRID/response/TRIM/Gate/metadata |
| `results/autotune/aircraft_aero.mat` | Simulink 使用的 `AERO` schema 2.0 |

MAT 的 GRID 主结构为 `AERO.grid`、`AERO.responses.controls`、`AERO.responses.rates`、`AERO.linearity`和 `AERO.assumptions`。TRIM 通过时，旧 `flight_points/trim/longitudinal/lateral/controls` 仍保留。

## 7. 从 audit 升级到 full_grid

先审查 `grid_response_summary.csv`，再修改：

```yaml
response_scan:
  scope: full_grid
```

然后执行 `run_aero.bat grid` 或 `run_aero.bat all`。

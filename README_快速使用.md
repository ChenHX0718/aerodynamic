# OpenVSP 无人机气动数据库：快速使用

本工具使用 OpenVSP/VSPAERO 3.51.3 生成规则或自适应 GRID、纵向 TRIM，以及 23 项可追溯的 production 气动导数，并导出 CSV、JSON 和 MATLAB `.mat`。

## 1. 首次准备

1. 安装 OpenVSP 3.51.3，确认安装目录包含 `vsp.exe` 和 `vspaero.exe`。
2. 在 `config/openvsp.yaml` 配置安装目录，或设置 `OPENVSP_ROOT`。
3. 运行 `setup.bat` 创建环境。
4. 运行 `run_aero.bat check` 检查模型、几何集、参考量和舵面映射。
5. 运行 `run_aero.bat numerical-convergence` 生成与当前模型、OpenVSP 和验证配置匹配的三层 Gate。

模型的 Tessellation/mesh 由用户在 OpenVSP 中负责。脚本使用 `.vsp3` 当前网格，不覆盖 Tess_U/Tess_W，也不宣称完成网格独立性认证。

## 2. 选择 uniform 或 adaptive GRID

在 `config/aircraft.yaml` 中设置：

```yaml
grid:
  mode: uniform       # 或 adaptive
```

- `uniform`：计算 `operating_conditions.speed/alpha/beta` 的完整笛卡尔积。
- `adaptive`：相同轴值仅作为 seed grid；每个 cell 额外计算真实几何中点，用 corner 的线性、双线性或三线性插值预测中心，再与真实 VSPAERO 结果比较。非线性超限时只沿当前归一化跨度最大的轴局部二分；跨度相同按 V、alpha、beta 排序。
- 某轴只有一个值时自动成为 inactive 轴，因此同一实现支持 1D、2D 和 3D。

自适应设置示例：

```yaml
grid:
  mode: adaptive
  adaptive:
    quantities: [CL, CD, Cm, CY, Cl, Cn]
    tolerance:
      near_zero_reference: 0.05
      pass_relative: 0.03
      warn_relative: 0.08
      pass_absolute: 0.005
      warn_absolute: 0.02
    max_depth: 3
    max_cases: 100
    min_spacing: {speed_mps: 0.25, alpha_deg: 0.25, beta_deg: 0.25}
```

- `tolerance`：真实中点与插值中心的独立绝对+相对误差门限，不与 Wake 容差混用。
- `max_depth`：一个 seed cell 最多局部二分层数。
- `max_cases`：seed、corner 和 midpoint 去重后的真实求解点总上限。
- `min_spacing`：二分后允许的最小轴间距。

PASS cell 接受；WARN/FAIL cell 继续细分。达到任一限制仍有 WARN 时整体 WARN，仍有 FAIL 时整体 FAIL，绝不会因达到上限自动 PASS。所有实际点都复用既有 `_grid_case` 求解、Wake schedule、坐标映射、签名和缓存。

## 3. p/q/r 导数如何获得

23 项 required derivatives 使用两类可信方法：

- 15 项 alpha、beta、aileron、elevator、rudder 导数：真实正负扰动的 `centered_finite_difference`；
- 8 项 `CL_q, Cm_q, CY_p, Cl_p, Cn_p, CY_r, Cl_r, Cn_r`：OpenVSP steady stability `.stab` 的 `vspaero_steady_rate_derivative`。

OpenVSP 3.51.3 求解器对 steady `.stab` 中 p/q/r 的分母分别使用：

```text
p_hat = p * bref / (2V)
q_hat = q * cref / (2V)
r_hat = r * bref / (2V)
```

因此这些 8 项直接进入唯一权威的 `production_derivatives`，不会仅因方法不同而 WARN。用户不需要手工运行 P/Q/R Analysis。

P/Q/R unsteady analysis 是附加诊断；Q/R 可输出 `q+alpha_dot`、`r-beta_dot` 等组合量，它们只进入 `unsteady_derivative_diagnostics`，不会覆盖经典 `Cm_q`、`Cn_r`。当前模型在 OpenVSP 3.51.3 默认自动时步下实测为 128 个时间步，成本较高，故默认 `solver.run_unsteady_diagnostics: false`；确需诊断时可显式开启。

## 4. 理解三个 Gate

`results/numerical_convergence/production_numerical_settings.yaml` 分别保存：

- `solver_gate`：当前 model/OpenVSP/config identity、基础系数 Wake 收敛、Wake schedule 和边界连续性；
- `derivative_gate`：导数 Wake 与 FD step 收敛、23 项 source/method/unit/finite 完整性和导数验证；
- `production_gate`：前两者组合，控制最终数据库/AUTOTUNE 交付。

规则如下：

- Adaptive GRID 只要求 `solver_gate == PASS`；即使 derivative gate 为 WARN，仍可做自适应求解，但最终 production dataset 保持 WARN。
- `solver_gate` 为 WARN 或 FAIL 时禁止 Adaptive GRID，`--force` 不能绕过。
- Uniform GRID/TRIM 继续接受 PASS/WARN；production FAIL 默认阻止，显式 `--force` 只留下审计记录，不改 Gate 的真实状态。

导数状态仅有 `PASS`、`WARN_NUMERICAL`、`FAIL`。缺失、非有限、单位或映射错误必须 FAIL。

## 5. 运行

```bat
run_aero.bat numerical-convergence
run_aero.bat all
```

也可分别运行：

```bat
run_aero.bat grid
run_aero.bat trim
```

TRIM 最多 15 次迭代；仅当力残差不超过 ±1 N 且俯仰力矩残差不超过 ±1 N·m 才成功。Jacobian 使用真实 alpha/elevator 中心差分与回溯，不使用 native derivative 替代。

## 6. 更换飞机时修改

主要修改 `config/aircraft.yaml`：

| 配置 | 用途 |
|---|---|
| `aircraft.model` | 新 `.vsp3` 路径 |
| `geometry_sets` | 薄面与厚体集合 |
| `reference` | Sref/bref/cref 与重心来源 |
| `controls` | 三组舵面名称、中立位与限位 |
| `operating_conditions` | uniform 网格或 adaptive seed |
| `grid.adaptive` | 自适应量、容差和停止限制 |
| `trim` | 质量、搜索范围和残差门限 |
| `numerical_convergence.wake` | Wake 代表点、候选和容差 |
| `derivatives` | FD 候选步长和稳定性容差 |

`config/required_derivatives.yaml` 是 23 项清单的唯一权威定义。不要为取得 PASS 而删除 required 项、改符号或放宽容差。

## 7. 输出位置

| 路径 | 内容 |
|---|---|
| `results/latest/aero_database.json` | 完整 GRID/TRIM、23 项导数、Gate 和 adaptive cell/history |
| `results/latest/aero_database.csv` | 一行一个真实 seed/adaptive/TRIM 点 |
| `results/latest/trim_derivatives.csv` | 23 项 value/source/source_field/method/unit/status/wake |
| `results/autotune/aircraft_aero.mat` | `AERO.longitudinal/lateral/controls`，包含 p/q/r rate derivatives |
| `results/adaptive_grid/adaptive_grid_report.json` | 自适应汇总、最终 cells 与 refinement history |
| `results/adaptive_grid/adaptive_grid_points.csv` | 全部去重后的真实 GRID 点 |
| `results/adaptive_grid/adaptive_grid_cells.csv` | 最终 cell bounds/depth/status/termination |
| `results/numerical_convergence/` | 三层 Gate、Wake/FD 报告与缓存 |
| `results/validation/` | 数据验证与机身参与检查 |

GRID 的 `grid_source` 为 `seed` 或 `adaptive_midpoint`。自适应点集天然不规则，不会强制转换为规则矩阵。

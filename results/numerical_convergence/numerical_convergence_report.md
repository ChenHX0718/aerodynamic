# Numerical Convergence / Validation Report

Generated: 2026-09-04T21:36:26+08:00

Solver / GRID gate: **PASS**
Derivative gate: **WARN**
Production gate: **WARN**

Tessellation / mesh quality is user responsibility and is not numerically certified by this workflow.

The workflow uses the mesh currently saved in the OpenVSP model and only checks that required VSPAERO cases and outputs are valid.

## Wake convergence map

| State | V (m/s) | alpha (deg) | beta (deg) | Production Wake | Base | FD derivatives | Wake 16 verification | Native diagnostic | Status | Why |
|---|---:|---:|---:|---:|---|---|---|---|---|---|
| linear_low_alpha | 8 | 0 | 0 | 12 | PASS | PASS | NOT_NEEDED | DIAGNOSTIC_ONLY | PASS | boundary continuity required upgrade from Wake 3 to Wake 12 |
| cruise_trim | 8 | 4.03401 | 0 | 12 | PASS | PASS | NOT_NEEDED | DIAGNOSTIC_ONLY | PASS | boundary continuity required upgrade from Wake 3 to Wake 12 |
| medium_alpha | 9 | 6 | 0 | 12 | PASS | PASS | NOT_NEEDED | DIAGNOSTIC_ONLY | PASS | boundary continuity required upgrade from Wake 3 to Wake 12 |
| high_alpha_beta | 9 | 10 | 2 | 12 | PASS | WARN | FAIL | DIAGNOSTIC_ONLY | WARN | Wake 12->16 remains outside PASS tolerance; production Wake remains 12 with a numerical warning |

Wake 16 is verification-only. It is run only when the 8->12 transition is not PASS, never enters the production candidate list, and never changes a production Wake above 12.

## FD step selection

| Derivative | Selected step (deg) | Status | Value | Method |
|---|---:|---|---:|---|
| CL_alpha | 0.5 | PASS | 4.70066 | centered_finite_difference |
| CD_alpha | 0.5 | PASS | 0.278141 | centered_finite_difference |
| Cm_alpha | 0.5 | PASS | -1.12786 | centered_finite_difference |
| CY_beta | 0.5 | PASS | -0.35414 | centered_finite_difference |
| Cl_beta | 0.5 | PASS | -0.101309 | centered_finite_difference |
| Cn_beta | 0.5 | PASS | 0.0424109 | centered_finite_difference |
| CY_delta_a | 1 | PASS | -0.0144281 | centered_finite_difference |
| Cl_delta_a | 1 | PASS | -0.279475 | centered_finite_difference |
| Cn_delta_a | 1 | PASS | 0.00861439 | centered_finite_difference |
| CL_delta_e | 1 | PASS | 0.503424 | centered_finite_difference |
| CD_delta_e | 1 | PASS | 0.00655276 | centered_finite_difference |
| Cm_delta_e | 1 | PASS | -1.42531 | centered_finite_difference |
| CY_delta_r | 1 | PASS | -0.157917 | centered_finite_difference |
| Cl_delta_r | 1 | PASS | 0.00428823 | centered_finite_difference |
| Cn_delta_r | 1 | PASS | 0.0674172 | centered_finite_difference |

Alpha, beta, and control derivatives select their own centered-FD steps. Classical p/q/r derivatives come from the steady VSPAERO stability table using p_hat/q_hat/r_hat denominators. P/Q/R unsteady damping outputs remain separate diagnostics.

## Required derivatives manifest

Required: **23**; PASS: **23**; WARN_NUMERICAL: **0**; FAIL: **0**.

## Boundary continuity

Status: **WARN**. Checks performed: 3.

## Cache / resume

Enabled: **True**; cache hits: **295**; new solver runs: **0**; failed real cases: **0**; solver time: **0.0 s**; wall time: **20.0 s**.

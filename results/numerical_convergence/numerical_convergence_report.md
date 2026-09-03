# Numerical Convergence / Validation Report

Generated: 2026-09-03T16:47:10+08:00

Production gate: **FAIL (incomplete targeted scope)**

Tessellation / mesh quality is user responsibility and is not numerically certified by this workflow.

The solver used the mesh currently saved in the OpenVSP model. No script-side mesh override or mesh-convergence gate was applied.

## Verification scope

This was a real, targeted affected-path verification at medium_alpha (V=9 m/s, alpha=6 deg, beta=0 deg), not the full representative-state convergence matrix or the full production database.

Real cached cases: **43**; failed cases: **0**; summed solver time: **6203.1 s**. The separate real smoke test is **PASS** (324.0 s).

Because the configured representative-state set and boundary checks were intentionally not run, no production Wake schedule was generated and the production gate remains FAIL. This is an incompleteness result, not a fabricated numerical failure.

## Wake verification

Production candidates are 3, 5, 8, and 12. Wake 16 is verification-only and the production implementation invokes it only when the 8->12 transition is not PASS.

For this targeted test, the 12->16 branch was exercised directly without evaluating Wake 8. The six base coefficients are all PASS; the aggregate status of the 15 required production centered-FD derivatives is WARN (13 PASS, 2 WARN). Production Wake remains 12.

| Quantity | Wake 12 | Wake 16 | Status |
|---|---:|---:|---|
| CL | 0.606044868 | 0.605791904 | PASS |
| CD | 0.0425207211 | 0.0424617001 | PASS |
| CY | 0.00395872447 | 0.0046137886 | PASS |
| Cl | -0.000751092592 | -0.000809039854 | PASS |
| Cm | -0.154363768 | -0.154099578 | PASS |
| Cn | -0.000441416808 | -0.000550066401 | PASS |
| CL_alpha | 4.69883435 | 4.75097062 | PASS |
| CD_alpha | 0.430432713 | 0.43959433 | PASS |
| Cm_alpha | -1.14961664 | -1.17565284 | PASS |
| CL_delta_e | 0.479440744 | 0.485587144 | PASS |
| CD_delta_e | 0.0430071107 | 0.0452742097 | PASS |
| Cm_delta_e | -1.34823952 | -1.35465134 | PASS |
| CY_beta | -0.346936734 | -0.374627 | WARN |
| Cl_beta | -0.120375517 | -0.117030723 | PASS |
| Cn_beta | 0.0412013795 | 0.0460783288 | PASS |
| CY_delta_a | -0.0129013913 | 0.00371521207 | WARN |
| Cl_delta_a | -0.288788662 | -0.293383487 | PASS |
| Cn_delta_a | 0.00941427177 | 0.00816519025 | PASS |
| CY_delta_r | -0.158260001 | -0.150662177 | PASS |
| Cl_delta_r | 0.00372398339 | 0.011948838 | PASS |
| Cn_delta_r | 0.0687303566 | 0.0689082037 | PASS |

## FD step selection

| Derivative | Selected step (deg) | Status | Value | Method |
|---|---:|---|---:|---|
| CL_alpha | 0.5 | PASS | 4.69883435 | centered_finite_difference |
| CD_alpha | 0.5 | PASS | 0.430432713 | centered_finite_difference |
| Cm_alpha | 0.5 | PASS | -1.14961664 | centered_finite_difference |
| CY_beta | 0.5 | PASS | -0.346936734 | centered_finite_difference |
| Cl_beta | 0.5 | PASS | -0.120375517 | centered_finite_difference |
| Cn_beta | 0.5 | PASS | 0.0412013795 | centered_finite_difference |
| CY_delta_a | 1 | PASS | -0.0129013913 | centered_finite_difference |
| Cl_delta_a | 1 | PASS | -0.288788662 | centered_finite_difference |
| Cn_delta_a | 1 | PASS | 0.00941427177 | centered_finite_difference |
| CL_delta_e | 1 | PASS | 0.479440744 | centered_finite_difference |
| CD_delta_e | 1 | PASS | 0.0430071107 | centered_finite_difference |
| Cm_delta_e | 1 | PASS | -1.34823952 | centered_finite_difference |
| CY_delta_r | 1 | PASS | -0.158260001 | centered_finite_difference |
| Cl_delta_r | 1 | PASS | 0.00372398339 | centered_finite_difference |
| Cn_delta_r | 1 | PASS | 0.0687303566 | centered_finite_difference |

Each derivative independently selects a centered-FD step. Production data, native diagnostics, and the required-derivative manifest are separate fields and files.

## Required derivatives manifest

Required: **23**; PASS: **15**; WARN_NUMERICAL: **0**; METHOD_LIMITATION: **8**; FAIL: **0**.

PASS is accepted, WARN_NUMERICAL and METHOD_LIMITATION are accepted with warning, and FAIL is rejected. Native VSPAERO derivatives are diagnostic-only: they do not enter the Wake gate, production derivative set, TRIM Jacobian, or overwrite centered-FD values.

Cm_q is METHOD_LIMITATION because OpenVSP/VSPAERO 3.51.3 does not expose a true negative steady-q input; its native value is retained only as a diagnostic reference.

## Cache / resume

A resume probe reused **22** cases with **0** misses. Stored case counts by Wake: {'12': 32, '16': 11}.

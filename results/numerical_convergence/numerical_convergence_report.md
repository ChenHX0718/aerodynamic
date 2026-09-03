# Numerical Convergence Report

Generated: 2026-09-03T00:36:07+08:00

Production gate: **FAIL**

## Wake convergence map

| State | V (m/s) | alpha (deg) | beta (deg) | Required Wake | Base | FD derivatives | Native diagnostic | Status | Why |
|---|---:|---:|---:|---:|---|---|---|---|---|
| linear_low_alpha | 8 | 0 | 0 | 3 | PASS | PASS | WARN | PASS | all subsequent higher-level transitions satisfy PASS tolerance |
| cruise_trim | 8 | 4.07757 | 0 | 12 | PASS | WARN | WARN | WARN | highest level is selected conservatively but remains unverified |
| medium_alpha | 9 | 6 | 0 | 12 | PASS | WARN | WARN | WARN | highest level is selected conservatively but remains unverified |
| high_alpha_beta | 9 | 10 | 2 | 12 | PASS | WARN | WARN | WARN | highest level is selected conservatively but remains unverified |

The map is generated from solver comparisons at the representative states. Untested states use a conservative discrete nearest-region lookup, a boundary buffer, and a configured one-level safety margin; no alpha threshold or linear Wake interpolation is used.

## Boundary continuity

Status: **PASS**. Checks performed: 3. Any discontinuous low-Wake endpoint is automatically upgraded in the final map.

## Tessellation

Recommended preset: **FINE** (FAIL). one or more tessellation cases failed, were dependency-skipped, or FINE remains unverified

## CY_delta_r diagnostic

Status: **WARN**; classification: **LARGE_DEFLECTION_NONLINEAR**; recommended delta_r: 1.0 deg. the local small-step range is stable but the 2-to-4 degree transition is not The extra rudder points are diagnostic-only and are not expanded over the GRID.

## Cm_q diagnostic

Status: **WARN**; numerical sensitivity: **FAIL**. OpenVSP 3.51.3 does not expose a negative steady q input; Cm_q remains a native positive-rate derivative and cannot be represented as a fabricated centered difference.

## Production numerical settings

Uniform tessellation: **FINE**. Wake is selected per flight state, then promoted to the maximum required by the complete derivative bundle. TRIM uses a low-cost pre-trim followed by a production trim and only upgrades Wake between complete trim solves.

Final convergence status: **FAIL**; production gate: **FAIL**.

## Cache / resume

Enabled: **True**; hits: **272**; misses: **0**; failed real cases: **0**; wall time: **2.0 s**.

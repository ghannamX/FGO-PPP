بِسْمِ اللَّهِ الرَّحْمَنِ الرَّحِيمِ

# SPP Sprint — Road to First Position Fix

Goal: compute a single-point position from GPS pseudoranges using weighted least squares.
Method: iterate the linearised pseudorange equation until position converges (< 1 mm).

---

## Sprint Tasks

### S1 — Time System Conversions `gnss/time`

Already have: `DateTime` (calendar epoch), `GnssTime` (week + ToW).
Need: round-trip conversions and a GPS → UTC offset lookup.

**Key conversions:**

```
GPS week + ToW  →  DateTime  (calendar epoch)
DateTime        →  GPS week + ToW
GPST            →  UTC  (subtract leap seconds)
```

**Files:**
- `include/time/GnssTime.hpp` — add `toGpsWeekTow()`, `fromGpsWeekTow()`, `gpstToUtc()`
- `src/gnss/time/GnssTime.cpp`

---

### S2 — Coordinate Frame Transforms `gnss/geodesy`

**ECEF → LLA (iterative Bowring or closed-form Zhu):**

```
p   = sqrt(X² + Y²)
lon = atan2(Y, X)

Iterative for lat and alt:
  lat₀ = atan2(Z, p · (1 − e²))
  repeat:
    N   = a / sqrt(1 − e² sin²(lat))
    lat = atan2(Z + e² N sin(lat),  p)
  alt = p / cos(lat) − N
```

**ECEF → ENU (at reference point r₀):**

```
δ = r_sat − r_rec  (ECEF difference)

| E |   | −sin(lon)            cos(lon)           0       | | δX |
| N | = | −sin(lat)cos(lon)   −sin(lat)sin(lon)  cos(lat) | | δY |
| U |   |  cos(lat)cos(lon)    cos(lat)sin(lon)  sin(lat) | | δZ |
```

**Elevation and azimuth:**

```
el  = atan2(U,  sqrt(E² + N²))
az  = atan2(E,  N)              (north-clockwise)
```

**Files:**
- `include/geodesy/CoordTransforms.hpp`
- `src/gnss/geodesy/CoordTransforms.cpp`
- `src/gnss/geodesy/CMakeLists.txt`

---

### S3 — Transmission Time Iteration `satellite`

Signal travel time must be solved iteratively because satellite position depends on
the transmission epoch, which in turn depends on the range.

```
Algorithm:
  t_rx      ← reception epoch (from obs epoch)
  τ         ← P / c              (first guess, ~0.07 s)
  repeat:
    t_tx    = t_rx − τ
    r_sat   = computeOrbit(eph, t_tx)           // Keplerian propagation
    r_sat   = applySagnac(r_sat, τ)             // Earth rotation correction
    ρ       = ||r_sat − r_rec||
    τ_new   = (ρ + Δt_sv · c) / c              // Δt_sv = clock correction
    δ       = |τ_new − τ|
    τ       ← τ_new
  until δ < 1e-12 s   (< 0.3 mm)
```

**Files:**
- `include/satellite/TransmissionTimeSolver.hpp`
- `src/gnss/satellite/TransmissionTimeSolver.cpp`

---

### S4 — Tropospheric Delay `corrections`

**Saastamoinen zenith delay:**

```
Zh_dry = (0.0022768 · P) / (1 − 0.00266 cos(2φ) − 0.00028 H)

where:
  P   = surface pressure [hPa]     (use standard: 1013.25 hPa)
  φ   = geodetic latitude [rad]
  H   = ellipsoidal height [km]

Zh_wet ≈ 0.002277 · (1255/T + 0.05) · e     (often use 0.1 m as default)
```

**Elevation mapping function (Niell / simple 1/sin):**

```
mf(el) = 1 / sin(el)     (simple, sufficient for SPP)

ΔT_trop = mf(el) · (Zh_dry + Zh_wet)
```

**Files:**
- `include/corrections/TroposphericModel.hpp`
- `src/gnss/corrections/TroposphericModel.cpp`
- `src/gnss/corrections/CMakeLists.txt`

---

### S5 — Ionospheric Delay `corrections`

**Klobuchar model** (GPS ICD, single-frequency L1):

```
φ_m   = geographic latitude of ionospheric pierce point [semi-circles]

PER   = α₀ + α₁φ_m + α₂φ_m² + α₃φ_m³    (period)
AMP   = β₀ + β₁φ_m + β₂φ_m² + β₃φ_m³    (amplitude)

x     = 2π(t − 50400) / PER              (t = local time at IPP [s])

        ⎧ 5·10⁻⁹ + AMP · (1 − x²/2 + x⁴/24)    |x| < π/2
T_iono = ⎨
        ⎩ 5·10⁻⁹                                  otherwise

ΔI_L1 = F · T_iono · c                   (F = obliquity factor ≈ 1 + 16(0.53 − el/π)³)
```

**Files:**
- `include/corrections/KlobucharModel.hpp`
- `src/gnss/corrections/KlobucharModel.cpp`

---

### S6 — SPP Observation Model and WLS Solver `solution`

**Pseudorange observation equation:**

```
P_i = ρ_i + c·δt_r − c·δt_s_i + ΔT_trop_i + ΔI_L1_i + ε_i

where:
  ρ_i       = geometric range to satellite i            [m]
  δt_r      = receiver clock offset                     [s]
  δt_s_i    = satellite clock correction (from S3)      [s]
  ΔT_trop_i = tropospheric delay (from S4)              [m]
  ΔI_L1_i   = ionospheric delay (from S5)               [m]
```

**Corrected pseudorange (reduced observation):**

```
P̃_i = P_i + c·δt_s_i − ΔT_trop_i − ΔI_L1_i
     = ρ_i + c·δt_r + ε_i
```

**Linearisation around approximate position x₀ = (X₀, Y₀, Z₀):**

```
ρ_i ≈ ρ₀_i + (∂ρ/∂x)·Δx + (∂ρ/∂y)·Δy + (∂ρ/∂z)·Δz

unit vector from receiver to satellite i:
  e_i = (r_sat_i − x₀) / ρ₀_i

design matrix row (4 unknowns: Δx, Δy, Δz, c·δt_r):
  h_i = [−e_iX, −e_iY, −e_iZ, 1]

innovation:
  v_i = P̃_i − ρ₀_i − c·δt_r⁰
```

**System (n satellites):**

```
v = H·Δx − w

H  ∈ ℝⁿˣ⁴    (design matrix)
Δx ∈ ℝ⁴      (corrections: δX, δY, δZ, c·δt_r)
w  ∈ ℝⁿ      (innovation vector)
```

**Elevation-dependent weight matrix:**

```
σ²_i = σ₀² / sin²(el_i)     (elevation weighting)

W = diag(1/σ²_i)
```

**Weighted least squares solution:**

```
Δx = (Hᵀ W H)⁻¹ Hᵀ W w

Update:
  x₀    ← x₀ + Δx[0:3]
  δt_r  ← δt_r + Δx[3] / c

Convergence: ||Δx[0:3]|| < 0.001 m  (iterate S3–S6)
```

**DOP values (from (HᵀH)⁻¹ = Q):**

```
PDOP = sqrt(Q_XX + Q_YY + Q_ZZ)
TDOP = sqrt(Q_tt)
GDOP = sqrt(PDOP² + TDOP²)
```

**Files:**
- `include/solution/SppSolver.hpp`
- `src/gnss/solution/SppSolver.cpp`
- `src/gnss/solution/CMakeLists.txt`
- `src/apps/spp/main.cpp`
- `src/apps/CMakeLists.txt` (add spp target)

---

### S7 — SPP App and Validation `apps`

App: `spp` — reads obs + nav, iterates WLS per epoch, prints position + DOP.

**Validation strategy:**
- Compare output ECEF against TLSE known coordinates (from RINEX header: `4627854.0  119640.0  4372993.0`)
- Expect horizontal error < 5 m for single-epoch SPP (no SA)
- Print residuals per satellite for sanity check

---

## Completion Criteria

| Check | Target |
|-------|--------|
| Convergence | `||Δpos|| < 1 mm` within 10 iterations |
| Horizontal error vs. known coord | < 5 m (single epoch, no corrections tuning) |
| Satellites used | ≥ 4 GPS (elevation mask 10°) |
| All corrections applied | Tropo (Saas.) + Iono (Klobuchar) + Sagnac + Sat. clock |

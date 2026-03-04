 ### Bismillah 
 *"And He it is Who made the stars for you that you might follow the right direction thereby in the darkness of the land and the sea."* — Quran 6:97
# FGO-PPP — Factor Graph Optimization for Precise Point Positioning

A high-accuracy GNSS positioning engine implementing **Precise Point Positioning (PPP)** and **PPP-RTK** through **Factor Graph Optimization (FGO)**, with optional tight-coupling to inertial and other sensors.

---

## Project Goals

| Goal | Method |
|------|--------|
| Centimeter-level positioning | PPP / PPP-RTK with integer ambiguity resolution |
| Robust state estimation | Factor Graph Optimization (iSAM2 / sliding-window) |
| Multi-constellation support | GPS, GLONASS, Galileo, BeiDou, QZSS |
| Sensor fusion ready | IMU tight-coupling via pre-integration factors |
| Real-time & post-processing | Dual-mode pipeline |

---

## Technology Stack

| Layer | Library / Tool |
|-------|---------------|
| Language | C++20 |
| Build system | CMake ≥ 3.25 |
| Package manager | Conan 2 |
| Linear algebra | Eigen 3 |
| Factor graph | GTSAM 4 (or custom, TBD) |
| Unit testing | Google Test |
| Logging | spdlog |
| Utilities | fmt, Boost (optional) |

---

## High-Level Architecture

```
┌──────────────────────────────────────────────────────────────────────┐
│                          FGO-PPP Engine                              │
│                                                                      │
│  ┌─────────────┐   ┌──────────────┐   ┌──────────────────────────┐  │
│  │  Data Layer │──▶│  Processing  │──▶│  Factor Graph Optimizer  │  │
│  │  (Parsers)  │   │  Pipeline    │   │  (State Estimator)       │  │
│  └─────────────┘   └──────────────┘   └──────────────────────────┘  │
│         │                 │                        │                 │
│         ▼                 ▼                        ▼                 │
│  ┌─────────────┐   ┌──────────────┐   ┌──────────────────────────┐  │
│  │  Corrections│   │  Observation │   │  Output / Results        │  │
│  │  & Models   │   │  Models      │   │  (Position, Cov, NMEA)   │  │
│  └─────────────┘   └──────────────┘   └──────────────────────────┘  │
└──────────────────────────────────────────────────────────────────────┘
```

---

## Module Breakdown

### 1. `gnss/input` — Data Ingestion & Parsers

Responsible for reading all standard GNSS data formats.

| Sub-module | Description |
|------------|-------------|
| `rinex/ObsParser` | RINEX 2/3/4 observation files — pseudorange, phase, Doppler, SNR per signal |
| `rinex/NavParser` | Broadcast navigation messages — GPS LNAV/CNAV, Galileo FNAV/INAV, BDS, GLO |
| `rinex/ClockParser` | RINEX clock files (`.clk`) — precise satellite & receiver clock products |
| `sp3/Sp3Parser` | SP3-c/d precise orbit files — satellite positions & velocities |
| `antex/AntexParser` | ANTEX antenna phase center offsets & variations (PCO/PCV) |
| `ionex/IonexParser` | Global Ionosphere Maps (GIM) in IONEX format |
| `dcb/DcbParser` | Differential Code Biases (CODE, CAS products) |
| `ssr/SsrDecoder` | State Space Representation corrections (RTCM 3 SSR, CLAS, SPARTN) |
| `stream/RtcmReader` | Live RTCM 3.x stream reader for real-time operation |

---

### 2. `gnss/time` — Time Systems & Reference Frames

| Component | Description |
|-----------|-------------|
| `GnssTime` | GPST, GLONASST, GST, BDT, UTC — conversion utilities |
| `LeapSecond` | Leap second table management |
| `CoordFrame` | ECEF ↔ ENU ↔ LLA ↔ NED transformations |
| `Itrf` | ITRF2020 / WGS-84 reference frame handling |

---

### 3. `gnss/satellite` — Satellite State Computation

Computes satellite positions, velocities, and clock offsets at signal transmission time.

| Component | Description |
|-----------|-------------|
| `OrbitInterpolator` | Lagrange polynomial interpolation of SP3 orbits |
| `BroadcastOrbit` | Keplerian orbit propagation from broadcast ephemeris (GPS, Gal, BDS) |
| `GloOrbitIntegrator` | GLONASS PZ-90 numerical orbit integration (RK4) |
| `SatClock` | Satellite clock correction — broadcast polynomial & precise products |
| `SatelliteSelector` | Elevation mask, PDOP thresholding, signal health check |

---

### 4. `gnss/corrections` — Error Modeling & Corrections

Each correction is a standalone, swappable component.

| Component | Model(s) |
|-----------|----------|
| `Troposphere` | Saastamoinen, Hopfield, VMF3, GPT3 + mapping functions (GMF, VMF) |
| `Ionosphere` | Dual-frequency IF combination, Klobuchar, IONEX bicubic interpolation, STEC |
| `AntennaPCC` | PCO/PCV correction from ANTEX for satellite & receiver antennas |
| `PhaseWindup` | Wu et al. carrier phase wind-up correction |
| `SolidEarthTide` | IERS Conventions 2010 solid Earth tide model |
| `OceanLoading` | Ocean tide loading (BLQ coefficients) |
| `RelativisticEffect` | Shapiro delay + satellite clock relativistic correction |
| `SagnacEffect` | Earth rotation correction (ECEF signal travel time) |
| `PhaseCenter` | Receiver antenna reference point to APC conversion |

---

### 5. `gnss/preprocessing` — Signal Quality & Conditioning

Runs before observation modeling.

| Component | Description |
|-----------|-------------|
| `CycleSlipDetector` | MW (Melbourne-Wübbena) + GF (Geometry-Free) dual-method detection |
| `OutlierRejector` | Innovation-based chi-squared test, DIA procedure |
| `HatchFilter` | Pseudorange smoothing with carrier phase (Hatch filter) |
| `MultipathEstimator` | Code-minus-carrier (CMC) multipath monitoring |
| `SignalSelector` | Best signal selection strategy per constellation/band |
| `ElevationWeighting` | Sine-squared, SNR-based stochastic model |

---

### 6. `gnss/observation` — GNSS Measurement Models

Linearized observation equations assembled into factor residuals.

| Component | Description |
|-----------|-------------|
| `ObservationModel` | Full GNSS observation equation: ρ = r + c·δt_r − c·δt_s + T + I + B + ε |
| `PseudorangeModel` | Code observation with all corrections applied |
| `CarrierPhaseModel` | Phase observation with integer ambiguity state N |
| `DopplerModel` | Velocity estimation from Doppler shift |
| `IFCombination` | Ionosphere-free linear combination (L1/L2, L1/L5, etc.) |
| `MWCombination` | Melbourne-Wübbena wide-lane combination |
| `GFCombination` | Geometry-free combination for ionosphere/cycle-slip monitoring |
| `UCModel` | Uncombined (raw) model for PPP-AR and multi-frequency PPP |

---

### 7. `gnss/ambiguity` — Integer Ambiguity Resolution

The heart of PPP-RTK precise positioning.

| Component | Description |
|-----------|-------------|
| `FloatAmbiguity` | Float ambiguity estimation within FGO (continuous states) |
| `LambdaSolver` | LAMBDA / MLAMBDA integer least-squares search |
| `PartialAmbFixer` | Subset selection for partial ambiguity fixing (elevation, variance criteria) |
| `RatioTest` | Ratio test & bootstrapping success-rate validation |
| `WideLaneNarrowLane` | Cascaded WL/NL fixing strategy (IRC or OSR approach) |
| `PhaseBiasCorrector` | Fractional Cycle Bias (FCB) / Satellite Phase Bias (SPB) correction |
| `AmbiguityPool` | Ambiguity state management across epochs (add, fix, remove) |

**Ambiguity Resolution Strategies:**

```
PPP (float only)
  └─▶ Ambiguity state estimated as real-valued
PPP-AR (with FCB/UPD products)
  └─▶ Wide-lane fix → Narrow-lane fix (cascaded LAMBDA)
PPP-RTK (with SSR phase biases)
  └─▶ Single-epoch fixing with network-augmented corrections
RTK (baseline)
  └─▶ Double-difference ambiguity fixing (short baseline)
```

---

### 8. `fgo` — Factor Graph Optimization Engine

The core estimation framework. Each measurement type maps to a **factor** with a residual function and Jacobian.

#### State Vector per Epoch

```
X = [ position (3), velocity (3), receiver_clock_offset (n_sys),
      receiver_clock_drift (1), troposphere_ZTD (1),
      ambiguities (n_sat × n_freq), IMU_bias (6, if fused) ]
```

#### Factor Types

| Factor | Residual | Connected States |
|--------|----------|-----------------|
| `PseudorangeFactor` | Code observation residual | pos, clk, ZTD |
| `CarrierPhaseFactor` | Phase observation residual | pos, clk, ZTD, N |
| `DopplerFactor` | Velocity from Doppler | vel, clk_drift |
| `TroposphereFactor` | ZTD random-walk prior | ZTD_prev → ZTD_curr |
| `ClockFactor` | Clock model (random walk / polynomial) | clk_prev → clk_curr |
| `AmbiguityFactor` | Ambiguity continuity (no cycle slip) | N_prev → N_curr |
| `IMUPreintegrationFactor` | IMU delta-pose residual | pos, vel, att, bias |
| `PriorFactor` | Initial state / anchor | any state |
| `BetweenFactor` | Relative constraint | any two states |
| `FixedAmbiguityFactor` | Integer-fixed hard constraint | N |

#### Optimizer Options

| Mode | Method | Use Case |
|------|--------|----------|
| Batch | Levenberg-Marquardt / Dogleg | Post-processing |
| Incremental | iSAM2 (Bayes tree) | Real-time |
| Sliding window | Fixed-lag smoother | Real-time (bounded latency) |

---

### 9. `sensor_fusion` — Multi-Sensor Tight Coupling

| Component | Description |
|-----------|-------------|
| `ImuPreintegrator` | IMU delta-pose on manifold (Forster et al. 2017) |
| `ImuBiasModel` | Accelerometer & gyroscope bias random walk |
| `GnssFgoFusion` | GNSS-IMU tight coupling factor assembly |
| `BarometerFactor` | Barometric altitude constraint |
| `MagnetometerFactor` | Magnetometer heading constraint |

---

### 10. `gnss/solution` — Output & Quality Control

| Component | Description |
|-----------|-------------|
| `PositionSolution` | ECEF/ENU/LLA position + full covariance matrix |
| `QualityIndicator` | PDOP, HDOP, VDOP, number of satellites, fix status |
| `NmeaSerializer` | GGA, RMC, GSA, GSV NMEA-0183 output |
| `ResultLogger` | CSV / binary result files with per-epoch statistics |
| `ResidualMonitor` | Post-fit residual analysis for quality assessment |

---

## Planned Positioning Modes

| Mode | Description | Accuracy |
|------|-------------|----------|
| SPP | Standard Point Positioning (broadcast ephemeris) | ~1–3 m |
| DGNSS | Differential code corrections | ~0.5–1 m |
| PPP (float) | Precise orbits/clocks, float ambiguity | ~5–10 cm |
| PPP-AR | PPP + integer ambiguity fixing (FCB products) | ~2–5 cm |
| PPP-RTK | PPP + network SSR augmentation | ~2–5 cm fast convergence |
| RTK | Short-baseline double-difference integer fixing | ~1–2 cm |
| GNSS/IMU | Tight-coupled GNSS + IMU via FGO | ~1–3 cm |

---

## Repository Layout (Planned)

```
FGO-PPP/
├── CMakeLists.txt
├── conanfile.py
├── README.md
│
├── src/
│   ├── gnss/
│   │   ├── input/         # Parsers (RINEX, SP3, ANTEX, IONEX, DCB, SSR)
│   │   ├── time/          # Time systems & coordinate frames
│   │   ├── satellite/     # Orbit & clock computation
│   │   ├── corrections/   # Atmospheric, hardware, geometric corrections
│   │   ├── preprocessing/ # Cycle slip, outlier, smoothing
│   │   ├── observation/   # Measurement models & combinations
│   │   ├── ambiguity/     # Float estimation & integer fixing
│   │   └── solution/      # Output, NMEA, quality indicators
│   │
│   ├── fgo/
│   │   ├── factors/       # All factor types (GNSS, IMU, prior, between)
│   │   ├── states/        # State variable definitions
│   │   ├── graph/         # Factor graph builder & optimizer wrapper
│   │   └── smoother/      # Sliding-window / iSAM2 interface
│   │
│   ├── sensorFusion/
│   │   ├── imu/           # Pre-integration, bias model
│   │   └── fusion/        # Multi-sensor graph assembly
│   │
│   └── apps/
│       ├── pppSolver/     # PPP/PPP-AR post-processing app
│       ├── rtkSolver/     # RTK processing app
│       └── rtReceiver/    # Real-time receiver pipeline
│
├── include/               # Public headers (mirrors src structure)
├── tests/                 # Google Test unit & integration tests
├── data/                  # Sample RINEX, SP3, test fixtures
├── scripts/               # Python analysis / plotting scripts
└── docs/                  # Detailed algorithm documentation
```

---

## Implementation Roadmap

### Phase 1 — Foundation
- [ ] CMake + Conan 2 project scaffold
- [ ] RINEX 3 observation parser
- [ ] RINEX navigation parser (GPS + Galileo)
- [ ] SP3 precise orbit parser
- [ ] Time system utilities (GPST, UTC, GLONASST)
- [ ] Coordinate frame transforms (ECEF ↔ LLA ↔ ENU)
- [ ] Broadcast orbit propagation (Keplerian)

### Phase 2 — SPP Baseline
- [ ] Satellite position & clock at transmission time
- [ ] Saastamoinen tropospheric model
- [ ] Klobuchar ionospheric model
- [ ] SPP solver (weighted least squares)
- [ ] Basic outlier rejection

### Phase 3 — PPP Float
- [ ] SP3 + RINEX clock precise products
- [ ] ANTEX PCO/PCV corrections
- [ ] Phase wind-up, solid Earth tide
- [ ] Uncombined PPP observation model
- [ ] Factor graph integration (GTSAM or Eigen-based)
- [ ] ZTD + receiver clock as states
- [ ] Float ambiguity states
- [ ] Cycle slip detection (MW + GF)

### Phase 4 — Ambiguity Resolution (PPP-AR)
- [ ] FCB/UPD product parser
- [ ] LAMBDA integer search engine
- [ ] Wide-lane / narrow-lane cascaded fixing
- [ ] Ratio test & partial fixing
- [ ] PPP-AR validation against IGS reference stations

### Phase 5 — RTK & PPP-RTK
- [ ] Double-difference observation model
- [ ] SSR decoder (RTCM 3 SSR messages)
- [ ] Network augmentation (OSR / SSR phase biases)
- [ ] PPP-RTK fast convergence

### Phase 6 — Sensor Fusion
- [ ] IMU pre-integration factor
- [ ] GNSS/IMU tight-coupled FGO
- [ ] Sliding-window smoother (iSAM2)
- [ ] Real-time pipeline

---

## Key References

- Teunissen, P.J.G. & Montenbruck, O. (Eds.) *Springer Handbook of Global Navigation Satellite Systems*, 2017
- Forster, C. et al. "IMU Preintegration on Manifold for Efficient Visual-Inertial Maximum-a-Posteriori Estimation", RSS 2015
- Kaplan, E. & Hegarty, C. *Understanding GPS/GNSS: Principles and Applications*, 3rd Ed.
- Dellaert, F. & Kaess, M. "Factor Graphs for Robot Perception", *Foundations and Trends in Robotics*, 2017
- Zumberge, J.F. et al. "Precise point positioning for the efficient and robust analysis of GPS data from large networks", *JGR*, 1997
- Ge, M. et al. "Resolution of GPS carrier-phase ambiguities in Precise Point Positioning with daily observations", *JoG*, 2008

---



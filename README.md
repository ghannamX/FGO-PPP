بِسْمِ اللَّهِ الرَّحْمَنِ الرَّحِيمِ

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
| Build system | CMake >= 3.25 |
| Package manager | Conan 2 |
| Linear algebra | Eigen 3 |
| Factor graph | GTSAM 4 (or custom, TBD) |
| Unit testing | Google Test |
| Logging | spdlog |
| Utilities | fmt |

---

## Current Status

### Completed

- **Project scaffold**: CMake + Conan 2 build system, Eigen3/spdlog/fmt/GTest integration
- **Common GNSS types** (`include/common/`):
  - `SatId` — constellation + PRN packed into uint16_t, O(1) hash
  - `ObsCode` — 3-char observation code packed into uint32_t, open-ended (no enum brittleness)
  - `ObsMeasurement` — value + LLI + signal strength per observation slot
  - `Constellation` — GPS, GLONASS, Galileo, BeiDou, QZSS, SBAS, NavIC
- **Time types** (`include/time/`):
  - `DateTime` — calendar epoch struct with full validation
- **RINEX 3 observation parser** (`include/input/rinex/`, `src/gnss/input/rinex/`):
  - Stateless parser for RINEX 3.03/3.04 observation files
  - Full header + epoch parsing, slot-indexed O(1) observation storage
  - Tested: TLSE RINEX 3.04 MIXED — 2880 epochs, 141 134 observations
- **RINEX 3 navigation parser** (`include/input/rinex/`, `src/gnss/input/rinex/`):
  - Stateless parser for RINEX 3.03/3.04 GPS navigation files
  - All 28 Keplerian + clock + metadata fields, Klobuchar iono coefficients
  - Tested: TLSE RINEX 3.04 GPS nav — 211 ephemeris records
- **Broadcast orbit & clock** (`include/satellite/`, `src/gnss/satellite/`):
  - `BroadcastOrbitComputer` — Keplerian propagation to ECEF + relativistic correction
  - `SatelliteClockCorrector` — af0/af1/af2 polynomial + relativistic correction
  - `EphemerisSelector` — selects closest ephemeris to a given epoch
  - `SagnacCorrection` — Earth rotation correction for signal travel time
  - Tested: TLSE first epoch, 13 GPS satellites — positions + clock corrections verified

### Next

See [roadmap/spp-sprint.md](roadmap/spp-sprint.md) for the active sprint to first SPP fix.
See [roadmap/phases.md](roadmap/phases.md) for the full project roadmap.

---

## High-Level Architecture

```
+----------------------------------------------------------------------+
|                          FGO-PPP Engine                              |
|                                                                      |
|  +-------------+   +--------------+   +--------------------------+  |
|  |  Data Layer |-->|  Processing  |-->|  Factor Graph Optimizer  |  |
|  |  (Parsers)  |   |  Pipeline    |   |  (State Estimator)       |  |
|  +-------------+   +--------------+   +--------------------------+  |
|         |                 |                        |                 |
|         v                 v                        v                 |
|  +-------------+   +--------------+   +--------------------------+  |
|  |  Corrections|   |  Observation |   |  Output / Results        |  |
|  |  & Models   |   |  Models      |   |  (Position, Cov, NMEA)   |  |
|  +-------------+   +--------------+   +--------------------------+  |
+----------------------------------------------------------------------+
```

---

## Repository Layout

```
FGO-PPP/
├── CMakeLists.txt
├── conanfile.py
├── .clang-format
├── README.md
│
├── include/
│   ├── common/            # SatId, ObsCode, ObsMeasurement, Constellation
│   ├── time/              # DateTime
│   ├── geodesy/           # CoordTransforms (ECEF/ENU/LLA)
│   ├── satellite/         # BroadcastOrbitComputer, EphemerisSelector, ...
│   ├── corrections/       # TroposphericModel, KlobucharModel (planned)
│   ├── solution/          # SppSolver (planned)
│   └── input/
│       └── rinex/         # rinexObsTypes, rinexNavTypes, parsers, ParseResult
│
├── src/
│   └── gnss/
│       ├── input/rinex/   # RinexObsParser.cpp, RinexNavParser.cpp
│       ├── time/          # GnssTime.cpp
│       ├── satellite/     # BroadcastOrbitComputer, EphemerisSelector, ...
│       ├── geodesy/       # CoordTransforms.cpp (planned)
│       ├── corrections/   # TroposphericModel, KlobucharModel (planned)
│       └── solution/      # SppSolver.cpp (planned)
│
├── apps/
│   ├── rinex_reader/      # RINEX 3 observation file reader
│   ├── nav_reader/        # RINEX 3 navigation file reader
│   ├── satellite_position/# ECEF position + clock correction per epoch
│   └── spp/               # SPP weighted least squares (planned)
│
├── roadmap/
│   ├── phases.md          # Full phase 1–6 roadmap
│   └── spp-sprint.md      # Active sprint: first SPP fix
│
├── docs/                  # Algorithm documentation with equations
├── tests/                 # Google Test unit & integration tests
└── data/
    └── rinex/
        ├── obs/           # RINEX 3 observation files
        └── nav/           # RINEX 3 navigation files
```

---

## Building

```bash
# Install dependencies
conan install . --output-folder=build --build=missing

# Configure and build
cmake --preset conan-default
cmake --build build/build --config Release

# Run apps
./build/build/src/apps/Release/rinex_reader      <obs-file>
./build/build/src/apps/Release/nav_reader        <nav-file>
./build/build/src/apps/Release/satellite_position <obs-file> <nav-file>
```

---

## Key References

- Teunissen, P.J.G. & Montenbruck, O. (Eds.) *Springer Handbook of Global Navigation Satellite Systems*, 2017
- Forster, C. et al. "IMU Preintegration on Manifold for Efficient Visual-Inertial Maximum-a-Posteriori Estimation", RSS 2015
- Kaplan, E. & Hegarty, C. *Understanding GPS/GNSS: Principles and Applications*, 3rd Ed.
- Dellaert, F. & Kaess, M. "Factor Graphs for Robot Perception", *Foundations and Trends in Robotics*, 2017
- Zumberge, J.F. et al. "Precise point positioning for the efficient and robust analysis of GPS data from large networks", *JGR*, 1997
- Ge, M. et al. "Resolution of GPS carrier-phase ambiguities in Precise Point Positioning with daily observations", *JoG*, 2008

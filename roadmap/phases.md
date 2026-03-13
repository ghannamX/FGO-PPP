بِسْمِ اللَّهِ الرَّحْمَنِ الرَّحِيمِ

# FGO-PPP — Implementation Phases

Full project roadmap from foundation to sensor fusion.

---

## Phase 1 — Foundation

- [x] CMake + Conan 2 project scaffold
- [x] Common GNSS types (`SatId`, `ObsCode`, `ObsMeasurement`, `Constellation`)
- [x] `DateTime` epoch struct with full validation
- [x] RINEX 3 observation parser (3.03/3.04, multi-constellation)
- [x] RINEX 3 navigation parser (GPS broadcast ephemeris, Klobuchar iono)
- [x] Shared `ParseResult<T>` error handling
- [ ] Time system utilities (GPST ↔ UTC ↔ GST ↔ BDT conversions)
- [ ] Coordinate frame transforms (ECEF / LLA / ENU)
- [ ] SP3 precise orbit parser

---

## Phase 2 — SPP Baseline

- [x] Broadcast orbit propagation (Keplerian → ECEF)
- [x] Satellite clock correction (polynomial + relativistic)
- [x] Ephemeris selection by closest ToC
- [x] Sagnac / Earth rotation correction
- [ ] Transmission time iteration
- [ ] Saastamoinen tropospheric model
- [ ] Klobuchar ionospheric model
- [ ] Elevation / azimuth computation
- [ ] SPP weighted least squares solver
- [ ] Basic outlier rejection (elevation mask + residual screening)

---

## Phase 3 — PPP Float

- [ ] SP3 + RINEX clock precise products parser
- [ ] ANTEX PCO/PCV corrections
- [ ] Phase wind-up correction
- [ ] Solid Earth tide
- [ ] Uncombined PPP observation model
- [ ] Factor graph integration (GTSAM or Eigen-based)
- [ ] ZTD + receiver clock as states
- [ ] Float ambiguity states
- [ ] Cycle slip detection (MW + GF)

---

## Phase 4 — Ambiguity Resolution (PPP-AR)

- [ ] FCB / UPD product parser
- [ ] LAMBDA integer search engine
- [ ] Wide-lane / narrow-lane cascaded fixing
- [ ] Ratio test & partial fixing
- [ ] PPP-AR validation against IGS reference stations

---

## Phase 5 — RTK & PPP-RTK

- [ ] Double-difference observation model
- [ ] SSR decoder (RTCM 3 SSR messages)
- [ ] Network augmentation (OSR / SSR phase biases)
- [ ] PPP-RTK fast convergence

---

## Phase 6 — Sensor Fusion

- [ ] IMU pre-integration factor (Forster 2017)
- [ ] GNSS/IMU tight-coupled FGO
- [ ] Sliding-window smoother (iSAM2)
- [ ] Real-time pipeline

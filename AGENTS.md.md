# baro_driver_AGENTS.md

Plan for a custom external barometer node (DroneCAN) that publishes a clean static pressure to ArduPilot, and how this
repo is used to design and validate it. Written 2026-10-22. Read `AGENTS.md` first; the rules there still apply
(use uv, small focused changes, ask before deleting files, adding dependencies or changing data formats).

Status legend used below: **[verified]** = checked in the ArduPilot source in `submodule/ardupilot` or in the datasheet;
**[measured]** = from the flight data in this repo's analysis; **[assumed]** = a guess that needs checking.

## 1. Goal

Our barometer sits in the rotor flow of a large multicopter. The blade-passing pressure pulses alias into the
barometer's band and show up as noise on the baro altitude. We want a node that samples the pressure fast, removes the
blade-passing tone digitally, and publishes a smooth static pressure over DroneCAN. Optionally the same node also
publishes dynamic pressure (pitot) so forward-flight pressure error can be compensated.

Non-goals for the first version: changing ArduPilot itself, or compensating forward flight with a model of our own.

## 2. What the data says

All from `00000064.BIN` (flight controller) and the MCAP rotor-speed log, analysed with `replay baro-sim`.

- **[measured]** Real baro altitude noise in flight (faster than 1 s): 0.666 m. The simulator, with no fitting, gave
  0.457 m (69 %). Almost all of the simulated noise is the 52.6 Hz blade tone aliasing into 0-5 Hz through 50 Hz
  sampling and the 10 Hz average. Without blade pulses the simulated noise is 0.041 m.
- **[measured]** The real baro reads HIGH when thrust rises. The model for a baro 0.3 m below the rotor plane predicts
  LOW. Either the mount geometry I assumed is wrong or another effect dominates. The model's steady offset is therefore
  not trustworthy; do not size `BARO1_THST_SCALE` from it.
- **[measured]** After removing the slow part (20 s high-pass), the model explains only 4-10 % of the real excess
  pressure. `CTUN.Alt` has its own offset and drift (about -7 m on the ground), so raw-altitude fits are dominated by that.
- **[measured]** MCAP clock is 0.116 s ahead of the BIN (attitude cross-correlation 0.9999).
- **[measured]** `IMU.AccX/Y/Z` in a `.BIN` is the firmware-filtered acceleration (see 3), so it cannot show the blade
  tone. IMU is logged at about 400 Hz; baro at 10 Hz.
- **[assumed]** Rotor positions relative to the baro (four rotors, 1.95 m, 45 degrees), 3 blades, radius 1.45 m,
  k_T = 1220 N/krpm^2. Replace with the real layout before trusting any geometry result.

Conclusion: the noise part of the model is useful, the steady-offset part is not. Design the filter from recorded
high-rate data, not from the model.

## 3. ArduPilot facts that shape the design

- **[verified]** Copter reads the baro at 10 Hz: `SCHED_TASK(update_altitude, 10, ...)`, `ArduCopter/Copter.cpp:175`.
- **[verified]** DroneCAN baro driver (`libraries/AP_Baro/AP_Baro_DroneCAN.cpp`): each `StaticPressure` message goes into
  a wrapping accumulator (cap 32 samples); at the 10 Hz update the driver returns the mean of what arrived and resets.
  Temperature is a separate `StaticTemperature` message. The variance field is not used. So ArduPilot only averages; it
  cannot remove aliasing. Whatever we publish is the signal.
- **[verified]** Other baro drivers also just average (BMP388: `pressure_sum / pressure_count`; MS5611: wrapping
  accumulator). `BARO_FLTR_RNG` is an outlier gate (default 0 = off), not a low-pass.
- **[verified]** Thrust compensation exists in this build: `BARO1_THST_SCALE` (Pa, range +-300) multiplied by filtered
  normalised throttle is subtracted from the pressure; `BARO_THST_FILT` (default 1 Hz) low-passes the throttle
  (`AP_Baro.cpp:255-275`, `:1069`). It corrects the slow, thrust-correlated offset only.
- **[verified]** Wind coefficient groups `BARO1_WCF_ENABLE/FWD/BCK/RGT/LFT/UP/DN` exist (`AP_Baro_Wind.cpp`) for
  pressure error from airflow. Not verified: whether they use a measured airspeed or only the EKF wind estimate.
- **[verified]** There is no baro position / lever-arm parameter. `BARO_ALT_OFFSET` is a constant added to altitude and
  resets on reboot. EKF3 tracks a slow baro height offset (`AP_NavEKF3_Measurements.cpp:815`), so a constant
  geometric offset is harmless; only thrust- and speed-dependent error matters.
- **[verified]** The logged `IMU` accel is `_accel_filtered` (low-pass `INS_ACCEL_FILTER`, default 20 Hz). Raw vibration needs
  the batch sampler (`ISBH`/`ISBD`, `INS_LOG_BAT_MASK`).
- **[verified]** ArduPilot already has native AUAV drivers (`AP_Baro_AUAV`, polls every 50 ms; `AP_Airspeed_AUAV`, every
  20 ms with the average-8 command). Use them as a reference for the sensor protocol and compensation maths.
- **[assumed]** Airspeed over DroneCAN is the `RawAirData` message (handler at `AP_Airspeed_DroneCAN.cpp:88`).
  ESC rpm is in `uavcan.equipment.esc.Status`. Check the exact field names in the DSDL before coding.

## 4. Sensor: Amphenol All Sensors AUAV-L05D-M25125-RR-N

From datasheet DS-0432 Rev A **[verified]**:

- Combined sensor, two independent channels. Differential +-5 inH2O (+-1.25 kPa); absolute 250-1250 mbar.
- Two barbed side ports, H and L. Differential positive pressure is applied to H; the absolute channel refers to **L**.
  So: pitot (total) to H, static to L, one static line feeds both channels, and H - L is dynamic pressure directly.
- I2C (0x26 diff, 0x27 abs) or SPI. 3.3 V calibrated, 1.68-3.6 V allowed, about 5.3 mA active.
- Resolution 17.2 bit no-missing-codes (about 0.7 Pa on the absolute channel). Noise figure is not given (TBD).
- Accuracy: differential +-0.10 % FS (about 2.5 Pa) and total error band +-0.25 % (about 6 Pa); absolute +-0.10 % of
  1000 mbar (about 100 Pa). Long-term offset drift +-0.25 % FS per year: zero the differential channel before each flight.
- Offset position sensitivity +-0.10 % FS per +-1 g. Unclear whether it applies to the absolute channel too (that would be
  about 100 Pa/g, about 8 m). **Ask the manufacturer.**
- Conversion time (typical): absolute 4.9 ms single, 9.6 / 19.0 / 37.8 / 75.2 ms for 2/4/8/16 averages; differential
  2.0 / 3.8 / 7.4 / 14.6 / 29.0 ms. So the absolute channel tops out near 200 Hz.
- The MCU applies the extended compensation (cubic correction plus temperature term from EEPROM coefficients). Copy
  it from `AP_Airspeed_AUAV.cpp` / `AP_Baro_AUAV.cpp`; the absolute channel needed care when reading coefficients
  (comment in the ArduPilot code: it "gets stuck" if read in one call).

Honest assessment: the absolute channel is **not** quieter or more accurate than a normal baro in absolute terms. The
value of this part is the dynamic channel and having both in one package on one static line.

## 5. Architecture

```
AUAV (I2C/SPI) --200 Hz--> [compensate] --> [rpm-tracking notch(es)] --> [low-pass, 2nd order] --> decimate 25-50 Hz
                                                  ^                                                   |
                           ESC rpm (DroneCAN esc.Status) -+                                           v
                                                         StaticPressure + StaticTemperature (DroneCAN) -> ArduPilot
                                         (optional) RawAirData from the differential channel ---------^
```

Filtering happens at the high rate, before decimation. That is the whole point: nothing downstream can undo aliasing.

## 6. Decisions, reasoning, pros and cons

### 6.1 Filter in the node (chosen) vs custom ArduPilot driver vs stock ArduPilot only
- **Node (chosen).** Pros: ArduPilot stays stock and upgradeable; the filter sees the full-rate samples; works with any
  autopilot; parameters tunable over DroneCAN. Cons: we own firmware, a bootloader and a parameter store; a faulty node
  is a new failure point; ArduPilot cannot know the data is filtered.
- Custom ArduPilot driver. Pros: one codebase. Cons: a fork to maintain on every upgrade; the I2C bus and the 10 Hz
  task are the wrong place to sample at 200 Hz.
- Stock only. Cons: cannot remove aliased tone; only averaging.

### 6.2 Low-pass is mandatory; notch is an addition (chosen: both)
- Low-pass (2nd-order Butterworth, start at 5 Hz). Pros: simple, no rpm needed, also removes harmonics and the
  once-per-revolution terms that grow in forward flight. Attenuation at 52 Hz about -40 dB. Cons: group delay about
  45 ms at 5 Hz (about 75 ms at 3 Hz, 110 ms at 2 Hz).
- rpm-tracking notch. Pros: removes the tone with much less delay than a lower cutoff; lets the low-pass stay at 5 Hz or
  higher. Cons: needs rpm over DroneCAN; each rotor has its own blade-passing frequency, so forward flight needs one
  notch per rotor (or a wide, low-Q notch); more state, more tuning; a stale or missing rpm leaves the tone in.
- Decision: low-pass always on, notch optional and parameter-enabled. If rpm is missing or stale for more than a set time,
  the notch bypasses itself (the low-pass alone still protects).

### 6.3 Cutoff
- 5 Hz, 2nd order as the starting point. The EKF fuses baro with the IMU, so the baro only has to follow dynamics under
  1-2 Hz. Tune from recordings. Lower (3 Hz) gives more margin but costs delay.
- Whatever the total delay is (sensor averaging + filter + CAN + driver average), set `EK3_HGT_DELAY` (default 60 ms)
  to match. A baro that is late but described as on time biases vertical velocity in manoeuvres.

### 6.4 Sampling rate and the sensor's own averaging
- Sample at the sensor's single-conversion rate (about 200 Hz) or use 2-4 internal averages. Pros of internal averaging: a
  free comb filter (average of 4 conversions, 19 ms, puts a null near 51 Hz). Cons: it is only a null at one frequency,
  rpm moves the tone, and it lowers the rate.
- Watch aliasing of **harmonics** at 200 Hz: the 2nd harmonic (about 105 Hz) folds to about 95 Hz (harmless), but the 3rd lands on
  200 Hz when the blade-passing frequency is about 67 Hz and folds onto DC, where no digital filter can remove it. Choose rate
  and averaging so no harmonic folds into the passband at normal rpm. This is [assumed] arithmetic; verify with a recording.

### 6.5 Publish rate: 25-50 Hz, steady
- At least 5x the cutoff, so the 10 Hz ArduPilot average sees a smooth signal. Constant rate matters more than a high one
  (the driver averages whatever arrived in 0.1 s). CAN load is trivial.

### 6.6 Where to correct the steady thrust offset
- In ArduPilot with `BARO1_THST_SCALE` (chosen for v1), not in the node. Pros: it uses the flight controller's throttle, no
  extra messages, easy to tune per airframe. Cons: linear in throttle only; throttle is not thrust (battery voltage,
  expo); one constant per baro.
- Alternative: the node computes it from ESC rpm (thrust is about rpm^2). Pros: physically closer, handles voltage sag. Cons:
  duplicates ArduPilot's logic and hides the correction from the log. Revisit if the throttle-based fit is poor.

### 6.7 Forward flight
- Dynamic pressure q = 0.5 rho V^2 is about 245 Pa at 20 m/s (about 20 m of equivalent altitude near the ground), so the
  error is large if the static port picks up even a fraction (Cp 0.1-0.3).
- Pitot + differential channel gives V; correct with `BARO1_WCF_*` (needs checking that it can use a measured airspeed) or in the
  node as `p_static - Cp(direction) * q`. Pros: measured, not estimated. Cons: pitot reads only axial flow, is polluted
  by rotor wash and angle, is poor below about 5 m/s (q about 15 Pa against a few Pa of offset); needs flight calibration.
- Mount pitot ahead of the rotors on a boom, angled to the cruise attitude, Kiel-style head if possible; take static from a
  pitot-static probe rather than a hole in the airframe.

### 6.8 Mounting height
- Higher above the rotor plane lowers the blade-passing pulses quickly, but the steady (suction) offset stays large up to at least 1 m in the model [assumed
  geometry] and the ram error grows because the port sees clean free stream. A tall mast also flexes; a resonance near the
  blade-passing frequency would add microphonic noise. Practical limit mentioned: 0.5 m or less.
- Treat height as a hardware filter, not a replacement for the electronic one.

### 6.9 Pneumatic filtering (restrictor + cavity) at the port
- Pros: removes the tone before sampling, so the electronics need less. Cons: adds a lag and can resonate; tuning needs
  bench tests. Worth adding after the electronic version works.

### 6.10 Direct I2C to the flight controller instead of DroneCAN
- Pros: native `AP_Baro_AUAV` driver already exists, no firmware to write. Cons: the 10 Hz task and I2C bus give no
  high-rate filtering, so nothing removes the aliasing, and long I2C cables add noise. DroneCAN is chosen because the
  signal must be filtered before it leaves the node.

## 7. Node parameters (DroneCAN node parameters, tunable from the ground station)

| Parameter | Start | Notes |
|---|---|---|
| sample_rate_hz | 200 | single conversion; check harmonic folding |
| avg_count | 1 | sensor averaging, 1/2/4 |
| lp_cutoff_hz | 5 | 0 disables |
| lp_order | 2 | start low; raise only if recordings show residual tone |
| notch_enable | 1 | needs rpm |
| notch_harmonics | 2 | blade-passing frequency and 2x |
| notch_q | 5 | wide enough for rotor spread |
| notch_per_rotor | 1 | one notch per ESC rpm |
| rpm_timeout_ms | 200 | beyond this the notch bypasses |
| blades | 3 | |
| publish_rate_hz | 50 | |
| baro_zero_mode | boot | pitot zero on ground before flight |
| raw_log_enable | 1 | publish/log unfiltered samples for design (v0) |

## 8. Plan

**Phase 0: understand the real signal (no filtering yet).** Build a node that logs unfiltered 200 Hz pressure and ESC rpm
(to SD, or over CAN to a logger). Fly hover and forward legs. Use `replay vibration` (works on any signal) on the pressure to see the
tone, harmonics and any other noise. Exit: we know the tone amplitude, which harmonics matter and the rpm spread.

**Phase 1: design offline.** Run candidate filters on the recording in Python (add a small module under `lib/` and a
test under `unittest/`; no new dependency needed, numpy is enough), compare residual noise and delay. Update the layout
in `config/baro_simulation.toml` with the real rotor positions and baro position, and re-run `replay baro-sim`.
Exit: chosen cutoff, order and notch settings, and a measured total delay.

**Phase 2: node firmware.** Sensor driver with extended compensation, filter chain, DroneCAN publishing, parameters,
health checks (stuck value, NaN, range, rpm timeout). Bench test with a signal injected through a pressure source.

**Phase 3: flight validation.** Compare the filtered output with the logged raw; check `BAlt` noise against the 0.666 m
baseline; set `EK3_HGT_DELAY` to the measured delay; fit `BARO1_THST_SCALE` from hover; check EKF innovations.

**Phase 4: forward flight.** Add the pitot, calibrate Cp per direction, enable and tune `BARO1_WCF_*` (after confirming
how it gets airspeed), then test fast legs and the pitch manoeuvres (around 12:43 in the logged flight).

## 9. Validation with this repo

- `uv run replay baro-sim --config config/baro_simulation.toml [--sweep-height=...]` for rotor-pressure predictions and
  height trade-offs. Remember the steady-offset caveat in section 2.
- `uv run replay vibration --file <log> --start .. --end ..` for spectra of any recorded signal (enDAQ support is a planned
  follow-up; the FFT code works on plain `Signal` objects in `lib/vibration/`).
- `uv run replay dashboard --file <summary>.sqlite -o out.html` to view results.
- Add tests under `unittest/` for any new filter code (impulse and sine response, delay, notch tracking), and keep the
  suite passing: `uv run python -m unittest discover -s unittest`.

## 10. Risks

- **Fault hiding:** a heavy filter makes a stuck sensor look smooth. Mitigate with node health checks and publishing nothing on failure.
- **Delay mismatch:** wrong `EK3_HGT_DELAY` degrades vertical velocity. Measure, then set.
- **rpm dependence:** a lost rpm stream leaves the tone; the low-pass must still cover it.
- **Harmonic folding** at specific rpm (6.4).
- **Model mismatch:** the simulator's steady offset has the wrong sign against real data. Do not tune parameters from it.
- **Sensor limits:** absolute accuracy about +-100 Pa; position sensitivity ambiguity (4); differential channel offset drift (zero before flight).
- **Mast resonance** near the blade-passing frequency.

## 11. Open questions

1. Real rotor positions and baro position relative to the rotors; real mast height available.
2. Do the ESCs publish rpm over DroneCAN, at what rate and latency?
3. Which MCU and RTOS for the node; is there room for a small FFT or Goertzel as a self-tracking fallback?
4. Does `BARO1_WCF_*` use a measured airspeed or only the EKF wind estimate? (read `AP_Baro_Wind.cpp` and its callers)
5. Manufacturer answer on the position-sensitivity spec for the absolute channel.
6. Exact DroneCAN field names for ESC rpm and `RawAirData` in this ArduPilot version.
7. `BARO1_THST_SCALE` and `BARO_THST_FILT` values in the logged flight (read from the BIN) to know whether `BAlt` already had a correction.

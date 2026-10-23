# Air Data Module firmware (Flowcopter)

Firmware for the [Beyond Robotix Air Data Module](https://beyond-robotix.gitbook.io/docs/air-data-module):
a Beyond Robotix Micro Node (STM32L431) with an Amphenol All Sensors AUAV
absolute and differential pressure sensor, talking DroneCAN.

It is based on the Beyond Robotix
[AUAV-DroneCAN](https://github.com/BeyondRobotix/AUAV-DroneCAN) firmware and
changes it for use as a **barometer on a multicopter**, where the static port
sits in the rotor flow: the absolute pressure is sampled fast, low-pass
filtered on the node to remove the blade-passing tone, and published as a
clean, steady DroneCAN `StaticPressure`. Airspeed is off by default.

<img src="./assets/sensor.jpg" width="50%">

- [How it works](#how-it-works)
- [Required setup](#required-setup)
- [Node parameters](#node-parameters)
- [Status messages](#status-messages)
- [Optional setup](#optional-setup)
- [Known limitations](#known-limitations)
- [Building](#building)
- [Licence](#licence)

## How it works

```
AUAV absolute channel --single conversions, BARO_SAMPLE_HZ (180 Hz)--> health checks
   --> Butterworth low-pass (BARO_LP_HZ, BARO_LP_ORDER) --> StaticPressure at BARO_PUB_HZ (50 Hz) --> ArduPilot
                                                       \-> StaticTemperature at 10 Hz
```

1. **Sampling.** The absolute channel is read with single conversions on a
   fixed time grid (`BARO_SAMPLE_HZ`, default 180 Hz; one conversion takes
   about 4.9 ms). The I2C bus runs at 400 kHz. The stock firmware instead
   averaged 16 conversions and read them about every 100 ms.
2. **Filtering.** Each sample goes through a Butterworth low-pass (default
   5 Hz, 2nd order) running at the full sample rate. Filtering on the node is
   the point: ArduPilot only averages what it receives at its 10 Hz baro
   update, so it cannot remove a blade-passing tone (about 52.6 Hz in hover on
   our aircraft) once that has aliased. At 5 Hz, 2nd order, the tone is
   attenuated by more than 40 dB and the filter delays slow signals by 45 ms.
3. **Publishing.** The filtered value is published as
   `uavcan.equipment.air_data.StaticPressure` on a steady grid (`BARO_PUB_HZ`,
   default 50 Hz), so ArduPilot's 10 Hz average always sees the same number of
   samples. `StaticTemperature` follows at 10 Hz.
4. **Health.** Nothing is published while the reading is unhealthy, so a dead
   sensor shows up in ArduPilot as a missing baro (it falls back to another)
   rather than a smooth, stale one. A reading is unhealthy when:
   - no valid sample arrived in the last 100 ms,
   - the value is outside the sensor's 250-1250 mbar range or not a number,
   - the raw sensor output has not changed for 1 s (stuck sensor), or
   - the filter settings are invalid (see [Known limitations](#known-limitations)).
5. **Optional outputs.** Airspeed (`RawAirData`) and a raw 11-bit CAN output
   of the filtered and unfiltered pressure are available but off by default;
   see [Optional setup](#optional-setup).

All node parameters except `NODEID` are re-read every second, so changes from
the ground station apply without a reboot.

## Required setup

### 1. Flash the firmware

Build `firmware.bin` (see [BUILD.md](BUILD.md)) and upload it over DroneCAN
from Mission Planner (DroneCAN/UAVCAN screen, select the node, update
firmware) or the DroneCAN GUI Tool. Use the `.bin`, not an `.apj`.

### 2. Check the node parameters

After an update that adds parameters, the node's parameters return to their
defaults. Set at least:

- `NODEID` back to the module's previous ID (default 50). ArduPilot identifies
  the baro by node ID, so a new ID appears as a different barometer.

The defaults are otherwise ready to use as a barometer.

### 3. Set up ArduPilot

| Parameter | Value | Why |
|---|---|---|
| `CAN_P1_DRIVER` | 1 | enable CAN port 1 |
| `CAN_D1_PROTOCOL` | 1 | DroneCAN |
| `ARSPD_ENABLE` | 0 | not needed for the barometer; airspeed is off on the module by default |
| `BARO_PRIMARY` | index of the module's baro | make it the main baro |
| `EK3_HGT_DELAY` | measured value | the filter adds about 45 ms (at 5 Hz, 2nd order) to the baro delay |

There is no "external baro" parameter: ArduPilot registers a DroneCAN baro for
every node that sends `StaticPressure`, but only at boot, so power the module
with the flight controller. To find its index, look for the `UAVCAN` baro in
Mission Planner's HW ID tab (or in `BARO1_DEVID` .. `BARO3_DEVID`);
`BARO_PRIMARY` counts from 0, so the second baro is 1.

## Node parameters

Set over DroneCAN from Mission Planner or the DroneCAN GUI Tool.

### Barometer

| Parameter | Default | Range | Description |
|---|---|---|---|
| `NODEID` | 50 | 0-127 | DroneCAN node ID. 0 = dynamic allocation. Applies after a reboot |
| `BARO_SAMPLE_HZ` | 180 | 20-190 | sample rate of the absolute channel. Higher rates may overrun the sensor (watch the overrun count in the status report) |
| `BARO_LP_ENABLE` | 1 | 0-1 | 0 bypasses the filter and publishes the latest raw sample. For comparison only: the raw signal still contains the blade-passing tone |
| `BARO_LP_HZ` | 5 | 0-20 | low-pass cutoff, Hz. 0 also bypasses the filter. Must be below 40 % of `BARO_SAMPLE_HZ` |
| `BARO_LP_ORDER` | 2 | 2-4 | filter order: **2 or 4 only** (3 is invalid) |
| `BARO_PUB_HZ` | 50 | 10-100 | `StaticPressure` publish rate |

Filter delay for slowly changing pressure: 2nd order about 0.225 / cutoff
(45 ms at 5 Hz), 4th order about 0.416 / cutoff (83 ms at 5 Hz). The node
reports the exact value at boot and whenever the filter settings change.

### Airspeed (off by default)

| Parameter | Default | Range | Description |
|---|---|---|---|
| `AIRSPEED_ENABLE` | 0 | 0-1 | 1 reads the differential channel and publishes `RawAirData` at 20 Hz |
| `SENSOR_PRESSURE` | 5 | 5, 10, 30, 60, 100 | differential range of the fitted AUAV sensor in inH2O (L05D = 5, L10D = 10, ...). Only used when airspeed is on |

### Raw 11-bit CAN output (off by default)

| Parameter | Default | Range | Description |
|---|---|---|---|
| `RAWCAN_ENABLE` | 0 | 0-1 | 1 sends the two frames below |
| `RAWCAN_ID` | 2030 (0x7EE) | 1-2031 | ID of the filtered frame |
| `RAWCAN_RAW_ID` | 2031 (0x7EF) | 1-2031 | ID of the unfiltered frame; must differ from `RAWCAN_ID` |

See [Raw 11-bit CAN output](#raw-11-bit-can-output) for the frame format.

## Status messages

The node sends these as DroneCAN debug messages; Mission Planner shows them in
the Messages tab.

| Message | Meaning |
|---|---|
| `BARO: 180 Hz, LP 5.0 Hz order 2, filter delay 45 ms, pub 50 Hz` | filter settings in use (at boot and on change) |
| `BARO: 180 Hz, filter OFF (raw samples), pub 50 Hz` | the filter is bypassed |
| `BARO: bad filter settings (...), not publishing` | invalid filter settings; no pressure is published |
| `BARO: 180 Hz measured, 0 overruns, 0 errors in 10 s` | sent 10 s after boot, then every 10 s only if there were overruns or errors. Overruns: lower `BARO_SAMPLE_HZ`. Errors: sensor or I2C problem |
| `BARO: raw pressure stuck, not publishing` | the sensor output stopped changing |
| `RAWCAN: filtered 0x7EE at 50 Hz, unfiltered 0x7EF per sample` | raw CAN output on |
| `RAWCAN: RAWCAN_RAW_ID equals RAWCAN_ID 0x7EE, unfiltered off` | both IDs are the same; only the filtered frame is sent |
| `RAWCAN: N frames not sent in 10 s (bus not acknowledging?)` | raw CAN frames could not be sent; usually nothing else on the bus is acknowledging them |
| `Configure SENSOR_PRESSURE to 5, 10, 30, 60, or 100 ...` | airspeed is on but `SENSOR_PRESSURE` is not a valid range |

## Optional setup

### Airspeed

Set `AIRSPEED_ENABLE = 1` and `SENSOR_PRESSURE` to the fitted sensor's range
on the module, and in ArduPilot `ARSPD_ENABLE = 1`, `ARSPD_TYPE = 8`
(DroneCAN). Connect the pitot-static tube with the **high** (H) port of the
sensor to the total-pressure (straight) barb and the **low** (L) port to the
static (angled) barb. The absolute channel, and so the barometer, also reads
the L port.

### Raw 11-bit CAN output

For receivers that cannot subscribe to DroneCAN messages (an ArduPilot 4.6 Lua
script, a CAN logger), and for looking at the unfiltered signal. With
`RAWCAN_ENABLE = 1` the node sends two 11-bit (standard ID) frames:

| ID | Rate | Content |
|---|---|---|
| `RAWCAN_ID` (0x7EE) | `BARO_PUB_HZ`, with each `StaticPressure` | the filtered value ArduPilot receives |
| `RAWCAN_RAW_ID` (0x7EF) | every sample (`BARO_SAMPLE_HZ`) | the unfiltered sensor sample |

Each frame has 5 data bytes: bytes 0-3 the pressure in Pa as a little-endian
float32, byte 4 a sequence counter (0-255). Each frame kind has its own
counter, which advances by one per published value or per sample, including
frames that could not be sent, so receivers can detect gaps. Details are in
[`lib/RawCanFrame/src/RawCanFrame.h`](lib/RawCanFrame/src/RawCanFrame.h).

A CAN database for both frames, for CAN tools such as SavvyCAN, PCAN-View,
CANalyzer or Python's `cantools`, is in
[`dbc/air_data_module.dbc`](dbc/air_data_module.dbc). It uses the default
IDs; edit its two `BO_` lines (decimal IDs) if the module uses other ones.

The default IDs are the two highest usable 11-bit IDs (0x7F0-0x7FF are
reserved by CAN 2.0A), which gives them the lowest bus priority: they lose
arbitration to all DroneCAN traffic except DroneCAN's lowest priority, so they
never delay ESC commands or other sensors. DroneCAN nodes ignore 11-bit
frames; make sure no other 11-bit device on the bus uses the same IDs. Bus
load at the defaults is about 230 frames/s, roughly 2.5 % of a 1 Mbit/s bus.
`StaticPressure` to ArduPilot is not affected by this output.

### Logging on the flight controller (Lua)

[`lua_scripts/adm_static_pressure_log.lua`](lua_scripts/adm_static_pressure_log.lua)
logs every pressure message from the module to the onboard log (ArduPilot's
own `BARO` log message is only written at 10 Hz) and prints the latest value
and rate to the ground station. It has two sources, chosen by `ADMP_SRC`:

| `ADMP_SRC` | Source | Needs | Log messages |
|---|---|---|---|
| 0 (default) | auto: DroneCAN if available, otherwise raw | | |
| 1 | DroneCAN `StaticPressure` | ArduPilot 4.7.0 or later | `ADMP` (Node, DtUS, Press) |
| 2 | raw 11-bit frames | `RAWCAN_ENABLE = 1` on the module; `CAN_D1_PROTOCOL2 = 10` on the flight controller | `ADMR` filtered, `ADMU` unfiltered (Seg, Idx, Seq, Miss, Press) |

Setup: `SCR_ENABLE = 1` (and `LOG_DISARMED = 1` to log before arming), copy
the script to `APM/scripts/` on the SD card and reboot. The script adds its
own parameters (`ADMP_ENABLE`, `ADMP_NODE`, `ADMP_RUN_MS`, `ADMP_RPT_S`,
`ADMP_DRV`, `ADMP_SRC`, `ADMP_CAN_ID`, `ADMP_RAW_ID`); they are documented at
the top of the script. `ADMP_CAN_ID` and `ADMP_RAW_ID` must match the module's
`RAWCAN_ID` and `RAWCAN_RAW_ID`.

Why two sources: ArduPilot 4.6 cannot give a script DroneCAN messages from a
DroneCAN port (a script sharing the port with `CAN_D1_PROTOCOL2` only receives
11-bit frames). ArduPilot 4.7 added `DroneCAN_Handle`, which subscribes to
DroneCAN messages directly.

Other scripts in `lua_scripts/`: `adm_pressure_print.lua` only prints the
filtered raw frame (superseded by the logging script; do not run both), and
`hello_ardupilot.lua` is a minimal scripting example.

### Analysing the blade-passing tone

To see the tone in the logs you need the **unfiltered** signal sampled at more
than twice its frequency: the `ADMU` log (raw source) at the default 180 Hz
sample rate covers tones up to 90 Hz. The filtered signal at 50 Hz cannot show
it: the filter removes it and 50 Hz sampling folds it to low frequencies.

- Use `ADMU`'s `Idx` as the time base (sample time = `Idx` / sample rate, with
  the rate from the node's `BARO: N Hz measured` message), not `TimeUS`, which
  carries up to `ADMP_RUN_MS` of polling jitter.
- Analyse each `Seg` separately; a new segment starts after gaps the 8-bit
  counter cannot measure.
- `ADMR` and `ADMU` run at different rates with separate counters; line them
  up by `TimeUS`.
- Harmonics above half the sample rate fold back to known frequencies; logging
  motor rpm alongside helps to identify them.

## Known limitations

- **Parameters reset after some updates.** The parameter store is keyed by the
  list of parameter names; when a firmware update adds or removes a parameter,
  all values return to their defaults. Changed defaults alone do not affect a
  module that already has stored values.
- **"Reset to defaults" from a ground station sets parameters to their
  minimums**, not their defaults (in libArduinoDroneCAN). That turns the filter
  off (`BARO_LP_ENABLE = 0`) and sets `BARO_SAMPLE_HZ = 20`, among others. Set
  the parameters by hand instead.
- **Invalid filter settings stop the baro.** `BARO_LP_ORDER = 3`, or a cutoff
  at or above 40 % of the sample rate, makes the node stop publishing pressure
  until the settings are fixed.
- **Single conversions are noisier per sample** than the stock firmware's
  16-conversion average; the filter takes care of this for the published
  value. The unfiltered output shows the full noise.
- **No extended compensation.** The sensor driver uses the datasheet's basic
  transfer function, without the optional cubic and temperature correction
  from the sensor's coefficients.
- **The raw CAN output is STM32L431-specific** (it writes the CAN controller's
  transmit mailboxes 1 and 2 directly, next to the DroneCAN library's
  mailbox 0).

## Building

See [BUILD.md](BUILD.md) for setup, build, unit tests and upload. In short:

```bash
pio run -e Micro-Node-App   # firmware: .pio/build/Micro-Node-App/firmware.bin
pio test -e native          # filter and frame-format unit tests on the PC
```

The board support package
([br_platformio_hwdef](https://github.com/BeyondRobotix/br_platformio_hwdef))
and the DroneCAN library
([libArduinoDroneCAN](https://github.com/BeyondRobotix/libArduinoDroneCAN)) are
downloaded by PlatformIO at the versions pinned in `platformio.ini`.

## Licence

The application is BSD 3-Clause, based on the Beyond Robotix AUAV-DroneCAN
firmware; see [LICENSE](LICENSE). The firmware binary also contains
third-party code under other licences; see
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md), including an open issue: the
AUAV sensor driver in `lib/AllSensors_AUAV` has no licence.

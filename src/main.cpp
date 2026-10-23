#include <Arduino.h>
#include <dronecan.h>
#include <AllSensors_AUAV.h>
#include <PressureFilter.h>
#include <RawCanFrame.h>
#include <math.h>

#include "raw_can.h"

// Air data node with a filtered static pressure.
//
// The absolute channel is sampled with single conversions on a fixed grid
// (BARO_SAMPLE_HZ), low-passed at that rate (BARO_LP_HZ, BARO_LP_ORDER) and
// published as StaticPressure at a steady BARO_PUB_HZ. Filtering before the
// signal leaves the node is the point: ArduPilot only averages what arrives
// at 10 Hz and cannot remove a blade-passing tone that has already aliased.
//
// BARO_LP_ENABLE = 0 bypasses the filter and publishes the latest raw sample,
// for comparing against the filtered output. The raw signal still contains
// the blade-passing tone, which aliases when ArduPilot averages it.
//
// The differential channel (airspeed) is off unless AIRSPEED_ENABLE = 1. When
// off, the node never talks to it and sends no RawAirData, so ArduPilot only
// sees a barometer. When on, it works as in the Beyond Robotix firmware:
// AVERAGE16 conversions published as RawAirData at 20 Hz, scaled by
// SENSOR_PRESSURE.
//
// RAWCAN_ENABLE = 1 additionally sends two 11-bit CAN frames (format in
// RawCanFrame.h), for receivers that cannot subscribe to DroneCAN and for
// analysing the raw signal. Off by default.
//   RAWCAN_ID      the filtered value published as StaticPressure, sent with
//                  it at BARO_PUB_HZ
//   RAWCAN_RAW_ID  every unfiltered sample, at the sample rate
// StaticPressure itself is unaffected.
//
// Nothing is published for a channel that is unhealthy (no fresh reading,
// out of range, or a raw value that stopped changing), so a dead sensor shows
// up as a missing baro in ArduPilot instead of a smooth, stale one.

AllSensors_AUAV pressureSensor(&Wire);

// set up your parameters here with default values. NODEID should be kept
std::vector<DroneCAN::parameter> custom_parameters = {
    {"NODEID", DroneCAN::INT, 50, 0, 127},
    {"SENSOR_PRESSURE", DroneCAN::INT, 5, 0, 100},
    {"BARO_SAMPLE_HZ", DroneCAN::INT, 180, 20, 190},
    {"BARO_LP_HZ", DroneCAN::REAL, 5, 0, 20},
    {"BARO_LP_ORDER", DroneCAN::INT, 2, 2, 4},
    {"BARO_PUB_HZ", DroneCAN::INT, 50, 10, 100},
    {"AIRSPEED_ENABLE", DroneCAN::INT, 0, 0, 1},
    {"BARO_LP_ENABLE", DroneCAN::INT, 1, 0, 1},
    {"RAWCAN_ENABLE", DroneCAN::INT, 0, 0, 1},
    {"RAWCAN_ID", DroneCAN::INT, 0x2A0, 1, RAW_CAN_ID_MAX},
    {"RAWCAN_RAW_ID", DroneCAN::INT, 0x2A1, 1, RAW_CAN_ID_MAX},
};

DroneCAN dronecan;

// Absolute channel limits from the datasheet (250-1250 mbar).
static const float ABS_MIN_PA = 25000.0f;
static const float ABS_MAX_PA = 125000.0f;

// A reading older than this makes the channel unhealthy.
static const uint32_t ABS_STALE_MS = 100;
static const uint32_t DIFF_STALE_MS = 300;

// The raw 24-bit pressure never repeats this long on a working sensor.
static const uint32_t ABS_STUCK_MS = 1000;

// Give up on a conversion that is still busy after this long and restart it.
static const uint32_t ABS_BUSY_TIMEOUT_US = 50000;
static const uint32_t ABS_BUSY_RETRY_US = 200;

// ---------------------------------------------------------------------------
// Configuration (re-read from the parameters once a second)

struct BaroConfig
{
    int sample_hz;
    bool lp_enable;
    float lp_hz;
    int lp_order;
    int pub_hz;

    bool operator!=(const BaroConfig &o) const
    {
        return sample_hz != o.sample_hz || lp_enable != o.lp_enable ||
               lp_hz != o.lp_hz || lp_order != o.lp_order || pub_hz != o.pub_hz;
    }
};

static BaroConfig config{};
static bool filter_ok = false;
static PressureLowPass lowpass;

static BaroConfig read_config()
{
    BaroConfig c;
    c.sample_hz = (int)dronecan.getParameter("BARO_SAMPLE_HZ");
    c.lp_enable = dronecan.getParameter("BARO_LP_ENABLE") > 0.5f;
    c.lp_hz = dronecan.getParameter("BARO_LP_HZ");
    c.lp_order = (int)dronecan.getParameter("BARO_LP_ORDER");
    c.pub_hz = (int)dronecan.getParameter("BARO_PUB_HZ");
    return c;
}

static void apply_config(const BaroConfig &c)
{
    config = c;
    // A cutoff of 0 configures the low-pass as a pass-through.
    filter_ok = lowpass.configure(c.sample_hz, c.lp_enable ? c.lp_hz : 0.0f, c.lp_order);

    char msg[90];
    if (filter_ok && !lowpass.enabled()) {
        snprintf(msg, sizeof(msg), "BARO: %d Hz, filter OFF (raw samples), pub %d Hz",
                 c.sample_hz, c.pub_hz);
        dronecan.debug(msg, 2);
        return;
    }
    if (!filter_ok) {
        snprintf(msg, sizeof(msg), "BARO: bad filter settings (LP %d.%d Hz order %d at %d Hz), not publishing",
                 (int)c.lp_hz, (int)(c.lp_hz * 10) % 10, c.lp_order, c.sample_hz);
        dronecan.debug(msg, 3);
        return;
    }
    // Filter delay only; EK3_HGT_DELAY also needs the conversion, publish
    // interval and ArduPilot's 10 Hz average on top.
    snprintf(msg, sizeof(msg), "BARO: %d Hz, LP %d.%d Hz order %d, filter delay %d ms, pub %d Hz",
             c.sample_hz, (int)c.lp_hz, (int)(c.lp_hz * 10) % 10, c.lp_order,
             (int)lroundf(lowpass.group_delay_s() * 1000.0f), c.pub_hz);
    dronecan.debug(msg, 1);
}

// ---------------------------------------------------------------------------
// Absolute channel: fixed-rate single conversions feeding the filter

static bool abs_in_flight = false;
static uint32_t abs_grid_us = 0;       // when the next conversion is due
static uint32_t abs_started_us = 0;    // when the one in flight started
static uint32_t abs_retry_us = 0;      // next poll while the sensor is busy
static bool abs_retrying = false;

static double abs_filtered = NAN;
static float abs_temperature = NAN;
static uint32_t abs_last_good_ms = 0;
static bool abs_have_good = false;
static uint32_t abs_last_raw = 0;
static uint32_t abs_raw_changed_ms = 0;
static bool abs_stuck = false;

// Counters for the periodic report.
static uint32_t abs_samples = 0;
static uint32_t abs_overruns = 0;
static uint32_t abs_errors = 0;

static void abs_start(uint32_t now_us)
{
    pressureSensor.startMeasurement(AllSensors_AUAV::SensorType::ABSOLUTE,
                                    AllSensors_AUAV::MeasurementType::SINGLE);
    abs_in_flight = true;
    abs_started_us = now_us;
    abs_retrying = false;
}

static void send_raw_can_unfiltered(float pressure_pa);

static void abs_process(float pressure, float temperature, uint32_t raw, uint32_t now_ms)
{
    if (!isfinite(pressure) || pressure < ABS_MIN_PA || pressure > ABS_MAX_PA) {
        abs_errors++;
        return;
    }

    if (raw != abs_last_raw) {
        abs_last_raw = raw;
        abs_raw_changed_ms = now_ms;
        abs_stuck = false;
    } else if (now_ms - abs_raw_changed_ms > ABS_STUCK_MS) {
        if (!abs_stuck) {
            dronecan.debug("BARO: raw pressure stuck, not publishing", 3);
        }
        abs_stuck = true;
    }

    // After a gap the old filter state no longer describes the signal.
    if (!abs_have_good || now_ms - abs_last_good_ms > ABS_STALE_MS) {
        lowpass.reset();
    }

    abs_filtered = lowpass.apply(pressure);
    abs_temperature = temperature;
    abs_last_good_ms = now_ms;
    abs_have_good = true;
    abs_samples++;

    // Same health rule as StaticPressure: nothing while stuck or misconfigured.
    if (!abs_stuck && filter_ok) {
        send_raw_can_unfiltered(pressure);
    }
}

static void abs_update(uint32_t now_us, uint32_t now_ms)
{
    const uint32_t period_us = 1000000UL / (uint32_t)config.sample_hz;

    if (!abs_in_flight) {
        if ((int32_t)(now_us - abs_grid_us) >= 0) {
            abs_start(now_us);
            abs_grid_us = now_us + period_us;
        }
        return;
    }

    // The conversion in flight is read at the next grid point (or at the
    // next retry if the sensor said it was still busy).
    const uint32_t due_us = abs_retrying ? abs_retry_us : abs_grid_us;
    if ((int32_t)(now_us - due_us) < 0) {
        return;
    }

    const uint8_t status = pressureSensor.readData(AllSensors_AUAV::SensorType::ABSOLUTE);
    if (status == 2) {
        if (now_us - abs_started_us > ABS_BUSY_TIMEOUT_US) {
            abs_errors++;
            abs_in_flight = false; // restart on the next pass
            abs_grid_us = now_us;
            return;
        }
        abs_retrying = true;
        abs_retry_us = now_us + ABS_BUSY_RETRY_US;
        return;
    }

    if (status == 0) {
        abs_process(pressureSensor.pressure_a, pressureSensor.temperature_a,
                    pressureSensor.raw_p, now_ms);
    } else {
        abs_errors++;
    }

    // Start the next conversion and move the grid on. If we are already past
    // the next grid point the rate is too high for the sensor and bus: count
    // it and resynchronise rather than trying to catch up.
    const bool late = abs_retrying || (int32_t)(now_us - (abs_grid_us + period_us)) >= 0;
    abs_start(now_us);
    if (late) {
        abs_overruns++;
        abs_grid_us = now_us + period_us;
    } else {
        abs_grid_us += period_us;
    }
}

static bool abs_healthy(uint32_t now_ms)
{
    return filter_ok && abs_have_good && !abs_stuck &&
           now_ms - abs_last_good_ms <= ABS_STALE_MS;
}

// ---------------------------------------------------------------------------
// Differential channel: AVERAGE16, as in the Beyond Robotix firmware

static float diff_pressure = NAN;
static float diff_temperature = NAN;
static uint32_t diff_last_good_ms = 0;
static uint32_t diff_last_restart_ms = 0;
static bool diff_have_good = false;

static void diff_update(uint32_t now_ms)
{
    const uint8_t status = pressureSensor.readData(AllSensors_AUAV::SensorType::DIFFERENTIAL);
    if (status == 2) {
        // still converting; try again next time
        if (now_ms - diff_last_good_ms > DIFF_STALE_MS &&
            now_ms - diff_last_restart_ms > DIFF_STALE_MS) {
            pressureSensor.startMeasurement(AllSensors_AUAV::SensorType::DIFFERENTIAL,
                                            AllSensors_AUAV::MeasurementType::AVERAGE16);
            diff_last_restart_ms = now_ms;
        }
        return;
    }
    if (status == 0 && isfinite(pressureSensor.pressure_d)) {
        diff_pressure = pressureSensor.pressure_d;
        diff_temperature = pressureSensor.temperature_d;
        diff_last_good_ms = now_ms;
        diff_have_good = true;
    }
    pressureSensor.startMeasurement(AllSensors_AUAV::SensorType::DIFFERENTIAL,
                                    AllSensors_AUAV::MeasurementType::AVERAGE16);
}

static bool diff_healthy(uint32_t now_ms)
{
    return diff_have_good && now_ms - diff_last_good_ms <= DIFF_STALE_MS;
}

// Airspeed runs only when enabled and SENSOR_PRESSURE names a real sensor
// range; a bad range is reported instead of publishing wrongly scaled data.
static bool airspeed_enabled = false;
static bool airspeed_active = false;
static int airspeed_range = 0;

static bool valid_sensor_range(int range)
{
    return range == 5 || range == 10 || range == 30 || range == 60 || range == 100;
}

static void update_airspeed_config(uint32_t now_ms)
{
    airspeed_enabled = dronecan.getParameter("AIRSPEED_ENABLE") > 0.5f;
    airspeed_range = (int)dronecan.getParameter("SENSOR_PRESSURE");

    const bool active = airspeed_enabled && valid_sensor_range(airspeed_range);
    if (active) {
        // Multiplied by 2 for the sensor range, as it is 2x the value in inH2O
        pressureSensor.setPressureRange(airspeed_range * 2);
    }
    if (active && !airspeed_active) {
        // Start fresh: an idle sensor would still return its last reading.
        diff_have_good = false;
        diff_last_good_ms = now_ms;
        diff_last_restart_ms = now_ms;
        pressureSensor.startMeasurement(AllSensors_AUAV::SensorType::DIFFERENTIAL,
                                        AllSensors_AUAV::MeasurementType::AVERAGE16);
    }
    airspeed_active = active;
}

// ---------------------------------------------------------------------------
// Raw 11-bit CAN output of the published pressure

static bool raw_can_enabled = false;
static uint16_t raw_can_id = 0;        // filtered frame
static uint16_t raw_can_raw_id = 0;    // unfiltered frame, 0 when not sent
static uint8_t raw_can_filtered_seq = 0;    // +1 per published value
static uint8_t raw_can_unfiltered_seq = 0;  // +1 per sample
static uint32_t raw_can_dropped = 0;   // frames not sent since the last report

static void update_raw_can_config()
{
    const bool enabled = dronecan.getParameter("RAWCAN_ENABLE") > 0.5f;
    const uint16_t id = (uint16_t)dronecan.getParameter("RAWCAN_ID");
    uint16_t raw_id = (uint16_t)dronecan.getParameter("RAWCAN_RAW_ID");
    if (raw_id == id) {
        raw_id = 0; // a receiver could not tell the two apart
    }
    if (enabled == raw_can_enabled && id == raw_can_id && raw_id == raw_can_raw_id) {
        return;
    }
    raw_can_enabled = enabled;
    raw_can_id = id;
    raw_can_raw_id = raw_id;
    if (!enabled) {
        return;
    }
    char msg[70];
    if (raw_id == 0) {
        snprintf(msg, sizeof(msg), "RAWCAN: RAWCAN_RAW_ID equals RAWCAN_ID 0x%03X, unfiltered off", id);
        dronecan.debug(msg, 2);
    } else {
        snprintf(msg, sizeof(msg), "RAWCAN: filtered 0x%03X at %d Hz, unfiltered 0x%03X per sample",
                 id, config.pub_hz, raw_id);
        dronecan.debug(msg, 1);
    }
}

static void send_raw_can_frame(uint16_t id, float pressure_pa, uint8_t seq)
{
    uint8_t data[8];
    const uint8_t len = raw_can_pack_pressure(pressure_pa, seq, data);
    if (!raw_can_send_std(id, data, len)) {
        raw_can_dropped++;
    }
}

// Each frame kind has its own sequence counter (the two run at different
// rates); it advances even if a frame cannot be sent, so receivers see gaps.
static void send_raw_can_filtered(float pressure_pa)
{
    if (raw_can_enabled) {
        send_raw_can_frame(raw_can_id, pressure_pa, raw_can_filtered_seq++);
    }
}

static void send_raw_can_unfiltered(float pressure_pa)
{
    if (raw_can_enabled && raw_can_raw_id != 0) {
        send_raw_can_frame(raw_can_raw_id, pressure_pa, raw_can_unfiltered_seq++);
    }
}

// ---------------------------------------------------------------------------

static void report_stats(uint32_t elapsed_ms, bool always)
{
    if (!always && abs_overruns == 0 && abs_errors == 0) {
        abs_samples = 0;
        return;
    }
    char msg[90];
    snprintf(msg, sizeof(msg), "BARO: %lu Hz measured, %lu overruns, %lu errors in %lu s",
             (unsigned long)(abs_samples * 1000UL / elapsed_ms), (unsigned long)abs_overruns,
             (unsigned long)abs_errors, (unsigned long)(elapsed_ms / 1000UL));
    dronecan.debug(msg, abs_overruns || abs_errors ? 2 : 1);
    abs_samples = 0;
    abs_overruns = 0;
    abs_errors = 0;
}

// Frames are only dropped when the previous one never left mailbox 2, which
// normally means nothing else on the bus is acknowledging frames.
static void report_raw_can(uint32_t elapsed_ms)
{
    if (raw_can_dropped == 0) {
        return;
    }
    char msg[80];
    snprintf(msg, sizeof(msg), "RAWCAN: %lu frames not sent in %lu s (bus not acknowledging?)",
             (unsigned long)raw_can_dropped, (unsigned long)(elapsed_ms / 1000UL));
    dronecan.debug(msg, 2);
    raw_can_dropped = 0;
}

void setup()
{
    app_setup();
    Serial.begin(115200);
    IWatchdog.begin(2000000); // if the loop takes longer than 2 seconds, reset the system
    dronecan.version_major = 2;
    dronecan.version_minor = 0;
    dronecan.init(
        custom_parameters,
        "com.flowcopter.airdata");

    delay(100);

    Wire.setSCL(PIN_WIRE_SCL);
    Wire.setSDA(PIN_WIRE_SDA);
    Wire.begin();
    // 400 kHz so a read plus the next start fits well inside the ~5.5 ms
    // sample period (at 100 kHz it is close to 1 ms).
    Wire.setClock(400000);

    IWatchdog.reload();

    delay(20);

    pressureSensor.setPressureDiffUnit(AllSensors_AUAV::PressureUnit::PASCAL);
    pressureSensor.setPressureAbsUnit(AllSensors_AUAV::PressureUnit::PASCAL);
    pressureSensor.setTemperatureUnit(AllSensors_AUAV::TemperatureUnit::KELVIN);

    pressureSensor.readStatus(AllSensors_AUAV::SensorType::ABSOLUTE);
    delay(100);

    IWatchdog.reload();

    apply_config(read_config());
    update_airspeed_config(millis());
    update_raw_can_config();

    uint32_t last_diff_ms = millis();
    uint32_t last_airspeed_error_ms = 0;
    uint32_t last_config_ms = millis();
    uint32_t last_stats_ms = millis();
    bool first_report_done = false;
    uint32_t next_pub_us = micros();
    uint32_t last_air_data_ms = millis();
    uint32_t last_temperature_ms = millis();

    while (true)
    {
        dronecan.cycle();
        IWatchdog.reload();

        uint32_t now_us = micros();
        uint32_t now_ms = millis();

        abs_update(now_us, now_ms);

        if (airspeed_active && now_ms - last_diff_ms >= 50)
        {
            diff_update(now_ms);
            last_diff_ms = now_ms;
        }

        if (now_ms - last_config_ms >= 1000)
        {
            const BaroConfig c = read_config();
            if (c != config) {
                apply_config(c);
            }
            update_airspeed_config(now_ms);
            update_raw_can_config();
            last_config_ms = now_ms;
        }

        if (airspeed_enabled && !airspeed_active && now_ms - last_airspeed_error_ms >= 10000)
        {
            dronecan.debug("Configure SENSOR_PRESSURE to 5, 10, 30, 60, or 100 (Based on your sensor)", 3);
            last_airspeed_error_ms = now_ms;
        }

        // One report 10 s after boot to confirm the real sample rate, then
        // every 10 s only if something went wrong.
        if (now_ms - last_stats_ms >= 10000)
        {
            report_stats(now_ms - last_stats_ms, !first_report_done);
            report_raw_can(now_ms - last_stats_ms);
            first_report_done = true;
            last_stats_ms = now_ms;
        }

        now_us = micros();
        now_ms = millis();
        const bool abs_ok = abs_healthy(now_ms);

        // Filtered static pressure on a steady grid; the 10 Hz average in
        // ArduPilot then sees the same number of samples every time.
        if ((int32_t)(now_us - next_pub_us) >= 0)
        {
            if (abs_ok)
            {
                uavcan_equipment_air_data_StaticPressure barometer_data{};
                barometer_data.static_pressure = (float)abs_filtered; // in Pascals
                sendUavcanMsg(dronecan, barometer_data, CANARD_TRANSFER_PRIORITY_HIGH);
                // The same value, at the same time, as an 11-bit frame.
                send_raw_can_filtered(barometer_data.static_pressure);
            }
            const uint32_t pub_period_us = 1000000UL / (uint32_t)config.pub_hz;
            next_pub_us += pub_period_us;
            if ((int32_t)(now_us - next_pub_us) >= 0) {
                next_pub_us = now_us + pub_period_us; // fell behind, resync
            }
        }

        // Temperature changes slowly; 10 Hz is plenty.
        if (now_ms - last_temperature_ms >= 100)
        {
            if (abs_ok)
            {
                uavcan_equipment_air_data_StaticTemperature temperature_data{};
                temperature_data.static_temperature = abs_temperature; // in Kelvin
                sendUavcanMsg(dronecan, temperature_data, CANARD_TRANSFER_PRIORITY_HIGH);
            }
            last_temperature_ms = now_ms;
        }

        // Send air data message 20 Hz, every 50ms
        if (now_ms - last_air_data_ms >= 50)
        {
            if (airspeed_active && diff_healthy(now_ms))
            {
                uavcan_equipment_air_data_RawAirData air_data{};
                air_data.differential_pressure = diff_pressure;                       // in Pascals
                air_data.differential_pressure_sensor_temperature = diff_temperature; // Kelvin
                air_data.static_pressure = abs_ok ? (float)abs_filtered : NAN;        // in Pascals
                air_data.static_pressure_sensor_temperature = abs_temperature;        // Kelvin
                sendUavcanMsg(dronecan, air_data, CANARD_TRANSFER_PRIORITY_HIGH);
            }
            last_air_data_ms = now_ms;
        }
    }
}

void loop()
{
    // Doesn't work coming from bootloader ? use while loop in setup
}

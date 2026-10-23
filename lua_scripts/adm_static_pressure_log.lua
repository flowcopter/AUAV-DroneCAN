--[[
  adm_static_pressure_log.lua - log and print the air data module's static
  pressure, received as DroneCAN messages (ArduPilot 4.7.0 or later).

  The script subscribes to uavcan.equipment.air_data.StaticPressure (data
  type ID 1028) on the flight controller's DroneCAN driver, so it sees every
  message the module publishes (50 Hz with the module's default BARO_PUB_HZ).
  ArduPilot's own baro driver still receives the same messages: DroneCAN
  broadcasts go to every subscriber, so the module stays your barometer.

  Requires the DroneCAN_Handle scripting API, added in ArduPilot 4.7.0.
  ArduPilot 4.6.x does not have it and the script stops with an error there.

  SETUP
    Nothing CAN-specific beyond the usual DroneCAN setup the baro already
    uses (CAN_P1_DRIVER = 1, CAN_D1_PROTOCOL = 1).
    SCR_ENABLE   = 1
    LOG_DISARMED = 1   (only if you want to log before arming)
  Reboot, copy this file to APM/scripts/ on the SD card, reboot again.

  LOG MESSAGE ADMP (one entry per message)
    Node   DroneCAN node ID of the sender
    DtUS   time since this node's previous message, us, from the DroneCAN
           receive timestamps; about 20000 at 50 Hz, larger values are gaps
    Press  static pressure, Pa
  TimeUS is when the script read the message, up to about ADMP_RUN_MS after
  it was received; use DtUS for the real spacing.

  GCS OUTPUT (every ADMP_RPT_S seconds)
    ADMP: node 50 101325.3 Pa 50.0 Hz
  or, if nothing arrived from the selected node(s):
    ADMP: no StaticPressure on driver 0

  PARAMETERS (added by the script; changes apply without a reboot, except
  ADMP_DRV which is read once at start)
    ADMP_ENABLE  1   0 stops logging and printing (messages are still drained)
    ADMP_NODE    0   0 = every node sending StaticPressure, otherwise only
                     this DroneCAN node ID (the module's NODEID)
    ADMP_RUN_MS  5   how often the script collects messages, ms (1-50). Up to
                     8 messages wait between runs, so keep
                     ADMP_RUN_MS x publish rate (Hz) / 1000 below 8.
    ADMP_RPT_S   1   seconds between GCS prints, 0 = off
    ADMP_DRV     0   DroneCAN driver index: 0 = the driver with CAN_D1_PROTOCOL
                     set to DroneCAN, 1 = CAN_D2_PROTOCOL
--]]

---@diagnostic disable: param-type-mismatch

local STATIC_PRESSURE_ID = 1028
local STATIC_PRESSURE_SIGNATURE = uint64_t(0xCDC7C434, 0x12BDC89A)
local MAX_PENDING = 8             -- messages a subscription can hold (DRONECAN_HANDLE_MAX_PAYLOADS)

local MAV_SEVERITY_ERROR = 3
local MAV_SEVERITY_WARNING = 4
local MAV_SEVERITY_INFO = 6

if not DroneCAN_Handle then
    gcs:send_text(MAV_SEVERITY_ERROR, "ADMP: needs ArduPilot 4.7 (no DroneCAN_Handle)")
    return
end

-- Parameter table. The key must be unique among the scripts on this vehicle;
-- 120 is not used by any script shipped with ArduPilot 4.7.2.
local PARAM_TABLE_KEY = 120
local PARAM_PREFIX = "ADMP_"
assert(param:add_table(PARAM_TABLE_KEY, PARAM_PREFIX, 5), "ADMP: could not add param table")

local function bind_param(index, name, default)
    assert(param:add_param(PARAM_TABLE_KEY, index, name, default),
           "ADMP: could not add param " .. PARAM_PREFIX .. name)
    return Parameter(PARAM_PREFIX .. name)
end

--[[
  // @Param: ADMP_ENABLE
  // @DisplayName: ADM pressure logging enable
  // @Description: Log and print StaticPressure messages from the air data module
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local ADMP_ENABLE = bind_param(1, "ENABLE", 1)

--[[
  // @Param: ADMP_NODE
  // @DisplayName: ADM pressure logging node ID
  // @Description: DroneCAN node ID to log; 0 logs every node that sends StaticPressure
  // @Range: 0 127
  // @User: Standard
--]]
local ADMP_NODE = bind_param(2, "NODE", 0)

--[[
  // @Param: ADMP_RUN_MS
  // @DisplayName: ADM pressure logging run period
  // @Description: How often the script collects messages. Up to 8 messages are held between runs.
  // @Units: ms
  // @Range: 1 50
  // @User: Advanced
--]]
local ADMP_RUN_MS = bind_param(3, "RUN_MS", 5)

--[[
  // @Param: ADMP_RPT_S
  // @DisplayName: ADM pressure print period
  // @Description: Seconds between pressure prints to the ground station; 0 disables them
  // @Units: s
  // @Range: 0 3600
  // @User: Standard
--]]
local ADMP_RPT_S = bind_param(4, "RPT_S", 1)

--[[
  // @Param: ADMP_DRV
  // @DisplayName: ADM pressure DroneCAN driver
  // @Description: DroneCAN driver index the module is on: 0 for the first DroneCAN driver. Read at script start.
  // @Range: 0 2
  // @User: Advanced
--]]
local ADMP_DRV = bind_param(5, "DRV", 0)

local driver_index = math.floor(ADMP_DRV:get())
local handle = DroneCAN_Handle(driver_index, STATIC_PRESSURE_SIGNATURE, STATIC_PRESSURE_ID)
if not handle or not handle:subscribe() then
    gcs:send_text(MAV_SEVERITY_ERROR, string.format(
        "ADMP: can't subscribe on DroneCAN driver %d", driver_index))
    return
end

local last_ts = {}        -- per node: receive timestamp of the previous message
local received = {}       -- per node: messages since the last print
local last_pressure = {}  -- per node: latest value, for the print
local last_report_ms = millis()

local function run_interval_ms()
    local ms = math.floor(ADMP_RUN_MS:get())
    return math.max(1, math.min(50, ms))
end

local function wanted(node)
    local target = math.floor(ADMP_NODE:get())
    return target <= 0 or node == target
end

-- payload: float32 static_pressure, float16 static_pressure_variance
local function handle_message(payload, node, timestamp)
    if #payload < 4 then
        return
    end
    local pressure = string.unpack("<f", payload)

    local dt_us = 0
    if last_ts[node] then
        dt_us = (timestamp - last_ts[node]):toint() or 0
    end
    last_ts[node] = timestamp
    received[node] = (received[node] or 0) + 1
    last_pressure[node] = pressure

    if wanted(node) then
        logger:write("ADMP", "Node,DtUS,Press", "BIf", "#sP", "-F0",
                     node, dt_us, pressure)
    end
end

local function reset_counts()
    for node in pairs(received) do
        received[node] = 0
    end
end

local function report()
    local now = millis()
    local elapsed_ms = (now - last_report_ms):toint()
    local report_ms = ADMP_RPT_S:get() * 1000
    if report_ms <= 0 then
        reset_counts()
        last_report_ms = now
        return
    end
    if elapsed_ms < report_ms then
        return
    end
    local any = false
    for node, count in pairs(received) do
        if wanted(node) and count > 0 then
            any = true
            gcs:send_text(MAV_SEVERITY_INFO, string.format("ADMP: node %d %.1f Pa %.1f Hz",
                          node, last_pressure[node], count * 1000.0 / elapsed_ms))
        end
    end
    if not any then
        local target = math.floor(ADMP_NODE:get())
        if target > 0 then
            gcs:send_text(MAV_SEVERITY_WARNING, string.format(
                "ADMP: no StaticPressure from node %d", target))
        else
            gcs:send_text(MAV_SEVERITY_WARNING, string.format(
                "ADMP: no StaticPressure on driver %d", driver_index))
        end
    end
    reset_counts()
    last_report_ms = now
end

local function update()
    local enabled = ADMP_ENABLE:get() > 0
    -- Always collect, so old messages are not logged when re-enabled.
    for _ = 1, MAX_PENDING do
        local payload, node, timestamp = handle:check_message()
        if not payload then
            break
        end
        if enabled then
            handle_message(payload, node, timestamp)
        end
    end
    if enabled then
        report()
    else
        -- Forget timestamps while off so the first DtUS after re-enabling
        -- does not span the time it was off.
        for node in pairs(last_ts) do
            last_ts[node] = nil
        end
        reset_counts()
        last_report_ms = millis()
    end
    return update, run_interval_ms()
end

gcs:send_text(MAV_SEVERITY_INFO, string.format(
    "ADMP: subscribed to StaticPressure on driver %d", driver_index))
return update, run_interval_ms()

--[[
  adm_static_pressure_log.lua - log and print the air data module's static
  pressure on the flight controller, one log entry per message received.

  Two sources, chosen by ADMP_SRC:

  1  DroneCAN: subscribe to uavcan.equipment.air_data.StaticPressure (ID 1028)
     on the DroneCAN driver. Needs ArduPilot 4.7.0 or later (DroneCAN_Handle).
     Nothing to set up on the module. Logged as ADMP.
  2  Raw 11-bit CAN frames: the module's RAWCAN output (node parameters
     RAWCAN_ENABLE = 1, RAWCAN_ID, RAWCAN_RAW_ID; layout in lib/RawCanFrame).
     Two frame kinds: the filtered pressure, sent with every StaticPressure
     at the module's BARO_PUB_HZ (logged as ADMR), and every unfiltered
     sample at the sample rate (logged as ADMU). Works on ArduPilot 4.6.
  0  Auto (default): DroneCAN if this firmware has DroneCAN_Handle, otherwise
     the raw frame.

  Either way ArduPilot's own baro driver keeps receiving StaticPressure, so
  the module stays your barometer.

  SETUP
    SCR_ENABLE = 1, LOG_DISARMED = 1 (only to log before arming).
    For the raw frame also: on the module RAWCAN_ENABLE = 1; on the flight
    controller CAN_D1_PROTOCOL2 = 10 (Scripting) on the DroneCAN driver. That
    hands the script every 11-bit frame on that port; DroneCAN itself is
    unaffected.
  Reboot (not just a scripting restart: the parameter table grew), copy this
  file to APM/scripts/ on the SD card, reboot again.

  LOG MESSAGES (one entry per message; TimeUS is when the script read it, up
  to about ADMP_RUN_MS later than it arrived)
    ADMP  DroneCAN source
      Node   DroneCAN node ID of the sender
      DtUS   time since this node's previous message, us, from the DroneCAN
             receive timestamps; about 20000 at 50 Hz, larger values are gaps
      Press  static pressure, Pa
    ADMR  raw source, filtered frame (ID ADMP_CAN_ID)
    ADMU  raw source, unfiltered frame (ID ADMP_RAW_ID), same fields:
      Seg    segment number, see below
      Idx    sample index within the segment: 0 for the segment's first
             frame, then +1 per frame the module sent, including missed ones
      Seq    the module's 8-bit sequence counter, 0-255
      Miss   frames missed since boot, from gaps in Seq
      Press  static pressure, Pa

    ADMR and ADMU have separate counters and rates, so entries with the same
    Idx are not the same moment; line the two up by TimeUS.
    For spectral analysis use ADMU's Idx, not TimeUS: the module sends one
    unfiltered frame per sample, so sample time is Idx / sample rate on its own
    clock (the module reports its measured rate as "BARO: N Hz measured"),
    whereas TimeUS carries up to ADMP_RUN_MS of polling jitter. Analyse each
    Seg separately. A new segment starts when the 8-bit Seq can no longer be
    trusted to count the gap: at the first frame, after ADMP_ENABLE was 0, and
    after more than SEGMENT_GAP_MS without a frame (256 frames at 180 Hz take
    1.4 s, so a longer gap could hide a whole wrap of Seq).

  GCS OUTPUT (every ADMP_RPT_S seconds)
    ADMP: node 50 101325.0 Pa 50.0 Hz               (DroneCAN)
    ADMP: filt 0x2A0 101325.0 Pa 50.0 Hz 0 miss     (raw source, one line
    ADMP: raw 0x2A1 101331.5 Pa 180.0 Hz 0 miss      per frame kind)
  or a warning naming the likely cause when nothing arrived.

  PARAMETERS (added by the script; ADMP_SRC and ADMP_DRV are read once at
  start, the others apply without a reboot)
    ADMP_ENABLE  1    0 stops logging and printing (messages are still drained)
    ADMP_NODE    0    DroneCAN only: 0 = every node, else only this node ID
    ADMP_RUN_MS  5    how often the script collects messages, ms (1-50)
    ADMP_RPT_S   1    seconds between GCS prints, 0 = off
    ADMP_DRV     0    DroneCAN only: DroneCAN driver index, 0 = first
    ADMP_SRC     0    0 auto, 1 DroneCAN, 2 raw 11-bit frame
    ADMP_CAN_ID  672  raw only: filtered frame ID, must equal the module's RAWCAN_ID
    ADMP_RAW_ID  673  raw only: unfiltered frame ID, must equal the module's
                      RAWCAN_RAW_ID; 0 ignores the unfiltered frames
--]]

---@diagnostic disable: param-type-mismatch

local STATIC_PRESSURE_ID = 1028
local STATIC_PRESSURE_SIGNATURE = uint64_t(0xCDC7C434, 0x12BDC89A)
local DRONECAN_MAX_PENDING = 8   -- messages a subscription can hold
local RAW_BUFFER_FRAMES = 25     -- maximum the scripting CAN buffer allows
local RAW_DLC = 5
local SEGMENT_GAP_MS = 1000      -- longer gap without a raw frame starts a new segment

local SRC_AUTO, SRC_DRONECAN, SRC_RAW = 0, 1, 2

local MAV_SEVERITY_ERROR = 3
local MAV_SEVERITY_WARNING = 4
local MAV_SEVERITY_INFO = 6

-- Parameter table. The key must be unique among the scripts on this vehicle;
-- 120 is not used by any script shipped with ArduPilot 4.6.3 or 4.7.2.
local PARAM_TABLE_KEY = 120
local PARAM_PREFIX = "ADMP_"
assert(param:add_table(PARAM_TABLE_KEY, PARAM_PREFIX, 8),
       "ADMP: could not add param table (reboot after updating the script)")

local function bind_param(index, name, default)
    assert(param:add_param(PARAM_TABLE_KEY, index, name, default),
           "ADMP: could not add param " .. PARAM_PREFIX .. name)
    return Parameter(PARAM_PREFIX .. name)
end

--[[
  // @Param: ADMP_ENABLE
  // @DisplayName: ADM pressure logging enable
  // @Description: Log and print the air data module's static pressure
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local ADMP_ENABLE = bind_param(1, "ENABLE", 1)

--[[
  // @Param: ADMP_NODE
  // @DisplayName: ADM pressure logging node ID
  // @Description: DroneCAN source only: node ID to log; 0 logs every node that sends StaticPressure
  // @Range: 0 127
  // @User: Standard
--]]
local ADMP_NODE = bind_param(2, "NODE", 0)

--[[
  // @Param: ADMP_RUN_MS
  // @DisplayName: ADM pressure logging run period
  // @Description: How often the script collects messages
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
  // @Description: DroneCAN source only: driver index the module is on, 0 for the first DroneCAN driver. Read at script start.
  // @Range: 0 2
  // @User: Advanced
--]]
local ADMP_DRV = bind_param(5, "DRV", 0)

--[[
  // @Param: ADMP_SRC
  // @DisplayName: ADM pressure source
  // @Description: Where to read the pressure: DroneCAN StaticPressure (ArduPilot 4.7+) or the module's raw 11-bit CAN frame. Auto uses DroneCAN when available. Read at script start.
  // @Values: 0:Auto,1:DroneCAN,2:Raw 11-bit CAN
  // @User: Standard
--]]
local ADMP_SRC = bind_param(6, "SRC", SRC_AUTO)

--[[
  // @Param: ADMP_CAN_ID
  // @DisplayName: ADM pressure raw CAN ID
  // @Description: Raw source only: 11-bit CAN ID of the module's pressure frame; must equal the module's RAWCAN_ID
  // @Range: 1 2047
  // @User: Standard
--]]
local ADMP_CAN_ID = bind_param(7, "CAN_ID", 0x2A0)

--[[
  // @Param: ADMP_RAW_ID
  // @DisplayName: ADM unfiltered pressure raw CAN ID
  // @Description: Raw source only: 11-bit CAN ID of the module's unfiltered pressure frame; must equal the module's RAWCAN_RAW_ID. 0 ignores the unfiltered frames.
  // @Range: 0 2047
  // @User: Standard
--]]
local ADMP_RAW_ID = bind_param(8, "RAW_ID", 0x2A1)

-- ---------------------------------------------------------------------------
-- Pick and open the source

local source = math.floor(ADMP_SRC:get())
if source == SRC_AUTO then
    source = DroneCAN_Handle and SRC_DRONECAN or SRC_RAW
end

local dronecan                  -- DroneCAN_Handle (DroneCAN source)
local can_buffer                -- ScriptingCANBuffer (raw source)
local driver_index = math.floor(ADMP_DRV:get())

if source == SRC_DRONECAN then
    if not DroneCAN_Handle then
        gcs:send_text(MAV_SEVERITY_ERROR, "ADMP: DroneCAN source needs ArduPilot 4.7")
        return
    end
    dronecan = DroneCAN_Handle(driver_index, STATIC_PRESSURE_SIGNATURE, STATIC_PRESSURE_ID)
    if not dronecan or not dronecan:subscribe() then
        gcs:send_text(MAV_SEVERITY_ERROR, string.format(
            "ADMP: can't subscribe on DroneCAN driver %d", driver_index))
        return
    end
elseif source == SRC_RAW then
    can_buffer = CAN:get_device(RAW_BUFFER_FRAMES)
    if not can_buffer then
        gcs:send_text(MAV_SEVERITY_ERROR, "ADMP: no scripting CAN, set CAN_D1_PROTOCOL2=10")
        return
    end
    -- Standard (11-bit) frames only: the extended-frame flag, bit 31 of the
    -- frame ID, must be clear. The IDs themselves are checked in the script
    -- so ADMP_CAN_ID and ADMP_RAW_ID can change at any time (a filter can't).
    can_buffer:add_filter(uint32_t(1) << 31, uint32_t(0))
else
    gcs:send_text(MAV_SEVERITY_ERROR, string.format("ADMP: invalid ADMP_SRC %d", source))
    return
end

-- ---------------------------------------------------------------------------
-- Shared state. Keys are DroneCAN node IDs, or the CAN ID for the raw source.

local received = {}       -- messages since the last print
local last_pressure = {}  -- latest value, for the print
local last_ts = {}        -- DroneCAN: receive timestamp of the previous message
-- Raw source: one stream per frame kind, each with its own sequence state.
--   last_seq  previous sequence counter; nil starts a new segment
--   missed    frames missed since boot
--   segment   current segment number (first segment is 1)
--   index     index of the latest frame within the segment
--   last_ms   when the previous frame was read
local function new_stream(log_name, label)
    return {log_name = log_name, label = label,
            last_seq = nil, missed = 0, segment = 0, index = 0, last_ms = nil}
end
local streams = {
    filtered = new_stream("ADMR", "filt"),
    unfiltered = new_stream("ADMU", "raw"),
}

-- Current frame IDs; the unfiltered ID is nil when ignored or the same as
-- the filtered one.
local function raw_ids()
    local filtered_id = math.floor(ADMP_CAN_ID:get())
    local unfiltered_id = math.floor(ADMP_RAW_ID:get())
    if unfiltered_id <= 0 or unfiltered_id == filtered_id then
        unfiltered_id = nil
    end
    return filtered_id, unfiltered_id
end
local last_report_ms = millis()

local function run_interval_ms()
    local ms = math.floor(ADMP_RUN_MS:get())
    return math.max(1, math.min(50, ms))
end

local function wanted_node(node)
    local target = math.floor(ADMP_NODE:get())
    return target <= 0 or node == target
end

local function count(key, pressure)
    received[key] = (received[key] or 0) + 1
    last_pressure[key] = pressure
end

-- DroneCAN payload: float32 static_pressure, float16 static_pressure_variance
local function handle_dronecan(payload, node, timestamp)
    if #payload < 4 then
        return
    end
    local pressure = string.unpack("<f", payload)
    local dt_us = 0
    if last_ts[node] then
        dt_us = (timestamp - last_ts[node]):toint() or 0
    end
    last_ts[node] = timestamp
    count(node, pressure)
    if wanted_node(node) then
        logger:write("ADMP", "Node,DtUS,Press", "BIf", "#sP", "-F0", node, dt_us, pressure)
    end
end

-- Raw frame: float32 pressure little-endian, uint8 sequence counter
local function handle_raw(frame, filtered_id, unfiltered_id)
    if frame:isExtended() or frame:dlc() ~= RAW_DLC then
        return
    end
    local id = frame:id_signed()
    local stream
    if id == filtered_id then
        stream = streams.filtered
    elseif id == unfiltered_id then
        stream = streams.unfiltered
    else
        return
    end
    local pressure = string.unpack("<f", string.char(
        frame:data(0), frame:data(1), frame:data(2), frame:data(3)))
    local seq = frame:data(4)
    local now = millis()
    if stream.last_seq and stream.last_ms and (now - stream.last_ms):toint() > SEGMENT_GAP_MS then
        stream.last_seq = nil   -- too long to trust the 8-bit counter across the gap
    end
    stream.last_ms = now
    if stream.last_seq then
        local step = (seq - stream.last_seq) % 256
        if step == 0 then
            return              -- the same frame again; nothing new
        end
        stream.missed = stream.missed + step - 1
        stream.index = stream.index + step
    else
        stream.segment = stream.segment + 1
        stream.index = 0
    end
    stream.last_seq = seq
    count(stream.label, pressure)
    logger:write(stream.log_name, "Seg,Idx,Seq,Miss,Press", "IIBIf", "----P", "----0",
                 stream.segment, stream.index, seq, stream.missed, pressure)
end

local function collect(enabled)
    if dronecan then
        for _ = 1, DRONECAN_MAX_PENDING do
            local payload, node, timestamp = dronecan:check_message()
            if not payload then
                break
            end
            if enabled then
                handle_dronecan(payload, node, timestamp)
            end
        end
    else
        local filtered_id, unfiltered_id = raw_ids()
        for _ = 1, RAW_BUFFER_FRAMES do
            local frame = can_buffer:read_frame()
            if not frame then
                break
            end
            if enabled then
                handle_raw(frame, filtered_id, unfiltered_id)
            end
        end
    end
end

-- ---------------------------------------------------------------------------
-- GCS prints

local function reset_counts()
    for key in pairs(received) do
        received[key] = 0
    end
end

local function print_dronecan(elapsed_ms)
    local any = false
    for node, n in pairs(received) do
        if wanted_node(node) and n > 0 then
            any = true
            gcs:send_text(MAV_SEVERITY_INFO, string.format("ADMP: node %d %.1f Pa %.1f Hz",
                          node, last_pressure[node], n * 1000.0 / elapsed_ms))
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
end

local function print_raw(elapsed_ms)
    local filtered_id, unfiltered_id = raw_ids()
    local function line(stream, id)
        local n = received[stream.label] or 0
        if n > 0 then
            gcs:send_text(MAV_SEVERITY_INFO, string.format("ADMP: %s 0x%03X %.1f Pa %.1f Hz %d miss",
                          stream.label, id, last_pressure[stream.label],
                          n * 1000.0 / elapsed_ms, stream.missed))
        end
        return n > 0
    end
    local got_filtered = line(streams.filtered, filtered_id)
    local got_unfiltered = unfiltered_id and line(streams.unfiltered, unfiltered_id)
    if not got_filtered and not got_unfiltered then
        gcs:send_text(MAV_SEVERITY_WARNING, string.format("ADMP: no frames with ID 0x%03X", filtered_id))
        gcs:send_text(MAV_SEVERITY_WARNING, "ADMP: module RAWCAN_ENABLE=1, same RAWCAN_ID?")
    elseif not got_filtered then
        gcs:send_text(MAV_SEVERITY_WARNING, string.format("ADMP: no filtered frames on 0x%03X", filtered_id))
    elseif unfiltered_id and not got_unfiltered then
        gcs:send_text(MAV_SEVERITY_WARNING, string.format("ADMP: no unfiltered frames on 0x%03X", unfiltered_id))
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
    if dronecan then
        print_dronecan(elapsed_ms)
    else
        print_raw(elapsed_ms)
    end
    reset_counts()
    last_report_ms = now
end

-- ---------------------------------------------------------------------------

local function update()
    local enabled = ADMP_ENABLE:get() > 0
    -- Always collect, so old messages are not logged when re-enabled.
    collect(enabled)
    if enabled then
        report()
    else
        -- Forget timing and sequence state while off, so the time spent off
        -- shows neither as a DtUS gap nor as missed frames.
        for node in pairs(last_ts) do
            last_ts[node] = nil
        end
        for _, stream in pairs(streams) do
            stream.last_seq = nil
        end
        reset_counts()
        last_report_ms = millis()
    end
    return update, run_interval_ms()
end

if dronecan then
    gcs:send_text(MAV_SEVERITY_INFO, string.format(
        "ADMP: DroneCAN StaticPressure, driver %d", driver_index))
else
    local filtered_id, unfiltered_id = raw_ids()
    if unfiltered_id then
        gcs:send_text(MAV_SEVERITY_INFO, string.format(
            "ADMP: raw CAN, filt 0x%03X, unfilt 0x%03X", filtered_id, unfiltered_id))
    else
        gcs:send_text(MAV_SEVERITY_INFO, string.format(
            "ADMP: raw CAN, filt 0x%03X", filtered_id))
    end
end
return update, run_interval_ms()

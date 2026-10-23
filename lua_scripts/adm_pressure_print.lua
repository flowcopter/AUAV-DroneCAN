--[[
  adm_pressure_print.lua - print the air data module's static pressure,
  received as a custom 11-bit CAN frame (ArduPilot 4.6.3).

  FRAME (sent by the module when its node parameter RAWCAN_ENABLE = 1, with
  the ID in its node parameter RAWCAN_ID; layout in lib/RawCanFrame)
    ID      11-bit standard, must equal ADMR_CAN_ID
    DLC     5
    0-3     static pressure, Pa, float32 little-endian (filtered value)
    4       sequence counter 0-255, +1 per frame

  SETUP
  The frame is 11-bit, so the script can share the DroneCAN port: in 4.6.3
  a secondary protocol on a DroneCAN port receives every 11-bit frame.
    CAN_D1_PROTOCOL  = 1   (DroneCAN, unchanged; the module stays your baro)
    CAN_D1_PROTOCOL2 = 10  (Scripting)
    SCR_ENABLE       = 1
  Reboot, copy this file to APM/scripts/ on the SD card, reboot again.

  PARAMETERS (added by the script; changes apply without a reboot)
    ADMR_CAN_ID    2030 (0x7EE) 11-bit CAN ID of the filtered frame, 1-2031
    ADMR_PRINT_MS  1000         how often to print, ms; 0 = never

  OUTPUT
  Every ADMR_PRINT_MS the latest value goes to the ground station:
    ADM: 101325.3 Pa, 50.0 Hz, 0 missed
  or, if no frame with that ID arrived in that time:
    ADM: no frames with ID 0x7EE
--]]

---@diagnostic disable: param-type-mismatch

local BUFFER_FRAMES = 25          -- maximum the scripting CAN buffer allows
local RUN_INTERVAL_MS = 5         -- 50 Hz frames: about one per four runs

local MAV_SEVERITY_ERROR = 3
local MAV_SEVERITY_WARNING = 4
local MAV_SEVERITY_INFO = 6

-- Parameter table. The key must be unique among the scripts on this vehicle;
-- 121 is not used by any script shipped with ArduPilot 4.6.3.
local PARAM_TABLE_KEY = 121
local PARAM_PREFIX = "ADMR_"
assert(param:add_table(PARAM_TABLE_KEY, PARAM_PREFIX, 2), "ADMR: could not add param table")

local function bind_param(index, name, default)
    assert(param:add_param(PARAM_TABLE_KEY, index, name, default),
           "ADMR: could not add param " .. PARAM_PREFIX .. name)
    return Parameter(PARAM_PREFIX .. name)
end

--[[
  // @Param: ADMR_CAN_ID
  // @DisplayName: ADM pressure CAN ID
  // @Description: 11-bit CAN ID of the air data module's pressure frame; must match the module's setting
  // @Range: 1 2031
  // @User: Standard
--]]
local ADMR_CAN_ID = bind_param(1, "CAN_ID", 0x7EE)

--[[
  // @Param: ADMR_PRINT_MS
  // @DisplayName: ADM pressure print period
  // @Description: How often to print the latest pressure to the ground station; 0 disables printing
  // @Units: ms
  // @Range: 0 60000
  // @User: Standard
--]]
local ADMR_PRINT_MS = bind_param(2, "PRINT_MS", 1000)

-- No CAN filter: the ID is checked below instead, so ADMR_CAN_ID can change
-- at any time (a filter cannot be changed once added). On a DroneCAN port
-- this buffer only receives 11-bit frames, which are rare there.
local driver = CAN:get_device(BUFFER_FRAMES)
if not driver then
    gcs:send_text(MAV_SEVERITY_ERROR, "ADMR: no scripting CAN, set CAN_D1_PROTOCOL2=10")
    return
end

local last_pressure = nil
local last_seq = nil
local count = 0              -- frames since the last print
local missed = 0             -- frames missed since boot, from sequence gaps
local last_print_ms = millis()

-- Returns pressure and sequence number, or nil if the frame is not ours.
local function decode(frame, can_id)
    if frame:isExtended() or frame:id_signed() ~= can_id then
        return nil
    end
    if frame:dlc() ~= 5 then
        return nil
    end
    local pressure = string.unpack("<f", string.char(
        frame:data(0), frame:data(1), frame:data(2), frame:data(3)))
    return pressure, frame:data(4)
end

local function print_status()
    local print_ms = ADMR_PRINT_MS:get()
    local now = millis()
    local elapsed_ms = (now - last_print_ms):toint()
    if print_ms <= 0 then
        count = 0
        last_print_ms = now
        return
    end
    if elapsed_ms < print_ms then
        return
    end
    if count > 0 then
        gcs:send_text(MAV_SEVERITY_INFO, string.format("ADM: %.1f Pa, %.1f Hz, %d missed",
                      last_pressure, count * 1000.0 / elapsed_ms, missed))
    else
        gcs:send_text(MAV_SEVERITY_WARNING, string.format("ADM: no frames with ID 0x%03X",
                      math.floor(ADMR_CAN_ID:get())))
    end
    count = 0
    last_print_ms = now
end

local function update()
    local can_id = math.floor(ADMR_CAN_ID:get())
    for _ = 1, BUFFER_FRAMES do
        local frame = driver:read_frame()
        if not frame then
            break
        end
        local pressure, seq = decode(frame, can_id)
        if pressure then
            if last_seq then
                missed = missed + (seq - last_seq - 1) % 256
            end
            last_seq = seq
            last_pressure = pressure
            count = count + 1
        end
    end
    print_status()
    return update, RUN_INTERVAL_MS
end

gcs:send_text(MAV_SEVERITY_INFO, "ADMR: listening for ADM pressure frames")
return update, RUN_INTERVAL_MS

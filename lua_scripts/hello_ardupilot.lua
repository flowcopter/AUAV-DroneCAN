--[[
  hello_ardupilot.lua - minimal ArduPilot 4.6.3 Lua script.

  Shows the basic pattern every ArduPilot script follows:
    - a function that does one step of work and then returns itself plus the
      delay (in ms) before it should run again
    - gcs:send_text() to send messages to the ground station
    - param:get() to read a parameter
    - baro / arming to read vehicle state
    - logger:write() to add a message to the onboard log (TimeUS is added
      automatically)

  Setup: SCR_ENABLE = 1, reboot, copy this file to APM/scripts/ on the SD card,
  reboot again. Messages show in the ground station's Messages tab.
  API reference: libraries/AP_Scripting/docs/docs.lua in the 4.6.3 source.
--]]

local MAV_SEVERITY_INFO = 6
local RUN_INTERVAL_MS = 1000

local count = 0

local function update()
    count = count + 1

    local pressure = baro:get_pressure()   -- primary baro, Pa
    local armed = arming:is_armed()
    local primary = param:get("BARO_PRIMARY")

    gcs:send_text(MAV_SEVERITY_INFO, string.format(
        "hello #%d: baro %.1f Pa (BARO_PRIMARY=%d), %s",
        count, pressure, primary or -1, armed and "armed" or "disarmed"))

    -- Log message HELO: Count, Press (Pa), Armed
    logger:write("HELO", "Count,Press,Armed", "IfB", "-P-", "-0-",
                 count, pressure, armed and 1 or 0)

    return update, RUN_INTERVAL_MS
end

gcs:send_text(MAV_SEVERITY_INFO, "hello_ardupilot.lua loaded")
return update, RUN_INTERVAL_MS

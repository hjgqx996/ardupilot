--[[
Script to control LED strips based on the roll of the aircraft. This is an example to demonstrate
the LED interface for WS2812 LEDs
--]]

local num_leds = 16 -- assume 16 LEDs in each strip
local chan_left = 9 -- LEDs for left wing on channel 9
local chan_right = 10 -- LEDs for right wing on channel 10

-- initialisation code
serialLED:set_num_LEDs(chan_left,  num_leds)
serialLED:set_num_LEDs(chan_right, num_leds)

-- constrain a value between limits
function constrain(v, vmin, vmax)
   if v < vmin then
      v = vmin
   end
   if v > vmax then
      v = vmax
   end
   return v
end

function update_LEDs()

  -- 16 LEDs per wing, two channels
  local roll = ahrs:get_roll_degrees()

  for led = 0, num_leds-1 do
    local v_left  = constrain(0.5 * (1 - (roll/45) * (led / num_leds)), 0, 1)
    local v_right = constrain(0.5 * (1 + (roll/45) * (led / num_leds)), 0, 1)

    serialLED:set_Rainbow(chan_left, 1<<led, v_left)
    serialLED:set_Rainbow(chan_right,1<<led, v_right)
  end
  serialLED:send()

  return update_LEDs, 50 -- run at 20Hz
end

return update_LEDs, 1000


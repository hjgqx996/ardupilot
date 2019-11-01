--[[
Script to control LED strips based on the roll of the aircraft. This is an example to demonstrate
the LED interface for WS2812 LEDs
--]]

local num_leds = 16 -- assume 16 LEDs in each strip
local chan_left = 0
local chan_right = 0

-- use SERVOn_FUNCTION 120 and 121 for left and right LEDs
chan_left = SRV_Channels:find_channel(120)
chan_right = SRV_Channels:find_channel(121)

gcs:send_text(6, string.format("LEDs: chan_left=" .. tostring(chan_left)))
gcs:send_text(6, string.format("LEDs: chan_right=" .. tostring(chan_right)))

if not (chan_left and chan_right) then
    gcs:send_text(6, string.format("LEDs: channels not set"))
    return
end

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

local rainbow = {    
  { 255, 0, 0 },    
  { 255, 127, 0 },    
  { 255, 255, 0 },    
  { 0,   255, 0 },    
  { 0,   0,   255 },    
  { 75,  0,   130 },    
  { 143, 0,   255 },    
}

function set_Rainbow(chan, led, v)    
  local num_rows = #rainbow
  local row = constrain(v * num_rows, 1, num_rows)
  local v0 = row / num_rows
  local v1 = (row+1) / num_rows
  local p = (v - v0) / (v1 - v0)
  serialLED:set_RGB(chan,
                    1 << led,
                    (rainbow[row].rgb[0] + p * (float(rainbow[row+1].rgb[0]) - float(rainbow[row].rgb[0]))),
                    (rainbow[row].rgb[1] + p * (float(rainbow[row+1].rgb[1]) - float(rainbow[row].rgb[1]))),
                    (rainbow[row].rgb[2] + p * (float(rainbow[row+1].rgb[2]) - float(rainbow[row].rgb[2]))))
end

function update_LEDs()

  -- 16 LEDs per wing, two channels
  local roll = ahrs:get_roll_degrees()

  for led = 0, num_leds-1 do
    local v_left  = constrain(0.5 * (1 - (roll/45) * (led / num_leds)), 0, 1)
    local v_right = constrain(0.5 * (1 + (roll/45) * (led / num_leds)), 0, 1)

    serialLED:set_Rainbow(chan_left, led, v_left)
    serialLED:set_Rainbow(chan_right,led, v_right)
  end
  serialLED:send()

  return update_LEDs, 50 -- run at 20Hz
end

return update_LEDs, 1000


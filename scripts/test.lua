gcs.send_text("Initial run")
local val = 1000
local step = 1

function adjust_servo (value)
  servo.set_output_pwm(96, value)
end

function inc_print ()
  if val >= 2000 then
    step = -1
  elseif val <= 1000 then
    step = 1
  end
  val = val + step
  gcs.send_text(string.format("Servo: %f", val))
  adjust_servo(val)
  return inc_print, 200 * math.random()
end

return inc_print, 1000

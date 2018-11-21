gcs.send_text("Initial run")
i = 0

function inc_print ()
  i = 1 + i
  gcs.send_text(string.format("Run B: %f", i))
  return inc_print, (2000 * math.random())//1
end

return inc_print, 1000

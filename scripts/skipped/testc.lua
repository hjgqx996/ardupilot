gcs.send_text("Initial run")
i = 0

function inc_print ()
  gcs.send_text("Subs run")
  return
end

return inc_print, 10

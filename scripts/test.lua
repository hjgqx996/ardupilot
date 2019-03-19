function update () 
    foo = Location()
    bar = Location()
    if ahrs:get_position(foo) and ahrs:get_position(bar) then
        gcs.send_text(string.format("Got %d %d", foo:lat(), foo:lng()))
        bar:offset(100, 100);
        gcs.send_text(string.format("Bar is %f meters from foo", bar:get_distance(foo)))
    end
    notify:play_tune("MBNT90L4O2A#O3DFN0N0N0")
    return update, 5000
end

return update, 1000

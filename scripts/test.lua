local ACCEPTANCE_DISTANCE = 5.0

local TUNE_POINT = "MFT200L8G>C3"
local TUNE_AWAY = "MFT100L4>B#A#2P8B#A#2"
local TUNE_TOWARDS = "MFT100L4>A#B#"
local TUNE_IDLE = "MFT100L8>B"

local target = Location()
local top_left = Location()
top_left:lat(-353613548)
top_left:lng(1491623648)

local last_update = Location()
local last_distance = 1e10
local score = 0

function find_next_point ()
    target:lat(top_left:lat())
    target:lng(top_left:lng())
    target:offset(math.random()*1000, math.random()*1000)
    gcs.send_text(string.format("New target %d %d", target:lat(), target:lng()))
    local current = Location()
    if ahrs:get_position(current) then
        last_distance = current:get_distance(target)
    end
    last_distance = 1e10
    return
end

function update () 
    local current = Location()
    if ahrs:get_position(current) then
        local dist = last_update:get_distance(current)
        if target:get_distance(current) < ACCEPTANCE_DISTANCE then
            notify:play_tune(TUNE_POINT)
            score = score + 1
            gcs.send_text(string.format("Got a point! %d total", score))
            find_next_point()
        elseif dist < (last_distance - 1) then
            notify:play_tune(TUNE_TOWARDS)
        elseif dist > (last_distance + 1) then
            notify:play_tune(TUNE_AWAYS)
        else
            notify:play_tune(TUNE_IDLE)
        end
    end
    return update, 5000
end

find_next_point()

return update, 1000

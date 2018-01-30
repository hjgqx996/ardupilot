#include "AP_Script.h"
#include "lua_bindings.h"

#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL &hal;

void AP_Script::init(void) {
    if (state != nullptr) {
        return;
    }

    state = luaL_newstate();
    luaL_openlibs(state);

    load_lua_bindings(state);
}

bool AP_Script::run_script(const char *script) {
    // FIXME: This should not be called if another script is running already
    if(luaL_dostring(state, script)) {
        gcs().send_text(MAV_SEVERITY_INFO, "Lua: %s", lua_tostring(state, -1));
        hal.console->printf("Lua: %s", lua_tostring(state, -1));
        return false;
    }
    return true;
}

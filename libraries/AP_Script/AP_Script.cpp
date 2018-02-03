#include "AP_Script.h"
#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"

#include "lua_bindings.h"

#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL &hal;

void AP_Script::init(const char *script) {
    if (state != nullptr) {
        return;
    }

    state = luaL_newstate();
    luaL_openlibs(state);

    load_lua_bindings(state);
    luaL_loadstring(state, script);
}

bool AP_Script::run_script(const char *script) {
    // FIXME: This should not be called if another script is running already
    uint32_t startMem = hal.util->available_memory();
    uint32_t loadEnd = AP_HAL::micros();
    lua_pushvalue(state, -1);
    if(lua_pcall(state, 0, LUA_MULTRET, 0)) {
        gcs().send_text(MAV_SEVERITY_INFO, "Lua: %s", lua_tostring(state, -1));
        hal.console->printf("Lua: %s", lua_tostring(state, -1));
        return false;
    }
    uint32_t runEnd = AP_HAL::micros();
    uint32_t endMem = hal.util->available_memory();
    gcs().send_text(MAV_SEVERITY_INFO, "Execution: %d Memory: %d", runEnd - loadEnd, startMem - endMem);
    return true;
}

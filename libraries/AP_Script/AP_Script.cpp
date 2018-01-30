#include "AP_Script.h"
#include <GCS_MAVLink/GCS.h>

int l_send_text(lua_State *L) {
    const char* str = lua_tostring(L, 1);
    gcs().send_text(MAV_SEVERITY_INFO, str);
    return 0;
}


void AP_Script::init(void) {
    if (state != nullptr) {
        return;
    }

    state = luaL_newstate();
    luaL_openlibs(state);

    // FIXME: Load ArduPilot bindings
    lua_pushcfunction(state, l_send_text);
    lua_setglobal(state, "sendText");
}

int AP_Script::run_script(const char *script) {
    // FIXME: This should not be called if another script is running already
    return luaL_dostring(state, script);
}

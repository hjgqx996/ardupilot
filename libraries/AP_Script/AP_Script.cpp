#include "AP_Script.h"

extern "C" {
#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"
}

extern const AP_HAL::HAL& hal;

AP_Script::AP_Script(void) {
    lua_State *state = luaL_newstate();
    luaL_openlibs(state);
    luaL_dostring(state,"local a=5 local b=10 local c=a+b return c");
    int r = luaL_checkinteger(state, -1);
    gcs().send_text(MAV_SEVERITY_INFO, "Got %d", r);
    lua_close(state);
}


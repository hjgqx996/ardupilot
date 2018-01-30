#include "AP_Script.h"

extern "C" {
#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"
}

extern const AP_HAL::HAL& hal;

int l_send_text(lua_State *L) {
    const char* str = lua_tostring(L, 1);
    gcs().send_text(MAV_SEVERITY_INFO, str);
    return 0;
}

AP_Script::AP_Script(void) {
    lua_State *state = luaL_newstate();
    luaL_openlibs(state);
    lua_pushcfunction(state, l_send_text);
    lua_setglobal(state, "sendText");
    luaL_dostring(state,"local a=5 local b=10 local c=a+b sendText(string.format(\"testing math %d\", c)) return c+2");
    int r = luaL_checkinteger(state, -1);
    gcs().send_text(MAV_SEVERITY_INFO, "Return value of %d", r);
    lua_close(state);
}


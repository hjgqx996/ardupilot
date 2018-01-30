#include "lua_bindings.h"

#include <GCS_MAVLink/GCS.h>

int lua_send_text(lua_State *state) {
    const char* str = lua_tostring(state, 1);
    gcs().send_text(MAV_SEVERITY_INFO, str);
    return 0;
}

static const luaL_Reg gcs_functions[] =
{
    {"send_text", lua_send_text},
    {NULL, NULL}
};

void load_lua_bindings(lua_State *state) {
    luaL_newlib(state, gcs_functions);
    lua_setglobal(state, "gcs");
}

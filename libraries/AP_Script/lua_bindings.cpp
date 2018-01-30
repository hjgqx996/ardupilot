#include "lua_bindings.h"

#include <AP_Common/AP_Common.h>
#include <AP_GPS/AP_GPS.h>
#include <GCS_MAVLink/GCS.h>

void push_location(lua_State *state, const Location &loc) {
    lua_createtable(state, 0, 3);
    lua_pushstring(state, "lat");
    lua_pushinteger(state, loc.lat);
    lua_settable(state, -3);

    lua_pushstring(state, "lng");
    lua_pushinteger(state, loc.lng);
    lua_settable(state, -3);

    lua_pushstring(state, "alt");
    lua_pushinteger(state, loc.alt);
    lua_settable(state, -3);
}

int lua_gps_get_location(lua_State *state) {
    push_location(state, AP::gps().location());
    return 1;
}

static const luaL_Reg gps_functions[] =
{
    {"location", lua_gps_get_location},
    {NULL, NULL}
};


int lua_gcs_send_text(lua_State *state) {
    const char* str = lua_tostring(state, 1);
    gcs().send_text(MAV_SEVERITY_INFO, str);
    return 0;
}

static const luaL_Reg gcs_functions[] =
{
    {"send_text", lua_gcs_send_text},
    {NULL, NULL}
};

void load_lua_bindings(lua_State *state) {
    luaL_newlib(state, gps_functions);
    lua_setglobal(state, "gps");
    luaL_newlib(state, gcs_functions);
    lua_setglobal(state, "gcs");
}

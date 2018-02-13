#include <AP_Common/AP_Common.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS.h>

#include "lua_bindings.h"

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

int lua_param_get(lua_State *state) {
    if (lua_type(state, 1) != LUA_TSTRING) {
        // don't convert things that aren't strings
        // FIXME: Should this raise a runtime error instead?
        lua_pushboolean(state, false);
        lua_pushnumber(state, 0.0f);
        return 2;
    }
    const char* str = lua_tostring(state, 1);
    enum ap_var_type var_type;
    AP_Param *param = AP_Param::find(str, &var_type);
    if (param == nullptr) {
        // no param by that name, return a failure
        lua_pushboolean(state, false);
        lua_pushnumber(state, 0.0f);
        return 2;
    }
    lua_pushboolean(state, true);
    lua_pushnumber(state, param->cast_to_float(var_type));
    return 2;
}

static const luaL_Reg param_functions[] =
{
    {"get", lua_param_get},
//    {"set", lua_param_set},
    {NULL, NULL}
};

void load_lua_bindings(lua_State *state) {
    luaL_newlib(state, gps_functions);
    lua_setglobal(state, "gps");
    luaL_newlib(state, gcs_functions);
    lua_setglobal(state, "gcs");
    luaL_newlib(state, param_functions);
    lua_setglobal(state, "param");
}

void hook(lua_State *L, lua_Debug *ar) {
    gcs().send_text(MAV_SEVERITY_INFO, "got a debug hook");
    lua_error(L);
}

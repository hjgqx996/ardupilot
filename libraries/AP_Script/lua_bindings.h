#pragma once

#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"

// load all known lua bindings into the state
void load_lua_bindings(lua_State *state);

#pragma once

extern "C" {
#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"
}

class AP_Script {

public:
    AP_Script(void) {};

    void init(void);

    int run_script(const char *script);

private:
    lua_State *state;

};

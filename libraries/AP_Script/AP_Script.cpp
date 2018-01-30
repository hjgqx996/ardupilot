#include "AP_Script.h"
#include "lua_bindings.h"

void AP_Script::init(void) {
    if (state != nullptr) {
        return;
    }

    state = luaL_newstate();
    luaL_openlibs(state);

    load_lua_bindings(state);
}

int AP_Script::run_script(const char *script) {
    // FIXME: This should not be called if another script is running already
    return luaL_dostring(state, script);
}

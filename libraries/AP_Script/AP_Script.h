#pragma once

struct lua_State;
class AP_Script {

public:
    AP_Script(void) {};

    void init(void);

    bool run_script(const char *script);

private:
    lua_State *state;

};

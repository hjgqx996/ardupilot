#pragma once
/*
// workaround lack of signals on non linux boards
#define sig_atomic_t lu_byte
#define signal(sig, func) do {} while(0)
#define SIGINT 0

// workaround for missing cstdio
*/
#define L_tmpnam 64
#define BUFSIZ 128

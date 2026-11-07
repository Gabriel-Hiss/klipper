// Wire protocol between libvclock.so (one connection per registered thread) and vclockd.
#ifndef VCLOCK_H
#define VCLOCK_H

#include <stdint.h>

#define VC_INF UINT64_MAX
#define VC_REASON_LEN 48

enum vc_msg_type {
    // thread -> daemon
    VC_REGISTER = 1, // a=pid, b=tid; reply VC_WELCOME
    VC_BLOCK,        // a=deadline_ns (VC_INF = none), reason; gives up the baton
    VC_BUSY,         // reply to VC_TRY: still waiting
    VC_RUNNING,      // reply to VC_TRY: proceeding, now holds the baton
    VC_EXIT,         // a=status; thread leaves (gives up the baton if held)
    VC_EXEC,         // a=pid; next VC_REGISTER from this pid inherits this slot
    // daemon -> thread
    VC_WELCOME = 100, // a=slot id, b=vnow_ns
    VC_TRY,           // b=vnow_ns: re-check the wait condition
};

struct vc_msg {
    uint32_t type;
    uint32_t pad;
    uint64_t a;
    uint64_t b;
    char reason[VC_REASON_LEN];
};

#endif

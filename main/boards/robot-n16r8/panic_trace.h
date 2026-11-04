#ifndef _ROBOT_N16R8_PANIC_TRACE_H_
#define _ROBOT_N16R8_PANIC_TRACE_H_

#include <cstdint>

// Last crash, kept in RTC memory across the reset (see panic_trace.cc).
struct PanicTrace {
    static constexpr int kMaxDepth = 24;
    uint32_t magic;
    int core;
    uint32_t addr;
    char reason[96];
    char task[16];
    uint32_t uptime_ms;  // when the crash happened (tells a stale record from a new one)
    int depth;
    uint32_t pcs[kMaxDepth];
};

extern PanicTrace g_panic_trace;
bool PanicTraceValid();

#endif  // _ROBOT_N16R8_PANIC_TRACE_H_

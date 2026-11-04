// Crash recorder (diagnostics only): the robot has no USB log attached and no coredump
// partition, so the panic handler is wrapped (-Wl,--wrap=esp_panic_handler) to keep the
// reason, task and backtrace PCs in RTC memory, which survives the software reset after the
// panic. The console 'crash' command prints them; decode with xtensa-esp32s3-elf-addr2line.
#include "panic_trace.h"

#include <cstring>

#include <esp_attr.h>
#include <esp_cpu_utils.h>
#include <esp_debug_helpers.h>
#include <esp_private/panic_internal.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <xtensa_context.h>

namespace {
constexpr uint32_t kMagic = 0x4A415256;  // "JARV"
}

RTC_NOINIT_ATTR PanicTrace g_panic_trace;

extern "C" void __real_esp_panic_handler(panic_info_t* info);

extern "C" void __wrap_esp_panic_handler(panic_info_t* info) {
    PanicTrace& t = g_panic_trace;
    std::memset(&t, 0, sizeof(t));
    t.core = info->core;
    t.addr = reinterpret_cast<uint32_t>(info->addr);
    const char* reason = g_panic_abort && g_panic_abort_details ? g_panic_abort_details
                                                                : info->reason;
    if (reason != nullptr) {
        std::strncpy(t.reason, reason, sizeof(t.reason) - 1);
    }
    TaskHandle_t task = xTaskGetCurrentTaskHandleForCore(info->core);
    if (task != nullptr) {
        std::strncpy(t.task, pcTaskGetName(task), sizeof(t.task) - 1);
    }
    const XtExcFrame* frame = static_cast<const XtExcFrame*>(info->frame);
    if (frame != nullptr) {
        esp_backtrace_frame_t bt = {};
        bt.pc = frame->pc;
        bt.sp = frame->a1;
        bt.next_pc = frame->a0;
        bt.exc_frame = const_cast<XtExcFrame*>(frame);
        t.pcs[t.depth++] = esp_cpu_process_stack_pc(bt.pc);
        while (t.depth < PanicTrace::kMaxDepth && bt.next_pc != 0 &&
               esp_backtrace_get_next_frame(&bt)) {
            t.pcs[t.depth++] = esp_cpu_process_stack_pc(bt.pc);
        }
    }
    t.magic = kMagic;
    __real_esp_panic_handler(info);
}

bool PanicTraceValid() { return g_panic_trace.magic == kMagic; }

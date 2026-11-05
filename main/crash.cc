#include "crash.h"

#include <cstdio>
#include <cstring>

#include <esp_attr.h>
#include <esp_cpu_utils.h>
#include <esp_debug_helpers.h>
#include <esp_private/panic_internal.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <xtensa_context.h>

namespace {
constexpr uint32_t kMagic = 0x4A475243;  // "JGRC"
constexpr int kDepth = 20;
struct Record {
    uint32_t magic;
    uint32_t uptime_ms;
    int core;
    char reason[96];
    char task[16];
    int depth;
    uint32_t pcs[kDepth];
};
RTC_NOINIT_ATTR Record g_record;
}  // namespace

extern "C" void __real_esp_panic_handler(panic_info_t* info);

extern "C" void __wrap_esp_panic_handler(panic_info_t* info) {
    Record& r = g_record;
    std::memset(&r, 0, sizeof(r));
    r.uptime_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000);
    r.core = info->core;
    const char* reason = g_panic_abort && g_panic_abort_details ? g_panic_abort_details : info->reason;
    if (reason) std::strncpy(r.reason, reason, sizeof(r.reason) - 1);
    if (TaskHandle_t t = xTaskGetCurrentTaskHandleForCore(info->core)) {
        std::strncpy(r.task, pcTaskGetName(t), sizeof(r.task) - 1);
    }
    if (auto* f = static_cast<const XtExcFrame*>(info->frame)) {
        esp_backtrace_frame_t bt = {};
        bt.pc = f->pc;
        bt.sp = f->a1;
        bt.next_pc = f->a0;
        bt.exc_frame = const_cast<XtExcFrame*>(f);
        r.pcs[r.depth++] = esp_cpu_process_stack_pc(bt.pc);
        while (r.depth < kDepth && bt.next_pc && esp_backtrace_get_next_frame(&bt)) {
            r.pcs[r.depth++] = esp_cpu_process_stack_pc(bt.pc);
        }
    }
    r.magic = kMagic;
    __real_esp_panic_handler(info);
}

namespace crash {

std::string Describe() {
    if (g_record.magic != kMagic) return "no crash recorded";
    char buf[96 + 64 + kDepth * 11];
    int n = snprintf(buf, sizeof(buf), "crash at %u ms, task %s, core %d: %s\nbacktrace:",
                     (unsigned)g_record.uptime_ms, g_record.task, g_record.core, g_record.reason);
    for (int i = 0; i < g_record.depth && i < kDepth && n < (int)sizeof(buf) - 12; ++i) {
        n += snprintf(buf + n, sizeof(buf) - n, " 0x%08x", (unsigned)g_record.pcs[i]);
    }
    return buf;
}

void Clear() { g_record.magic = 0; }

const char* ResetReason() {
    switch (esp_reset_reason()) {
        case ESP_RST_POWERON: return "power-on";
        case ESP_RST_SW: return "software";
        case ESP_RST_PANIC: return "PANIC";
        case ESP_RST_INT_WDT: return "interrupt watchdog";
        case ESP_RST_TASK_WDT: return "task watchdog";
        case ESP_RST_WDT: return "other watchdog";
        case ESP_RST_BROWNOUT: return "BROWNOUT";
        default: return "other";
    }
}

}  // namespace crash

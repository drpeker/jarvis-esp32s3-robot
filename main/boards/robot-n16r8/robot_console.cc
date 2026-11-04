#include "robot_console.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <string>

#include <esp_heap_caps.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "battery_gauge.h"
#include "config.h"
#include "motor_controller.h"
#include "mpu6050.h"
#include "net_console.h"
#include "panic_trace.h"
#include "application.h"

namespace {

MotorController* g_motor = nullptr;
Mpu6050* g_imu = nullptr;
BatteryGauge* g_gauge = nullptr;

NetConsole& Console() { return NetConsole::GetInstance(); }

const char* ResetReasonName(esp_reset_reason_t reason) {
    switch (reason) {
        case ESP_RST_POWERON: return "power-on";
        case ESP_RST_SW: return "software (esp_restart)";
        case ESP_RST_PANIC: return "PANIC (crash)";
        case ESP_RST_INT_WDT: return "interrupt watchdog";
        case ESP_RST_TASK_WDT: return "task watchdog";
        case ESP_RST_WDT: return "other watchdog";
        case ESP_RST_BROWNOUT: return "BROWNOUT (supply voltage dropped)";
        case ESP_RST_DEEPSLEEP: return "deep sleep";
        default: return "other";
    }
}

int ArgInt(const NetConsole::Args& args, size_t index, int fallback) {
    return args.size() > index ? std::atoi(args[index].c_str()) : fallback;
}

// Average |yaw rate| over `ms`, sampled every 10 ms.
float MeasureRate(int ms) {
    float sum = 0;
    int count = 0;
    for (int t = 0; t < ms; t += 10) {
        float dps = 0;
        if (g_imu->ReadYawRate(dps)) {
            sum += std::fabs(dps);
            ++count;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return count > 0 ? sum / count : 0;
}

void PrintTuning() {
    const auto t = g_motor->GetTuning();
    Console().Print(
        "min_duty  L fwd %d  L rev %d  R fwd %d  R rev %d (%%)\r\n"
        "gain      L %d  R %d (%%)\r\n"
        "turn_speed %d%%  lead %.1f deg  kick %d ms  brake %d ms  pwm %d Hz  factor %d%%\r\n"
        "forward accel axis %c%c\r\n"
        "distance: speed %d%%  20 cm = fwd %d ms  back %d ms\r\n",
        t.min_duty[0][0], t.min_duty[0][1], t.min_duty[1][0], t.min_duty[1][1], t.gain[0],
        t.gain[1], t.turn_speed, t.turn_lead_x10 / 10.0f, t.kick_ms, t.brake_ms, t.pwm_hz,
        g_motor->SpeedFactor(), t.accel_sign < 0 ? '-' : '+', t.accel_axis == 1 ? 'Y' : 'X',
        t.dist_speed, t.ms20[0], t.ms20[1]);
}

// Runs one gyro turn and waits for the result. Returns the final angle or -1.
float TurnAndWait(const char* direction, int degrees) {
    const uint32_t before = g_motor->TurnCount();
    const auto t = g_motor->GetTuning();
    const int speed = t.turn_speed;
    const bool left = std::string(direction) == "left";
    if (!g_motor->Drive(left ? -speed : speed, left ? speed : -speed, 5000, degrees)) {
        Console().Print("motor refused the turn (battery lock or raw mode)\r\n");
        return -1;
    }
    for (int i = 0; i < 80 && g_motor->TurnCount() == before; ++i) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    if (g_motor->TurnCount() == before) {
        return -1;
    }
    return g_motor->LastTurnFinalX10() / 10.0f;
}

// One track alone (pivot): ramp the duty and find where it starts to move.
void CalibrateMotors(MotorController::Tuning& tuning) {
    static const char* const kMotorNames[] = {"left", "right"};
    static const char* const kDirNames[] = {"fwd", "rev"};
    float full_rate[2][2] = {};
    g_motor->SetRawMode(true);
    for (int m = 0; m < 2; ++m) {
        for (int d = 0; d < 2; ++d) {
            const int sign = d == 0 ? 1 : -1;
            int start = -1;
            Console().Print("-- %s track %s --\r\n", kMotorNames[m], kDirNames[d]);
            for (int duty = 10; duty <= 100; duty += 5) {
                const int signed_duty = sign * duty;
                g_motor->SetRawDuty(m == 0 ? signed_duty : 0, m == 1 ? signed_duty : 0);
                vTaskDelay(pdMS_TO_TICKS(250));
                const float rate = MeasureRate(200);
                Console().Print("  duty %3d%%  yaw %6.1f deg/s%s\r\n", duty, rate,
                                (start < 0 && rate > 8) ? "  <- starts" : "");
                if (start < 0 && rate > 8) {
                    start = duty;
                }
                full_rate[m][d] = rate;
            }
            g_motor->SetRawDuty(0, 0);
            vTaskDelay(pdMS_TO_TICKS(700));
            if (start < 0) {
                Console().Print("  !! %s track %s did NOT move even at 100%% (check motor/"
                                "wiring/driver)\r\n",
                                kMotorNames[m], kDirNames[d]);
                tuning.min_duty[m][d] = 95;
            } else {
                tuning.min_duty[m][d] = start;
            }
        }
    }
    g_motor->SetRawMode(false);

    // Balance: the stronger track is scaled down to match the weaker one at full power.
    float avg[2];
    for (int m = 0; m < 2; ++m) {
        avg[m] = (full_rate[m][0] + full_rate[m][1]) / 2;
    }
    if (avg[0] > 1 && avg[1] > 1) {
        const float weaker = std::min(avg[0], avg[1]);
        for (int m = 0; m < 2; ++m) {
            tuning.gain[m] = static_cast<int>(std::lround(100 * weaker / avg[m]));
        }
    }
    Console().Print("full power yaw: left %.1f/%.1f  right %.1f/%.1f deg/s (fwd/rev)\r\n",
                    full_rate[0][0], full_rate[0][1], full_rate[1][0], full_rate[1][1]);
}

// 90 degree turns both ways: the stop lead absorbs the coast after braking.
void CalibrateTurns(MotorController::Tuning& tuning) {
    for (int round = 0; round < 2; ++round) {
        float error_sum = 0;
        int count = 0;
        for (const char* direction : {"left", "right"}) {
            vTaskDelay(pdMS_TO_TICKS(800));
            const float final_angle = TurnAndWait(direction, 90);
            if (final_angle < 0) {
                continue;
            }
            Console().Print("  turn %s 90 -> %.1f deg\r\n", direction, final_angle);
            error_sum += final_angle - 90;
            ++count;
        }
        if (count == 0) {
            Console().Print("turn test failed\r\n");
            return;
        }
        const float error = error_sum / count;
        tuning.turn_lead_x10 = std::max(0, tuning.turn_lead_x10 + static_cast<int>(error * 10));
        g_motor->SetTuning(tuning, false);
        Console().Print("round %d: mean error %+.1f deg -> lead %.1f deg\r\n", round + 1, error,
                        tuning.turn_lead_x10 / 10.0f);
    }
}

bool SetTuningValue(MotorController::Tuning& t, const std::string& key, int value) {
    if (key == "min_lf") t.min_duty[0][0] = value;
    else if (key == "min_lr") t.min_duty[0][1] = value;
    else if (key == "min_rf") t.min_duty[1][0] = value;
    else if (key == "min_rr") t.min_duty[1][1] = value;
    else if (key == "gain_l") t.gain[0] = value;
    else if (key == "gain_r") t.gain[1] = value;
    else if (key == "turn_speed") t.turn_speed = value;
    else if (key == "lead") t.turn_lead_x10 = value;
    else if (key == "kick") t.kick_ms = value;
    else if (key == "brake") t.brake_ms = value;
    else if (key == "pwm") t.pwm_hz = value;
    else if (key == "axis") t.accel_axis = value;
    else if (key == "axis_sign") t.accel_sign = value;
    else if (key == "dist_speed") t.dist_speed = value;
    else if (key == "ms20_fwd") t.ms20[0] = value;
    else if (key == "ms20_back") t.ms20[1] = value;
    else return false;
    return true;
}

}  // namespace

void RegisterRobotConsole(MotorController* motor, Mpu6050* imu, BatteryGauge* gauge) {
    g_motor = motor;
    g_imu = imu;
    g_gauge = gauge;
    auto& console = Console();

    console.AddCommand("status", "status  - battery, IMU, motor tuning", [](const auto&) {
        if (g_gauge != nullptr) {
            Console().Print("battery %s\r\n", g_gauge->StatusJson().c_str());
        } else {
            Console().Print("battery: INA219 not found\r\n");
        }
        Console().Print("imu %s, motors %s%s\r\n", g_imu ? "ok" : "missing",
                        g_motor->IsMoving() ? "moving" : "stopped",
                        g_motor->IsMotionLocked() ? ", LOCKED (battery)" : "");
        PrintTuning();
        Console().Print("last reset: %s, uptime %u s\r\n", ResetReasonName(esp_reset_reason()),
                        static_cast<unsigned>(esp_timer_get_time() / 1000000));
        Console().Print("internal RAM free %u (min %u, largest block %u), PSRAM free %u (min %u)\r\n",
                        heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                        heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL),
                        heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL),
                        heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
                        heap_caps_get_minimum_free_size(MALLOC_CAP_SPIRAM));
    });

    console.AddCommand("crash", "crash  - last crash (reason, task, backtrace PCs)", [](const auto&) {
        if (!PanicTraceValid()) {
            Console().Print("no crash recorded since power-on\r\n");
            return;
        }
        const PanicTrace& t = g_panic_trace;
        Console().Print("crash: %s\r\ntask %s, core %d, addr 0x%08x\r\nBacktrace:", t.reason,
                        t.task, t.core, static_cast<unsigned>(t.addr));
        for (int i = 0; i < t.depth && i < PanicTrace::kMaxDepth; ++i) {
            Console().Print(" 0x%08x", static_cast<unsigned>(t.pcs[i]));
        }
        Console().Print("\r\n");
    });

    console.AddCommand("tune", "tune  - show motor tuning", [](const auto&) { PrintTuning(); });

    console.AddCommand(
        "set",
        "set <min_lf|min_lr|min_rf|min_rr|gain_l|gain_r|turn_speed|lead(x0.1deg)|kick|brake|"
        "pwm|axis(0=X,1=Y)|axis_sign(1|-1)|dist_speed|ms20_fwd|ms20_back> <value>  - change now "
        "(save to keep)",
        [](const NetConsole::Args& args) {
            if (args.size() < 3) {
                Console().Print("usage: set <key> <value>\r\n");
                return;
            }
            const int value = std::atoi(args[2].c_str());
            auto t = g_motor->GetTuning();
            if (!SetTuningValue(t, args[1], value)) {
                Console().Print("unknown key '%s'\r\n", args[1].c_str());
                return;
            }
            g_motor->SetTuning(t, false);
            PrintTuning();
        });

    console.AddCommand("save", "save  - store the motor tuning in flash", [](const auto&) {
        // NVS writes need an internal-RAM stack; the console task's stack is in PSRAM.
        Application::GetInstance().Schedule(
            []() { g_motor->SetTuning(g_motor->GetTuning(), true); });
        vTaskDelay(pdMS_TO_TICKS(300));
        Console().Print("saved\r\n");
    });

    console.AddCommand("m", "m <left%> <right%> <ms>  - raw PWM (-100..100), no mapping",
                       [](const NetConsole::Args& args) {
                           const int left = std::clamp(ArgInt(args, 1, 0), -100, 100);
                           const int right = std::clamp(ArgInt(args, 2, 0), -100, 100);
                           const int ms = std::clamp(ArgInt(args, 3, 500), 50, 5000);
                           g_motor->SetRawMode(true);
                           g_motor->SetRawDuty(left, right);
                           float rate = 0;
                           if (g_imu != nullptr) {
                               vTaskDelay(pdMS_TO_TICKS(std::min(ms / 2, 400)));
                               rate = MeasureRate(std::max(10, ms - std::min(ms / 2, 400)));
                           } else {
                               vTaskDelay(pdMS_TO_TICKS(ms));
                           }
                           g_motor->SetRawMode(false);
                           Console().Print("raw L %d%% R %d%% %d ms, yaw %.1f deg/s\r\n", left,
                                           right, ms, rate);
                       });

    console.AddCommand("drive", "drive <fwd|back> <ms> [speed]  - mapped drive",
                       [](const NetConsole::Args& args) {
                           const bool back = args.size() > 1 && args[1] == "back";
                           const int ms = std::clamp(ArgInt(args, 2, 800), 100, 5000);
                           const int speed = std::clamp(ArgInt(args, 3, 100), 1, 100) *
                                             g_motor->SpeedFactor() / 100;
                           const int s = std::max(speed, 1) * (back ? -1 : 1);
                           Console().Print("%s\r\n", g_motor->Drive(s, s, ms) ? "ok" : "refused");
                       });

    console.AddCommand("turn", "turn <left|right> [deg]  - gyro turn, prints final angle",
                       [](const NetConsole::Args& args) {
                           if (g_imu == nullptr) {
                               Console().Print("IMU missing\r\n");
                               return;
                           }
                           const char* direction =
                               (args.size() > 1 && args[1] == "left") ? "left" : "right";
                           const int degrees = std::clamp(ArgInt(args, 2, 20), 1, 360);
                           const float final_angle = TurnAndWait(direction, degrees);
                           Console().Print("turn %s %d -> %.1f deg\r\n", direction, degrees,
                                           final_angle);
                       });

    console.AddCommand(
        "dist", "dist <fwd|back> <cm>  - timed distance drive (ms20_fwd/ms20_back), measure it",
        [](const NetConsole::Args& args) {
            const bool back = args.size() > 1 && args[1] == "back";
            const int cm = std::clamp(ArgInt(args, 2, 20), 1, 300);
            const auto t = g_motor->GetTuning();
            const uint32_t ms = std::min<uint32_t>(g_motor->DistanceMs(cm, back),
                                                   MOTOR_MAX_RUN_TIME_MS);
            const int s = back ? -t.dist_speed : t.dist_speed;
            if (!g_motor->Drive(s, s, ms, 0, cm)) {
                Console().Print("refused\r\n");
                return;
            }
            Console().Print("dist %s %d cm -> %u ms at %d%% (measure it with a tape)\r\n",
                            back ? "back" : "fwd", cm, static_cast<unsigned>(ms), t.dist_speed);
        });

    console.AddCommand(
        "cal_axis", "cal_axis  - find which accelerometer axis points forward (drives ~0.5 s)",
        [](const auto&) {
            if (g_imu == nullptr) {
                Console().Print("IMU missing\r\n");
                return;
            }
            float bias_x = 0, bias_y = 0;
            for (int i = 0; i < 20; ++i) {
                float x = 0, y = 0;
                g_imu->ReadAccelXY(x, y);
                bias_x += x / 20;
                bias_y += y / 20;
                vTaskDelay(1);
            }
            g_motor->SetRawMode(true);
            g_motor->SetRawDuty(100, 100);
            float peak_x = 0, peak_y = 0;
            for (int i = 0; i < 25; ++i) {
                float x = 0, y = 0;
                if (g_imu->ReadAccelXY(x, y)) {
                    x -= bias_x;
                    y -= bias_y;
                    if (std::fabs(x) > std::fabs(peak_x)) peak_x = x;
                    if (std::fabs(y) > std::fabs(peak_y)) peak_y = y;
                }
                vTaskDelay(1);
            }
            g_motor->SetRawMode(false);
            auto t = g_motor->GetTuning();
            t.accel_axis = std::fabs(peak_y) > std::fabs(peak_x) ? 1 : 0;
            const float peak = t.accel_axis == 1 ? peak_y : peak_x;
            t.accel_sign = peak < 0 ? -1 : 1;
            g_motor->SetTuning(t, false);
            Console().Print("start peak: X %+.3f g  Y %+.3f g -> forward is %c%c (save to keep)\r\n",
                            peak_x, peak_y, t.accel_sign < 0 ? '-' : '+',
                            t.accel_axis == 1 ? 'Y' : 'X');
        });

    console.AddCommand("stop", "stop", [](const auto&) {
        g_motor->Stop();
        Console().Print("stopped\r\n");
    });

    console.AddCommand(
        "accel", "accel [sec]  - live accel X/Y/Z (g) and gyro X/Y/Z, to see how the IMU sits",
        [](const NetConsole::Args& args) {
            if (g_imu == nullptr) {
                Console().Print("IMU missing\r\n");
                return;
            }
            const float* up = g_imu->Up();
            Console().Print("startup up vector (%.2f, %.2f, %.2f), mounting tilt %.1f deg\r\n",
                            up[0], up[1], up[2], g_imu->MountTiltDegrees());
            const int seconds = std::clamp(ArgInt(args, 1, 10), 1, 60);
            for (int i = 0; i < seconds * 2; ++i) {
                float a[3], g[3];
                if (g_imu->ReadMotion(a, g)) {
                    Console().Print("acc X %+.2f Y %+.2f Z %+.2f g | gyro X %+6.1f Y %+6.1f Z %+6.1f\r\n",
                                    a[0], a[1], a[2], g[0], g[1], g[2]);
                }
                vTaskDelay(pdMS_TO_TICKS(500));
            }
        });

    console.AddCommand(
        "gyro_hand", "gyro_hand [sec]  - turn the robot by hand exactly one full turn (360) "
                     "within the time; prints the measured angle and the scale to use",
        [](const NetConsole::Args& args) {
            if (g_imu == nullptr) {
                Console().Print("IMU missing\r\n");
                return;
            }
            const int seconds = std::clamp(ArgInt(args, 1, 20), 5, 60);
            Console().Print("now turn the robot by hand ONE full turn (360) back to the mark; "
                            "%d s, current scale %.4f\r\n",
                            seconds, g_imu->YawScale());
            // The FIFO heading has no gaps, whatever delays this task.
            const float start_heading = g_imu->Heading();
            float heading = 0;
            for (int s = 1; s <= seconds; ++s) {
                vTaskDelay(pdMS_TO_TICKS(1000));
                heading = g_imu->Heading() - start_heading;
                Console().Print("  %2d s  %7.1f deg\r\n", s, heading);
            }
            const float measured = std::fabs(heading);
            if (measured < 90) {
                Console().Print("only %.1f deg measured - no full turn, nothing changed\r\n",
                                measured);
                return;
            }
            const float scale = g_imu->YawScale() * 360.0f / measured;
            Console().Print("measured %.1f deg for a real 360 -> suggested scale %.4f "
                            "(apply: gyro_scale %.4f)\r\n",
                            measured, scale, scale);
        });

    console.AddCommand("gyro_scale", "gyro_scale <value>  - set and save the yaw scale (1.0 = raw)",
                       [](const NetConsole::Args& args) {
                           if (g_imu == nullptr || args.size() < 2) {
                               Console().Print("usage: gyro_scale <value>\r\n");
                               return;
                           }
                           g_imu->SetYawScale(std::atof(args[1].c_str()));
                           Application::GetInstance().Schedule([]() { g_imu->SaveYawScale(); });
                           vTaskDelay(pdMS_TO_TICKS(300));
                           Console().Print("yaw scale %.4f saved\r\n", g_imu->YawScale());
                       });

    console.AddCommand("imu", "imu [sec]  - yaw rate and heading", [](const NetConsole::Args& args) {
        if (g_imu == nullptr) {
            Console().Print("IMU missing\r\n");
            return;
        }
        const int seconds = std::clamp(ArgInt(args, 1, 5), 1, 30);
        const float start_heading = g_imu->Heading();
        for (int i = 0; i < seconds * 5; ++i) {
            vTaskDelay(pdMS_TO_TICKS(200));
            float dps = 0;
            g_imu->ReadYawRate(dps);
            Console().Print("yaw %7.1f deg/s  heading %7.1f deg\r\n", dps,
                            g_imu->Heading() - start_heading);
        }
    });

    console.AddCommand(
        "cal", "cal [motors|turns]  - automatic calibration (robot on the floor, clear space)",
        [](const NetConsole::Args& args) {
            if (g_imu == nullptr) {
                Console().Print("IMU missing, calibration needs the gyro\r\n");
                return;
            }
            const std::string part = args.size() > 1 ? args[1] : "all";
            auto tuning = g_motor->GetTuning();
            Console().Print("calibration '%s' starts in 3 s - keep the area clear\r\n",
                            part.c_str());
            vTaskDelay(pdMS_TO_TICKS(3000));
            if (part == "all" || part == "motors") {
                CalibrateMotors(tuning);
                g_motor->SetTuning(tuning, false);
            }
            if (part == "all" || part == "turns") {
                CalibrateTurns(tuning);
            }
            Console().Print("result (not saved yet, type 'save' to keep):\r\n");
            PrintTuning();
        });

    console.AddCommand(
        "ota", "ota <http://host:port/xiaozhi.bin>  - update the firmware over Wi-Fi, then reboot",
        [](const NetConsole::Args& args) {
            if (args.size() < 2) {
                Console().Print("usage: ota <url>\r\n");
                return;
            }
            g_motor->Stop();
            // The main task runs the stock upgrade: closes the conversation, stops audio,
            // shows "Updating...", writes the other OTA slot and reboots. Progress shows up in
            // the live log ("Progress: N%").
            Console().Print("ota started: %s\r\n", args[1].c_str());
            const std::string url = args[1];
            Application::GetInstance().Schedule(
                [url]() { Application::GetInstance().UpgradeFirmware(url, "(Wi-Fi console)"); });
        });

    console.AddCommand("reboot", "reboot", [](const auto&) {
        Console().Print("rebooting\r\n");
        vTaskDelay(pdMS_TO_TICKS(300));
        esp_restart();
    });
}

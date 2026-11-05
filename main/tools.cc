#include "tools.h"

#include <cstdlib>
#include <cstring>
#include <initializer_list>
#include <vector>

#include <cJSON.h>
#include <esp_app_desc.h>
#include <esp_crt_bundle.h>
#include <esp_heap_caps.h>
#include <esp_http_client.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include "audio.h"
#include "camera.h"
#include "face.h"
#include "identity.h"
#include "imu.h"
#include "motors.h"
#include "power.h"
#include "status.h"

#define TAG "tools"

namespace {

std::function<void(const std::string&)> g_send;
QueueHandle_t g_queue = nullptr;  // char* payloads (PSRAM)
std::string g_vision_url, g_vision_token;

// ---- helpers --------------------------------------------------------------------------

double Num(const cJSON* args, const char* name, double fallback) {
    const cJSON* v = cJSON_GetObjectItem(args, name);
    if (cJSON_IsNumber(v)) return v->valuedouble;
    if (cJSON_IsString(v) && v->valuestring[0]) {  // models sometimes send "45"
        char* end = nullptr;
        const double d = std::strtod(v->valuestring, &end);
        if (end != v->valuestring) return d;
    }
    return fallback;
}

std::string Str(const cJSON* args, const char* name) {
    const cJSON* v = cJSON_GetObjectItem(args, name);
    return cJSON_IsString(v) ? v->valuestring : "";
}

void Reply(int id, cJSON* result) {
    cJSON* r = cJSON_CreateObject();
    cJSON_AddStringToObject(r, "jsonrpc", "2.0");
    cJSON_AddNumberToObject(r, "id", id);
    cJSON_AddItemToObject(r, "result", result);
    char* s = cJSON_PrintUnformatted(r);
    g_send(s);
    cJSON_free(s);
    cJSON_Delete(r);
}

void ReplyError(int id, const char* message) {
    cJSON* r = cJSON_CreateObject();
    cJSON_AddStringToObject(r, "jsonrpc", "2.0");
    cJSON_AddNumberToObject(r, "id", id);
    cJSON_AddStringToObject(cJSON_AddObjectToObject(r, "error"), "message", message);
    char* s = cJSON_PrintUnformatted(r);
    g_send(s);
    cJSON_free(s);
    cJSON_Delete(r);
}

// Tool result: one text item.
void ReplyText(int id, const std::string& text, bool error = false) {
    cJSON* result = cJSON_CreateObject();
    cJSON* item = cJSON_CreateObject();
    cJSON_AddStringToObject(item, "type", "text");
    cJSON_AddStringToObject(item, "text", text.c_str());
    cJSON_AddItemToArray(cJSON_AddArrayToObject(result, "content"), item);
    cJSON_AddBoolToObject(result, "isError", error);
    Reply(id, result);
}

// ---- tool definitions -----------------------------------------------------------------

void Prop(cJSON* props, const char* name, const char* type, const char* description = nullptr,
          std::initializer_list<const char*> choices = {}) {
    cJSON* p = cJSON_AddObjectToObject(props, name);
    cJSON_AddStringToObject(p, "type", type);
    if (description) cJSON_AddStringToObject(p, "description", description);
    if (choices.size()) {
        cJSON* e = cJSON_AddArrayToObject(p, "enum");
        for (const char* c : choices) cJSON_AddItemToArray(e, cJSON_CreateString(c));
    }
}

void Tool(cJSON* list, const char* name, const char* description, cJSON* props = nullptr,
          std::initializer_list<const char*> required = {}) {
    cJSON* t = cJSON_CreateObject();
    cJSON_AddStringToObject(t, "name", name);
    cJSON_AddStringToObject(t, "description", description);
    cJSON* schema = cJSON_AddObjectToObject(t, "inputSchema");
    cJSON_AddStringToObject(schema, "type", "object");
    cJSON_AddItemToObject(schema, "properties", props ? props : cJSON_CreateObject());
    if (required.size()) {
        cJSON* r = cJSON_AddArrayToObject(schema, "required");
        for (const char* q : required) cJSON_AddItemToArray(r, cJSON_CreateString(q));
    }
    cJSON_AddItemToArray(list, t);
}

cJSON* ToolList() {
    cJSON* list = cJSON_CreateArray();
    Tool(list, "self.get_device_status",
         "Jarvis's own body: battery voltage, current, percent and protection state, tilt, heading, Wi-Fi "
         "signal, memory, uptime, speaker volume. Use it whenever the user asks about Jarvis himself.");
    cJSON* p = cJSON_CreateObject();
    Prop(p, "direction", "string", "left/right: turn in place; forward/backward: drive",
         {"left", "right", "forward", "backward"});
    Prop(p, "degrees", "number", "for left/right: angle 1-720 (a little: 20, quarter: 90, turn around: 180)");
    Prop(p, "cm", "number", "for forward/backward: distance 1-100 cm (a little: 5)");
    Tool(list, "self.robot.move",
         "Move Jarvis, the tracked robot. Turns are measured by the gyro, forward/backward is timed. "
         "'Turn around' is a 180 degree turn, not driving backward. Without an amount: 20 degrees or 10 cm. "
         "Call it once per requested move and report only what the result says.",
         p, {"direction"});
    Tool(list, "self.robot.stop", "Stop every movement at once.");
    p = cJSON_CreateObject();
    Prop(p, "question", "string", "what to look for or describe");
    Tool(list, "self.camera.take_photo",
         "Take a photo with Jarvis's camera and get it described. Use it whenever you need to see.", p,
         {"question"});
    p = cJSON_CreateObject();
    Prop(p, "volume", "integer", "0-100");
    Tool(list, "self.audio_speaker.set_volume", "Set the speaker volume.", p, {"volume"});
    p = cJSON_CreateObject();
    Prop(p, "emotion", "string", nullptr, {"happy", "sad", "surprised", "angry", "sleepy", "neutral"});
    Tool(list, "self.robot.express", "Show an emotion on Jarvis's face for a few seconds.", p, {"emotion"});
    return list;
}

// ---- vision: photo to the server's explain URL ----------------------------------------

int g_last_upload_ms = 0;  // time to send the photo (console 'cam sweep upload')

std::string Explain(const std::vector<uint8_t>& jpeg, const std::string& question, bool& ok) {
    ok = false;
    g_last_upload_ms = 0;
    if (g_vision_url.empty()) return "the server gave no vision URL";
    const char* boundary = "----JarvisPhoto7f3";
    const std::string head = std::string("--") + boundary +
                             "\r\nContent-Disposition: form-data; name=\"question\"\r\n\r\n" + question + "\r\n--" +
                             boundary +
                             "\r\nContent-Disposition: form-data; name=\"file\"; filename=\"camera.jpg\"\r\n"
                             "Content-Type: image/jpeg\r\n\r\n";
    const std::string tail = std::string("\r\n--") + boundary + "--\r\n";
    esp_http_client_config_t cfg = {};
    cfg.url = g_vision_url.c_str();
    cfg.method = HTTP_METHOD_POST;
    cfg.crt_bundle_attach = esp_crt_bundle_attach;
    cfg.timeout_ms = 20000;
    esp_http_client_handle_t c = esp_http_client_init(&cfg);
    const std::string type = std::string("multipart/form-data; boundary=") + boundary;
    esp_http_client_set_header(c, "Content-Type", type.c_str());
    esp_http_client_set_header(c, "Device-Id", identity::Mac().c_str());
    esp_http_client_set_header(c, "Client-Id", identity::Uuid().c_str());
    if (!g_vision_token.empty()) {
        const std::string auth = "Bearer " + g_vision_token;
        esp_http_client_set_header(c, "Authorization", auth.c_str());
    }
    std::string resp;
    int status = -1;
    const int64_t t0 = esp_timer_get_time();
    const bool sent = esp_http_client_open(c, head.size() + jpeg.size() + tail.size()) == ESP_OK &&
                      esp_http_client_write(c, head.data(), head.size()) == (int)head.size() &&
                      esp_http_client_write(c, (const char*)jpeg.data(), jpeg.size()) == (int)jpeg.size() &&
                      esp_http_client_write(c, tail.data(), tail.size()) == (int)tail.size();
    if (sent) g_last_upload_ms = (int)((esp_timer_get_time() - t0) / 1000);
    if (sent && esp_http_client_fetch_headers(c) >= 0) {
        status = esp_http_client_get_status_code(c);
        char buf[512];
        int n;
        while ((n = esp_http_client_read(c, buf, sizeof(buf))) > 0) resp.append(buf, n);
    }
    esp_http_client_close(c);
    esp_http_client_cleanup(c);
    ESP_LOGI(TAG, "vision HTTP %d: %.200s", status, resp.c_str());
    ok = status == 200;
    return ok ? resp : "photo upload failed (HTTP " + std::to_string(status) + ")";
}

// ---- tools ----------------------------------------------------------------------------

void Call(int id, const std::string& name, const cJSON* args) {
    ESP_LOGI(TAG, "call %s", name.c_str());
    char buf[256];
    if (name == "self.robot.move") {
        const std::string d = Str(args, "direction");
        motors::Result r{false, 0, "unknown direction"};
        if (d == "left" || d == "right") {
            r = motors::Turn(d == "left", (int)Num(args, "degrees", 20));
            snprintf(buf, sizeof(buf), "{\"turned_degrees\":%d,\"completed\":%s%s%s%s}", (int)(r.value + 0.5f),
                     r.ok ? "true" : "false", r.note[0] ? ",\"note\":\"" : "", r.note, r.note[0] ? "\"" : "");
        } else if (d == "forward" || d == "backward") {
            r = motors::Distance(d == "forward", (int)Num(args, "cm", 10));
            snprintf(buf, sizeof(buf), "{\"driven_cm\":%d,\"completed\":%s%s%s%s}", (int)(r.value + 0.5f),
                     r.ok ? "true" : "false", r.note[0] ? ",\"note\":\"" : "", r.note, r.note[0] ? "\"" : "");
        } else {
            return ReplyText(id, "direction must be left, right, forward or backward", true);
        }
        return ReplyText(id, buf);
    }
    if (name == "self.robot.stop") {
        motors::Stop();
        return ReplyText(id, "stopped");
    }
    if (name == "self.camera.take_photo") {
        std::vector<uint8_t> jpeg;
        if (!camera::TakePhoto(jpeg)) return ReplyText(id, "the camera delivered no picture", true);
        face::ShowPhoto(jpeg);
        std::string q = Str(args, "question");
        if (q.empty()) q = "Describe what you see.";
        bool ok = false;
        const std::string answer = Explain(jpeg, q, ok);
        return ReplyText(id, answer, !ok);
    }
    if (name == "self.get_device_status") {
        const power::Reading b = power::Get();
        snprintf(buf, sizeof(buf),
                 "{\"battery_volts\":%.2f,\"current_amps\":%.3f,\"battery_percent\":%d,\"battery_state\":\"%s\","
                 "\"tilt_degrees\":%d,\"heading_degrees\":%d,\"wifi_rssi_dbm\":%d,\"free_internal_kb\":%u,"
                 "\"uptime_minutes\":%d,\"volume_percent\":%d}",
                 b.mv / 1000.0, b.ma / 1000.0, b.percent, b.critical ? "critical (motors locked)" : b.low ? "low" : "normal",
                 (int)imu::TiltDegrees(), (int)imu::Heading(), status::WifiRssi(),
                 (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024),
                 (int)(esp_timer_get_time() / 60000000), audio::Volume());
        return ReplyText(id, b.ok ? buf : "power sensor not found");
    }
    if (name == "self.audio_speaker.set_volume") {
        audio::SetVolume((int)Num(args, "volume", audio::Volume()));
        return ReplyText(id, "volume " + std::to_string(audio::Volume()));
    }
    if (name == "self.robot.express") {
        face::SetEmotion(Str(args, "emotion").c_str());
        return ReplyText(id, "shown");
    }
    ReplyError(id, ("unknown tool: " + name).c_str());
}

void Handle(const cJSON* msg) {
    const cJSON* idj = cJSON_GetObjectItem(msg, "id");
    const std::string method = Str(msg, "method");
    const cJSON* params = cJSON_GetObjectItem(msg, "params");
    if (!cJSON_IsNumber(idj)) return;  // notifications need no answer
    const int id = idj->valueint;
    if (method == "initialize") {
        const cJSON* vision = cJSON_GetObjectItem(cJSON_GetObjectItem(params, "capabilities"), "vision");
        g_vision_url = Str(vision, "url");
        g_vision_token = Str(vision, "token");
        ESP_LOGI(TAG, "initialize, vision %s", g_vision_url.empty() ? "none" : g_vision_url.c_str());
        cJSON* r = cJSON_CreateObject();
        cJSON_AddStringToObject(r, "protocolVersion", "2024-11-05");
        cJSON_AddObjectToObject(cJSON_AddObjectToObject(r, "capabilities"), "tools");
        cJSON* info = cJSON_AddObjectToObject(r, "serverInfo");
        cJSON_AddStringToObject(info, "name", "Jarvis");
        cJSON_AddStringToObject(info, "version", esp_app_get_description()->version);
        Reply(id, r);
    } else if (method == "tools/list") {
        cJSON* r = cJSON_CreateObject();
        cJSON_AddItemToObject(r, "tools", ToolList());
        Reply(id, r);
    } else if (method == "tools/call") {
        Call(id, Str(params, "name"), cJSON_GetObjectItem(params, "arguments"));
    } else {
        ReplyError(id, ("method not implemented: " + method).c_str());
    }
}

void Task(void*) {
    char* payload = nullptr;
    while (true) {
        if (xQueueReceive(g_queue, &payload, portMAX_DELAY) != pdTRUE) continue;
        if (cJSON* msg = cJSON_Parse(payload)) {
            Handle(msg);
            cJSON_Delete(msg);
        }
        heap_caps_free(payload);
    }
}

}  // namespace

namespace tools {

bool MeasureUpload(const std::vector<uint8_t>& jpeg, int* upload_ms, int* total_ms) {
    bool ok = false;
    const int64_t t0 = esp_timer_get_time();
    Explain(jpeg, "Describe this picture in one short sentence.", ok);
    *total_ms = (int)((esp_timer_get_time() - t0) / 1000);
    *upload_ms = g_last_upload_ms;
    return ok;
}

void Start(std::function<void(const std::string&)> send) {
    g_send = std::move(send);
    g_queue = xQueueCreate(8, sizeof(char*));
    // Internal-RAM stack: tools may write NVS (volume) and use the camera.
    xTaskCreatePinnedToCore(Task, "tools", 8192, nullptr, 4, nullptr, 1);
}

void HandleMcp(const std::string& payload) {
    if (!g_queue) return;
    char* copy = (char*)heap_caps_malloc(payload.size() + 1, MALLOC_CAP_SPIRAM);
    if (!copy) return;
    memcpy(copy, payload.c_str(), payload.size() + 1);
    if (xQueueSend(g_queue, &copy, 0) != pdTRUE) heap_caps_free(copy);
}

}  // namespace tools

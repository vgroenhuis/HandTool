// Web server: the pages (embedded, gzipped), the JSON endpoints of the Arduino version (same names and fields), a live
// stream, and calibration.
//   GET  /allData /angles /rawAdcValues /filteredAdcValues /fk /dhParams     as before
//   GET  /serialAngles?enable=0|1  /serialFK?enable=0|1  /serialFrequency[?hz=10]   serial (USB) output, as before
//   WS   /ws                    /allData pushed 50 times a second (the page sends one message, "stream", to start it)
//   GET  /api/state             firmware, sensors, display, battery, Wi-Fi, calibration
//   POST /api/zero              the current pose becomes the zero pose (?reset=1: back to the factory values)
//   GET/POST /api/calibration   {"zero": [6 raw readings at 0 degrees]}; POST saves it
#include "web.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cJSON.h"
#include "closer_demo.h"
#include "display.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "kinematics.h"
#include "lwip/sockets.h"
#include "sensors.h"
#include "web_table.h"

static const char *TAG = "web";

// ---------------------------------------------------------------- clients (for the report)

#define MAX_CLIENTS 16
static struct { uint32_t ip; int64_t last_us; } seen[MAX_CLIENTS];

static void touch_client(httpd_req_t *req) {
    struct sockaddr_in6 addr;
    socklen_t len = sizeof addr;
    if (getpeername(httpd_req_to_sockfd(req), (struct sockaddr *)&addr, &len) != 0) return;
    uint32_t ip = addr.sin6_family == AF_INET6 ? addr.sin6_addr.un.u32_addr[3] : ((struct sockaddr_in *)&addr)->sin_addr.s_addr;
    int64_t now = esp_timer_get_time();
    int oldest = 0;
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (seen[i].ip == ip) { seen[i].last_us = now; return; }
        if (seen[i].last_us < seen[oldest].last_us) oldest = i;
    }
    seen[oldest].ip = ip;
    seen[oldest].last_us = now;
}

static int active_clients(void) {
    int64_t now = esp_timer_get_time();
    int n = 0;
    for (int i = 0; i < MAX_CLIENTS; i++) n += seen[i].ip && now - seen[i].last_us < 60 * 1000000LL;
    return n;
}

// ---------------------------------------------------------------- helpers

static const char *b(bool v) {
    return v ? "true" : "false";
}

static esp_err_t send_json_text(httpd_req_t *req, const char *json, int len) {
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, json, len);
}

static esp_err_t send_json(httpd_req_t *req, cJSON *j) {
    char *out = cJSON_PrintUnformatted(j);
    cJSON_Delete(j);
    if (!out) return httpd_resp_send_500(req);
    esp_err_t e = send_json_text(req, out, strlen(out));
    free(out);
    return e;
}

// Appends with snprintf, never past the end.
typedef struct { char *buf; size_t cap; int n; } sbuf_t;

static void put(sbuf_t *s, const char *fmt, ...) {
    if (s->n >= (int)s->cap) return;
    va_list ap;
    va_start(ap, fmt);
    int k = vsnprintf(s->buf + s->n, s->cap - s->n, fmt, ap);
    va_end(ap);
    s->n = k < 0 ? (int)s->cap : s->n + k;
}

static void put_floats(sbuf_t *s, const char *name, const float *v, int count, int decimals) {
    put(s, "\"%s\":[", name);
    for (int i = 0; i < count; i++) put(s, "%s%.*f", i ? "," : "", decimals, v[i]);
    put(s, "]");
}

static void put_matrix(sbuf_t *s, const float T[4][4]) {
    put(s, "\"matrix\":[");
    for (int r = 0; r < 4; r++) {
        put(s, "%s[%.6f,%.6f,%.6f,%.6f]", r ? "," : "", T[r][0], T[r][1], T[r][2], T[r][3]);
    }
    put(s, "]");
}

static int build_all_data(char *buf, size_t cap) {
    sensor_snapshot_t d;
    sensors_get(&d);
    sbuf_t s = {buf, cap, 0};
    put(&s, "{\"valid\":%s,\"dragButtonPressed\":%s,\"rawAdcValues\":[", b(d.valid), b(d.button));
    for (int i = 0; i < 8; i++) put(&s, "%s%u", i ? "," : "", d.raw[i]);
    put(&s, "],");
    put_floats(&s, "filteredAdcValues", d.filtered, 8, 2);
    put(&s, ",");
    put_floats(&s, "jointAngles", d.angles_deg, 6, 2);
    put(&s, ",");
    put_matrix(&s, d.T);
    put(&s, ",\"battery\":%.2f,\"hz\":%.0f,\"seq\":%lu}", d.battery_v, d.rate_hz, (unsigned long)d.seq);
    return s.n < (int)cap ? s.n : -1;
}

// ---------------------------------------------------------------- static files

static esp_err_t static_get(httpd_req_t *req) {
    char path[64];
    size_t n = strcspn(req->uri, "?");
    if (n >= sizeof path) return httpd_resp_send_404(req);
    memcpy(path, req->uri, n);
    path[n] = 0;
    if (strcmp(path, "/") == 0) strcpy(path, "/index.html");
    for (size_t i = 0; i < sizeof WEB_FILES / sizeof WEB_FILES[0]; i++) {
        if (strcmp(WEB_FILES[i].path, path) == 0) {
            if (strstr(path, ".html")) touch_client(req);
            httpd_resp_set_type(req, WEB_FILES[i].type);
            httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
            httpd_resp_set_hdr(req, "Cache-Control", strstr(path, "vendor") || strstr(path, ".jpg") ? "public, max-age=3600" : "no-cache");
            return httpd_resp_send(req, (const char *)WEB_FILES[i].start, WEB_FILES[i].end - WEB_FILES[i].start);
        }
    }
    return httpd_resp_send_404(req);
}

// ---------------------------------------------------------------- the JSON endpoints of the Arduino version

static esp_err_t all_data_get(httpd_req_t *req) {
    touch_client(req);
    char buf[1024];
    int n = build_all_data(buf, sizeof buf);
    return n < 0 ? httpd_resp_send_500(req) : send_json_text(req, buf, n);
}

static esp_err_t angles_get(httpd_req_t *req) {
    touch_client(req);
    sensor_snapshot_t d;
    sensors_get(&d);
    char buf[256];
    sbuf_t s = {buf, sizeof buf, 0};
    put(&s, "{\"valid\":%s,\"dragButtonPressed\":%s,", b(d.valid), b(d.button));
    put_floats(&s, "angles", d.angles_deg, 6, 2);
    put(&s, "}");
    return send_json_text(req, buf, s.n);
}

static esp_err_t raw_get(httpd_req_t *req) {
    sensor_snapshot_t d;
    sensors_get(&d);
    char buf[160];
    sbuf_t s = {buf, sizeof buf, 0};
    put(&s, "{\"valid\":%s,\"values\":[", b(d.valid));
    for (int i = 0; i < 8; i++) put(&s, "%s%u", i ? "," : "", d.raw[i]);
    put(&s, "]}");
    return send_json_text(req, buf, s.n);
}

static esp_err_t filtered_get(httpd_req_t *req) {
    sensor_snapshot_t d;
    sensors_get(&d);
    char buf[200];
    sbuf_t s = {buf, sizeof buf, 0};
    put(&s, "{\"valid\":%s,", b(d.valid));
    put_floats(&s, "values", d.filtered, 8, 2);
    put(&s, "}");
    return send_json_text(req, buf, s.n);
}

static esp_err_t fk_get(httpd_req_t *req) {
    sensor_snapshot_t d;
    sensors_get(&d);
    char buf[320];
    sbuf_t s = {buf, sizeof buf, 0};
    put(&s, "{\"valid\":%s,\"dragButtonPressed\":%s,", b(d.valid), b(d.button));
    put_matrix(&s, d.T);
    put(&s, "}");
    return send_json_text(req, buf, s.n);
}

static esp_err_t dh_get(httpd_req_t *req) {
    char buf[512];
    sbuf_t s = {buf, sizeof buf, 0};
    put(&s, "{\"dhParams\":[");
    for (int i = 0; i < 6; i++) {
        put(&s, "%s{\"joint\":%d,\"a\":%.6f,\"alpha_deg\":%.2f,\"d\":%.6f,\"theta_deg\":%.2f}", i ? "," : "", i + 1,
            DH_PARAMS[i].a, DH_PARAMS[i].alpha_deg, DH_PARAMS[i].d, DH_PARAMS[i].theta_deg);
    }
    put(&s, "]}");
    return send_json_text(req, buf, s.n);
}

// ---------------------------------------------------------------- serial output (as in the Arduino version)

static volatile bool serial_angles, serial_fk;
static volatile int serial_interval_ms = 100;

static bool query_value(httpd_req_t *req, const char *key, char *out, size_t cap) {
    char q[64];
    return httpd_req_get_url_query_str(req, q, sizeof q) == ESP_OK && httpd_query_key_value(q, key, out, cap) == ESP_OK;
}

static esp_err_t serial_toggle(httpd_req_t *req, volatile bool *flag, const char *what) {
    char v[8];
    if (!query_value(req, "enable", v, sizeof v)) {
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_sendstr(req, "Missing 'enable' parameter");
    }
    *flag = strcmp(v, "1") == 0 || strcmp(v, "true") == 0;
    char msg[48];
    snprintf(msg, sizeof msg, "%s output %s", what, *flag ? "enabled" : "disabled");
    return httpd_resp_sendstr(req, msg);
}

static esp_err_t serial_angles_get(httpd_req_t *req) {
    return serial_toggle(req, &serial_angles, "Angles");
}

static esp_err_t serial_fk_get(httpd_req_t *req) {
    return serial_toggle(req, &serial_fk, "FK");
}

static esp_err_t serial_freq_get(httpd_req_t *req) {
    char v[16], msg[96];
    if (query_value(req, "hz", v, sizeof v)) {
        float hz = strtof(v, NULL);
        if (!(hz > 0 && hz <= 1000)) {
            httpd_resp_set_status(req, "400 Bad Request");
            return httpd_resp_sendstr(req, "Frequency must be between 0.1 and 1000 Hz");
        }
        serial_interval_ms = (int)(1000.0f / hz);
        if (serial_interval_ms < 1) serial_interval_ms = 1;
        snprintf(msg, sizeof msg, "Serial output frequency set to %.1f Hz (%d ms)", hz, serial_interval_ms);
    } else {
        snprintf(msg, sizeof msg, "Current: %.1f Hz (%d ms)", 1000.0f / serial_interval_ms, serial_interval_ms);
    }
    return httpd_resp_sendstr(req, msg);
}

static void serial_task(void *arg) {
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(serial_interval_ms));
        if (!serial_angles && !serial_fk) continue;
        sensor_snapshot_t d;
        sensors_get(&d);
        if (!d.valid) continue;
        unsigned long ms = (unsigned long)(esp_timer_get_time() / 1000);
        if (serial_angles) {
            printf("$ANGLES: %lu, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n", ms, d.angles_deg[0], d.angles_deg[1],
                   d.angles_deg[2], d.angles_deg[3], d.angles_deg[4], d.angles_deg[5]);
        }
        if (serial_fk) {
            printf("$FK: %lu", ms);
            for (int r = 0; r < 4; r++) {
                for (int c = 0; c < 4; c++) printf(", %.6f", d.T[r][c]);
            }
            printf("\n");
        }
    }
}

// ---------------------------------------------------------------- live stream (WebSocket)

#define MAX_WS_CLIENTS 4
#define STREAM_PERIOD_MS 20
static httpd_handle_t server;
static int ws_fds[MAX_WS_CLIENTS] = {-1, -1, -1, -1};
static SemaphoreHandle_t ws_lock;

static void ws_add(int fd) {
    xSemaphoreTake(ws_lock, portMAX_DELAY);
    bool known = false;
    for (int i = 0; i < MAX_WS_CLIENTS; i++) known |= ws_fds[i] == fd;
    for (int i = 0; i < MAX_WS_CLIENTS && !known; i++) {
        if (ws_fds[i] < 0) { ws_fds[i] = fd; known = true; }
    }
    if (!known) httpd_sess_trigger_close(server, fd);   // too many viewers: the page falls back to polling
    xSemaphoreGive(ws_lock);
}

// The server answers the handshake itself and calls this for frames the page sends; the first ("stream") registers it.
static esp_err_t ws_handler(httpd_req_t *req) {
    httpd_ws_frame_t frame = {0};
    uint8_t buf[32];
    frame.payload = buf;
    esp_err_t e = httpd_ws_recv_frame(req, &frame, sizeof buf);
    if (e != ESP_OK) return e;
    touch_client(req);
    ws_add(httpd_req_to_sockfd(req));
    return ESP_OK;
}

static void stream_task(void *arg) {
    char buf[1024];
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(STREAM_PERIOD_MS));
        xSemaphoreTake(ws_lock, portMAX_DELAY);
        int fds[MAX_WS_CLIENTS], count = 0;
        for (int i = 0; i < MAX_WS_CLIENTS; i++) if (ws_fds[i] >= 0) fds[count++] = ws_fds[i];
        xSemaphoreGive(ws_lock);
        if (!count) continue;
        int n = build_all_data(buf, sizeof buf);
        if (n < 0) continue;
        httpd_ws_frame_t frame = {.type = HTTPD_WS_TYPE_TEXT, .payload = (uint8_t *)buf, .len = n};
        for (int k = 0; k < count; k++) {
            bool alive = httpd_ws_get_fd_info(server, fds[k]) == HTTPD_WS_CLIENT_WEBSOCKET &&
                         httpd_ws_send_frame_async(server, fds[k], &frame) == ESP_OK;
            if (!alive) {
                xSemaphoreTake(ws_lock, portMAX_DELAY);
                for (int i = 0; i < MAX_WS_CLIENTS; i++) if (ws_fds[i] == fds[k]) ws_fds[i] = -1;
                xSemaphoreGive(ws_lock);
            }
        }
    }
}

static int ws_viewers(void) {
    xSemaphoreTake(ws_lock, portMAX_DELAY);
    int n = 0;
    for (int i = 0; i < MAX_WS_CLIENTS; i++) n += ws_fds[i] >= 0;
    xSemaphoreGive(ws_lock);
    return n;
}

// ---------------------------------------------------------------- state and calibration

static void add_zero(cJSON *j) {
    int16_t z[6];
    sensors_get_zero(z);
    cJSON *a = cJSON_AddArrayToObject(j, "zero");
    for (int i = 0; i < 6; i++) cJSON_AddItemToArray(a, cJSON_CreateNumber(z[i]));
    cJSON *d = cJSON_AddArrayToObject(j, "default_zero");
    for (int i = 0; i < 6; i++) cJSON_AddItemToArray(d, cJSON_CreateNumber(SENSORS_DEFAULT_ZERO[i]));
    cJSON_AddBoolToObject(j, "factory_zero", sensors_using_default_zero());
}

static esp_err_t state_get(httpd_req_t *req) {
    sensor_snapshot_t d;
    sensors_get(&d);
    closer_demo_status_t st;
    closer_demo_get_status(&st);
    cJSON *j = cJSON_CreateObject();
    cJSON_AddStringToObject(j, "fw", closer_demo_version());
    cJSON_AddBoolToObject(j, "adc_ok", d.valid);
    cJSON_AddBoolToObject(j, "display_ok", display_present());
    cJSON_AddNumberToObject(j, "battery", d.battery_v);
    cJSON_AddNumberToObject(j, "hz", d.rate_hz);
    cJSON_AddBoolToObject(j, "wifi", st.connected);
    cJSON_AddStringToObject(j, "ip", st.ip);
    cJSON_AddNumberToObject(j, "rssi", st.rssi);
    cJSON_AddNumberToObject(j, "viewers", ws_viewers());
    cJSON_AddStringToObject(j, "note", st.note);
    add_zero(j);
    return send_json(req, j);
}

static esp_err_t zero_post(httpd_req_t *req) {
    char v[8];
    bool reset = query_value(req, "reset", v, sizeof v) && v[0] == '1';
    bool ok = reset ? sensors_set_zero(SENSORS_DEFAULT_ZERO) : sensors_zero_here();
    if (!ok) {
        httpd_resp_set_status(req, reset ? "500 Internal Server Error" : "409 Conflict");
        return httpd_resp_sendstr(req, reset ? "could not save" : "the potentiometers must be readable to set the zero");
    }
    return httpd_resp_sendstr(req, reset ? "factory zero restored" : "zero saved");
}

static esp_err_t cal_get(httpd_req_t *req) {
    cJSON *j = cJSON_CreateObject();
    add_zero(j);
    return send_json(req, j);
}

static esp_err_t cal_post(httpd_req_t *req) {
    char body[256];
    if (req->content_len <= 0 || req->content_len >= (int)sizeof body) return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad body");
    int got = 0;
    while (got < req->content_len) {
        int r = httpd_req_recv(req, body + got, req->content_len - got);
        if (r <= 0) return ESP_FAIL;
        got += r;
    }
    body[got] = 0;
    cJSON *j = cJSON_Parse(body);
    cJSON *a = cJSON_GetObjectItem(j, "zero");
    int16_t z[6];
    bool ok = cJSON_IsArray(a) && cJSON_GetArraySize(a) == 6;
    for (int i = 0; ok && i < 6; i++) {
        cJSON *v = cJSON_GetArrayItem(a, i);
        ok = cJSON_IsNumber(v) && v->valuedouble >= 0 && v->valuedouble <= 1023;
        if (ok) z[i] = (int16_t)v->valuedouble;
    }
    cJSON_Delete(j);
    if (!ok) return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "need {\"zero\": [6 numbers 0..1023]}");
    if (!sensors_set_zero(z)) return httpd_resp_send_500(req);
    return httpd_resp_sendstr(req, "saved");
}

// ---------------------------------------------------------------- start

static void stats_task(void *arg) {
    for (;;) {
        sensor_snapshot_t d;
        sensors_get(&d);
        closer_demo_set_clients(active_clients());
        closer_demo_set_stat("adc_ok", d.valid);
        closer_demo_set_stat("display_ok", display_present());
        closer_demo_set_stat("battery_v", d.battery_v);
        closer_demo_set_stat("rate_hz", d.rate_hz);
        closer_demo_set_stat("button", d.button);
        closer_demo_set_stat("x_mm", d.T[0][3] * 1000);
        closer_demo_set_stat("y_mm", d.T[1][3] * 1000);
        closer_demo_set_stat("z_mm", d.T[2][3] * 1000);
        closer_demo_set_stat("viewers", ws_viewers());
        if (!d.valid) closer_demo_set_message("MCP3008 (potentiometer ADC) not found: check its wires");
        else if (!display_present()) closer_demo_set_message("display not found");
        else closer_demo_set_message("");
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void web_start(void) {
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.lru_purge_enable = true;
    cfg.uri_match_fn = httpd_uri_match_wildcard;
    cfg.max_uri_handlers = 20;
    cfg.stack_size = 6144;
    cfg.send_wait_timeout = 2;   // a stalled viewer must not hold the stream up for long
    ws_lock = xSemaphoreCreateMutex();
    esp_err_t started = httpd_start(&server, &cfg);
    if (started != ESP_OK) {
        ESP_LOGE(TAG, "web server did not start: %s", esp_err_to_name(started));   // sensors, display, reports carry on
        return;
    }
    const httpd_uri_t uris[] = {
        {.uri = "/allData", .method = HTTP_GET, .handler = all_data_get},
        {.uri = "/angles", .method = HTTP_GET, .handler = angles_get},
        {.uri = "/rawAdcValues", .method = HTTP_GET, .handler = raw_get},
        {.uri = "/filteredAdcValues", .method = HTTP_GET, .handler = filtered_get},
        {.uri = "/fk", .method = HTTP_GET, .handler = fk_get},
        {.uri = "/dhParams", .method = HTTP_GET, .handler = dh_get},
        {.uri = "/serialAngles", .method = HTTP_GET, .handler = serial_angles_get},
        {.uri = "/serialFK", .method = HTTP_GET, .handler = serial_fk_get},
        {.uri = "/serialFrequency", .method = HTTP_GET, .handler = serial_freq_get},
        {.uri = "/ws", .method = HTTP_GET, .handler = ws_handler, .is_websocket = true},
        {.uri = "/api/state", .method = HTTP_GET, .handler = state_get},
        {.uri = "/api/zero", .method = HTTP_POST, .handler = zero_post},
        {.uri = "/api/calibration", .method = HTTP_GET, .handler = cal_get},
        {.uri = "/api/calibration", .method = HTTP_POST, .handler = cal_post},
        {.uri = "/*", .method = HTTP_GET, .handler = static_get},   // last: the page files
    };
    for (size_t i = 0; i < sizeof uris / sizeof uris[0]; i++) {
        if (httpd_register_uri_handler(server, &uris[i]) != ESP_OK) ESP_LOGE(TAG, "could not register %s", uris[i].uri);
    }
    xTaskCreate(stats_task, "stats", 4096, NULL, 2, NULL);
    xTaskCreate(stream_task, "stream", 4096, NULL, 4, NULL);
    xTaskCreate(serial_task, "serial", 3072, NULL, 2, NULL);
    ESP_LOGI(TAG, "web server up");
}

#include <string.h>
#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_http_client.h"
#include "nvs_flash.h"
#include "esp_heap_caps.h"
#include "lwip/sockets.h"
#include "driver/ledc.h"
#include "esp_timer.h"
#include <stdlib.h>
#include "freertos/semphr.h"

#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "driver/spi_master.h"

#include "i2c_driver.h"
#include "tca9554.h"
#include "esp_lcd_spd2010.h"
#include "tjpgd.h"
#include "spd2010_touch.h"

/* ── Configuration ────────────────────────────────────────────────────────── */

#define WIFI_SSID       "AIR-A01-BHC204544"
#define WIFI_PASS       "33732272"
#define CAM_IP          "192.168.0.10"
#define LV_PORT         65001
#define JPEG_BUF_SIZE   (256 * 1024)
#define PKT_BUF_SIZE    4096
#define PUSHEVENT_PORT  9001

/* Display modes */
typedef enum {
    DISPLAY_CENTERED,   /* 1:1, image centred with black borders */
    DISPLAY_FILL_WIDTH, /* upscale to fill LCD width, letterbox top/bottom */
} display_mode_t;

static display_mode_t s_display_mode   = DISPLAY_FILL_WIDTH;
static volatile bool  s_wifi_connected = false;
static bool           s_ring_on_fb     = false;

/* Display */
#define LCD_W           412
#define LCD_H           412
#define LCD_SCK         40
#define LCD_D0          46
#define LCD_D1          45
#define LCD_D2          42
#define LCD_D3          41
#define LCD_CS          21
#define LCD_BL          5
#define LCD_SPI_CLK_HZ  (80 * 1000 * 1000)

static const char *TAG = "airview";
static EventGroupHandle_t s_wifi_events;
static SemaphoreHandle_t  s_http_mutex = NULL;
#define WIFI_CONNECTED_BIT BIT0

static esp_lcd_panel_handle_t s_panel = NULL;

/* ── Shooting mode ────────────────────────────────────────────────────────── */

/* Display name and OPC API value are separate — "iA" shows on screen, "iAuto" goes to camera */
static const char *s_mode_display[] = {"P", "A", "S", "M", "iA"};
static const char *s_mode_api[]     = {"P", "A", "S", "M", "iAuto"};
#define NUM_MODES 5

static int            s_shoot_mode  = 0;

typedef struct { const char *api; const char *label; } wb_cam_t;
static const wb_cam_t s_wb_cam[] = {
    {"WB_AUTO",           "AWB"},
    {"MWB_FINE",          "SUN"},
    {"MWB_SHADE",         "SHADE"},
    {"MWB_CLOUD",         "CLOUD"},
    {"MWB_LAMP",          "TUNG"},
    {"MWB_FLUORESCENCE1", "FLUO"},
    {"MWB_WATER_1",       "WATER"},
    {"WB_CUSTOM1",        "CUSTM"},
};
static const int s_wb_cam_n = (int)(sizeof(s_wb_cam) / sizeof(s_wb_cam[0]));
static int      s_wb_idx   = 0;

#define EXPREV_CAM_MAX 32
typedef struct { char str[8]; } exprev_cam_t;
static exprev_cam_t s_exprev_cam[EXPREV_CAM_MAX];
static int          s_exprev_cam_n = 0;
static int          s_exprev_idx   = 0;

static char s_battery_str[8] = "---";
static bool s_charging        = false;

static volatile bool  s_tap_pending = false;
static volatile uint16_t s_tap_x   = 0;
static volatile uint16_t s_tap_y   = 0;


static void touch_task(void *arg)
{
    uint16_t x, y;
    while (true) {
        if (touch_poll_tap(&x, &y)) {
            s_tap_x       = x;
            s_tap_y       = y;
            s_tap_pending = true;
        }
        vTaskDelay(pdMS_TO_TICKS(10)); /* 100 Hz */
    }
}

/* ── OSD camera data ──────────────────────────────────────────────────────── */

static struct {
    int32_t shutter_num;
    int32_t shutter_denom;
    int32_t fnum_x10;
    int32_t iso;
    bool    valid;
} s_osd;

/* AF box reported by the camera each frame via RTP extension header (func_id 2).
   Written by parse_rtp_ext, read by decode_and_display — both in the liveview task. */
static int32_t s_af_color = 0;  /* 0=none, 1=succeeded(green), 2=failed(red) */
static int32_t s_af_lv_x  = 0, s_af_lv_y = 0;
static int32_t s_af_lv_w  = 0, s_af_lv_h = 0;

typedef struct { int lv_x, lv_y; } af_req_t;
static QueueHandle_t s_af_queue   = NULL;

#define PROP_NAME_MAX 32
static QueueHandle_t s_prop_queue = NULL;


static int s_selected_field = -1;   /* -1=none, 0=shutter, 1=aperture, 2=ISO, 3=exprev */

/* ── WiFi channel cache ───────────────────────────────────────────────────── */

static uint8_t channel_cache_load(void)
{
    nvs_handle_t h;
    uint8_t ch = 0;
    if (nvs_open("airview", NVS_READONLY, &h) == ESP_OK) {
        nvs_get_u8(h, "wifi_ch", &ch);
        nvs_close(h);
    }
    return ch;
}

static void channel_cache_save(uint8_t ch)
{
    nvs_handle_t h;
    if (nvs_open("airview", NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u8(h, "wifi_ch", ch);
        nvs_commit(h);
        nvs_close(h);
    }
}

/* ── WiFi credential NVS storage ─────────────────────────────────────────── */

static void wifi_creds_load(char *ssid, char *pass)
{
    strncpy(ssid, WIFI_SSID, 32); ssid[32] = '\0';
    strncpy(pass, WIFI_PASS, 64); pass[64] = '\0';
    nvs_handle_t h;
    if (nvs_open("airview", NVS_READONLY, &h) == ESP_OK) {
        size_t len = 33;
        nvs_get_str(h, "wifi_ssid", ssid, &len);
        len = 65;
        nvs_get_str(h, "wifi_pass", pass, &len);
        nvs_close(h);
    }
}

static void wifi_creds_save(const char *ssid, const char *pass)
{
    nvs_handle_t h;
    if (nvs_open("airview", NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_str(h, "wifi_ssid", ssid);
        nvs_set_str(h, "wifi_pass", pass);
        nvs_commit(h);
        nvs_close(h);
    }
}

/* ── WiFi ─────────────────────────────────────────────────────────────────── */

static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *evt = (wifi_event_sta_disconnected_t *)data;
        ESP_LOGW(TAG, "disconnected (reason %u), retrying", (unsigned)evt->reason);
        s_wifi_connected = false;
        xEventGroupClearBits(s_wifi_events, WIFI_CONNECTED_BIT);
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *evt = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "got IP: " IPSTR, IP2STR(&evt->ip_info.ip));
        uint8_t primary; wifi_second_chan_t second;
        if (esp_wifi_get_channel(&primary, &second) == ESP_OK && primary != 0) {
            channel_cache_save(primary);
            ESP_LOGI(TAG, "cached wifi channel %u", primary);
        }
        s_wifi_connected = true;
        xEventGroupSetBits(s_wifi_events, WIFI_CONNECTED_BIT);
    }
}

static void wifi_driver_init(void)
{
    s_wifi_events = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    /* RAM-only storage: no NVS read/write during WPA2 handshake.
       NVS flash I/O on first connect races the EAPOL handler and stalls
       the 4-way handshake until the camera's 10 s timeout evicts us. */
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    esp_wifi_set_ps(WIFI_PS_NONE);
    ESP_ERROR_CHECK(esp_wifi_start());
}

/* Full WiFi init + connect start, NON-BLOCKING.
   Registers event handlers before esp_wifi_start() so the STA_START event
   triggers esp_wifi_connect() automatically — no extra delay to first connect.
   Caller waits for WIFI_CONNECTED_BIT in s_wifi_events when it's ready to block. */
static void wifi_begin_connecting(void)
{
    s_wifi_events = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    /* RAM-only storage avoids NVS flash I/O racing the EAPOL handler. */
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

    /* Register handlers BEFORE start so STA_START → esp_wifi_connect() fires
       without any extra round-trip through app_main. */
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                               wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                               wifi_event_handler, NULL));

    char ssid[33], pass[65];
    wifi_creds_load(ssid, pass);
    uint8_t cached_ch = channel_cache_load();
    if (cached_ch)
        ESP_LOGI(TAG, "trying cached channel %u first", cached_ch);

    wifi_config_t wifi_cfg = {
        .sta = {
            .scan_method        = WIFI_FAST_SCAN,
            .channel            = cached_ch,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    strncpy((char *)wifi_cfg.sta.ssid,     ssid, sizeof(wifi_cfg.sta.ssid));
    strncpy((char *)wifi_cfg.sta.password, pass, sizeof(wifi_cfg.sta.password));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    esp_wifi_set_ps(WIFI_PS_NONE);
    ESP_ERROR_CHECK(esp_wifi_start());
    /* STA_START fires → wifi_event_handler calls esp_wifi_connect(). */
    ESP_LOGI(TAG, "connecting to %s (non-blocking)...", ssid);
}

/* ── HTTP helper ──────────────────────────────────────────────────────────── */

static char s_resp[2048];

static esp_err_t http_evt(esp_http_client_event_t *evt)
{
    if (evt->event_id == HTTP_EVENT_ON_DATA && evt->data_len > 0) {
        int n = evt->data_len < (int)sizeof(s_resp) - 1 ? evt->data_len : (int)sizeof(s_resp) - 1;
        memcpy(s_resp, evt->data, n);
        s_resp[n] = '\0';
    }
    return ESP_OK;
}

/* Minimum gap between any two HTTP calls. The camera's embedded server
   cannot handle rapid-fire connections and stops responding after a burst. */
#define HTTP_MIN_GAP_MS 150

static int64_t s_http_last_us = 0;

/* Enforce minimum inter-call gap. Caller must hold s_http_mutex. */
static void http_throttle(void)
{
    if (s_http_last_us) {
        int64_t wait_us = (HTTP_MIN_GAP_MS * 1000LL) - (esp_timer_get_time() - s_http_last_us);
        if (wait_us > 0)
            vTaskDelay(pdMS_TO_TICKS(wait_us / 1000 + 1));
    }
}

/* Internal GET — caller must hold s_http_mutex */
static int cam_get_impl(const char *path)
{
    http_throttle();

    char url[256];
    snprintf(url, sizeof(url), "http://%s%s", CAM_IP, path);
    esp_http_client_config_t cfg = {
        .url = url, .event_handler = http_evt, .timeout_ms = 2000,
    };
    s_resp[0] = '\0';
    esp_http_client_handle_t c = esp_http_client_init(&cfg);
    esp_http_client_set_header(c, "User-Agent", "OlympusCameraKit");
    esp_http_client_set_header(c, "X-Protocol", "OlympusCameraKit");
    esp_err_t err = esp_http_client_perform(c);
    int status = 0;
    if (err == ESP_OK) {
        status = esp_http_client_get_status_code(c);
        ESP_LOGI(TAG, "GET %s  →  %d", path, status);
    } else {
        ESP_LOGE(TAG, "GET %s failed: %s", path, esp_err_to_name(err));
    }
    esp_http_client_cleanup(c);

    s_http_last_us = esp_timer_get_time();
    return status;
}

/* Fire a GET on an already-open client handle (no throttle, no init/cleanup).
   Caller must hold s_http_mutex. Used to batch sequential requests on one TCP
   connection so the inter-request throttle is not paid for each call. */
static int cam_get_on(esp_http_client_handle_t c, const char *path)
{
    char url[256];
    snprintf(url, sizeof(url), "http://%s%s", CAM_IP, path);
    s_resp[0] = '\0';
    esp_http_client_set_url(c, url);
    esp_err_t err = esp_http_client_perform(c);
    int status = 0;
    if (err == ESP_OK) {
        status = esp_http_client_get_status_code(c);
        ESP_LOGI(TAG, "GET %s  →  %d", path, status);
    } else {
        ESP_LOGE(TAG, "GET %s failed: %s", path, esp_err_to_name(err));
    }
    return status;
}

static int cam_get(const char *path)
{
    xSemaphoreTake(s_http_mutex, portMAX_DELAY);
    int r = cam_get_impl(path);
    xSemaphoreGive(s_http_mutex);
    return r;
}

static bool cam_get_prop(const char *name, char *val, size_t val_len)
{
    char path[128];
    snprintf(path, sizeof(path), "/get_camprop.cgi?com=get&propname=%s", name);
    xSemaphoreTake(s_http_mutex, portMAX_DELAY);
    int  st     = cam_get_impl(path);
    bool ok     = (st == 200 || st == 520);
    bool parsed = false;
    if (ok) {
        const char *p = strstr(s_resp, "<value>");
        if (p) {
            p += 7;
            const char *end = strstr(p, "</value>");
            if (end) {
                size_t n = (size_t)(end - p);
                if (n >= val_len) n = val_len - 1;
                memcpy(val, p, n);
                val[n] = '\0';
                parsed = true;
            }
        }
        if (!parsed)
            ESP_LOGW(TAG, "get %s [%d] no <value> in: %s", name, st, s_resp);
    }
    xSemaphoreGive(s_http_mutex);
    return parsed;
}

static bool cam_set_prop(const char *name, const char *value)
{
    char url[256];
    snprintf(url, sizeof(url), "http://%s/set_camprop.cgi?com=setlist", CAM_IP);
    char body[256];
    snprintf(body, sizeof(body),
             "<?xml version=\"1.0\"?><set>"
             "<prop name=\"%s\"><value>%s</value></prop>"
             "</set>", name, value);

    esp_http_client_config_t cfg = {
        .url            = url,
        .method         = HTTP_METHOD_POST,
        .event_handler  = http_evt,
        .timeout_ms     = 2000,
    };
    xSemaphoreTake(s_http_mutex, portMAX_DELAY);
    http_throttle();
    s_resp[0] = '\0';
    esp_http_client_handle_t c = esp_http_client_init(&cfg);
    esp_http_client_set_header(c, "User-Agent", "OlympusCameraKit");
    esp_http_client_set_header(c, "X-Protocol", "OlympusCameraKit");
    esp_http_client_set_header(c, "Content-Type", "text/xml");
    esp_http_client_set_post_field(c, body, strlen(body));
    esp_err_t err = esp_http_client_perform(c);
    int status = 0;
    if (err == ESP_OK) {
        status = esp_http_client_get_status_code(c);
        ESP_LOGI(TAG, "set %s=%s → %d  body=[%s]", name, value, status, s_resp);
    } else {
        ESP_LOGE(TAG, "set %s failed: %s", name, esp_err_to_name(err));
    }
    esp_http_client_cleanup(c);
    s_http_last_us = esp_timer_get_time();
    xSemaphoreGive(s_http_mutex);
    return (status == 200 || status == 520);
}

/* Download binary data (JPEG, XML) directly into a caller-supplied buffer.
   Returns number of bytes received, or 0 on error. */
/* ── Field selection tables ───────────────────────────────────────────────── */

#define ARRAY_SIZE(a) ((int)(sizeof(a) / sizeof((a)[0])))

/* Fallback tables (1/3-stop CIPA standard) used when the camera's getlist
   is unavailable.  All values confirmed against Olympus OPC field reports. */
static const struct { int32_t x10; const char *api; } s_fb_fnum[] = {
    {10,"1.0"},{11,"1.1"},{12,"1.2"},{14,"1.4"},{16,"1.6"},{18,"1.8"},
    {20,"2.0"},{22,"2.2"},{25,"2.5"},{28,"2.8"},{32,"3.2"},{35,"3.5"},
    {40,"4.0"},{45,"4.5"},{50,"5.0"},{56,"5.6"},{63,"6.3"},{71,"7.1"},
    {80,"8.0"},{90,"9.0"},{100,"10.0"},{110,"11.0"},{130,"13.0"},
    {140,"14.0"},{160,"16.0"},{180,"18.0"},{200,"20.0"},{220,"22.0"},
};

/* Sub-second: denominator only (e.g. "250" = 1/250s).
   Whole seconds: number + double-quote (e.g. "2\"" = 2s). */
static const struct { int32_t num; int32_t denom; const char *api; } s_fb_shutter[] = {
    {1,4000,"4000"},{1,3200,"3200"},{1,2500,"2500"},{1,2000,"2000"},
    {1,1600,"1600"},{1,1250,"1250"},{1,1000,"1000"},{1,800,"800"},
    {1,640,"640"},  {1,500,"500"},  {1,400,"400"},  {1,320,"320"},
    {1,250,"250"},  {1,200,"200"},  {1,160,"160"},  {1,125,"125"},
    {1,100,"100"},  {1,80,"80"},    {1,60,"60"},    {1,50,"50"},
    {1,40,"40"},    {1,30,"30"},    {1,25,"25"},    {1,20,"20"},
    {1,15,"15"},    {1,13,"13"},    {1,10,"10"},    {1,8,"8"},
    {1,6,"6"},      {1,5,"5"},      {1,4,"4"},      {1,2,"2"},
    {1,1,"1\""},    {2,1,"2\""},    {4,1,"4\""},    {8,1,"8\""},
    {15,1,"15\""},  {30,1,"30\""},
};

static const struct { int32_t val; const char *api; } s_fb_iso[] = {
    {100,"100"},{125,"125"},{160,"160"},{200,"200"},{250,"250"},{320,"320"},
    {400,"400"},{500,"500"},{640,"640"},{800,"800"},{1000,"1000"},
    {1250,"1250"},{1600,"1600"},{2000,"2000"},{2500,"2500"},{3200,"3200"},
};

/* Dynamic lists populated from getlist, or from fallback tables above. */
#define CAM_LIST_MAX 64

typedef struct { char str[12]; int32_t x10;           } fnum_cam_t;
typedef struct { char str[12]; int32_t num; int32_t denom; } shutter_cam_t;
typedef struct { char str[8];  int32_t val;           } iso_cam_t;

static fnum_cam_t    s_fnum_cam[CAM_LIST_MAX];
static int           s_fnum_cam_n = 0;

static shutter_cam_t s_shutter_cam[CAM_LIST_MAX];
static int           s_shutter_cam_n = 0;

static iso_cam_t     s_iso_cam[CAM_LIST_MAX];
static int           s_iso_cam_n = 0;

/* Parse <item>VALUE</item> sequences out of an already-captured s_resp.
   Caller must hold s_http_mutex (or know s_resp is stable). */
static int parse_item_list(const char *xml, char out[][12], int out_max, int item_max)
{
    int n = 0;
    const char *p = xml;
    while (n < out_max) {
        p = strstr(p, "<item>");
        if (!p) break;
        p += 6;
        const char *e = strstr(p, "</item>");
        if (!e) break;
        int len = (int)(e - p);
        if (len > 0 && len < item_max) {
            memcpy(out[n], p, len);
            out[n][len] = '\0';
            n++;
        }
        p = e + 7;
    }
    return n;
}

/* Parse space-separated tokens from an <enum>...</enum> block (com=desc response). */
static int parse_enum_list(const char *xml, char out[][20], int out_max)
{
    int n = 0;
    const char *s = strstr(xml, "<enum>");
    if (!s) return 0;
    s += 6;
    const char *e = strstr(s, "</enum>");
    if (!e) return 0;
    while (s < e && n < out_max) {
        while (s < e && *s == ' ') s++;
        if (s >= e) break;
        const char *t = s;
        while (t < e && *t != ' ') t++;
        int len = (int)(t - s);
        if (len > 0 && len < 20) {
            memcpy(out[n], s, len);
            out[n][len] = '\0';
            n++;
        }
        s = t;
    }
    return n;
}

/* Query the camera's permitted value lists for the three adjustable properties.
   Results are stored in s_fnum_cam / s_shutter_cam / s_iso_cam.
   The camera returns HTTP 520 for all commands (including GETs), so we accept
   both 200 and 520.  We hold the mutex across the HTTP call + parse to keep
   s_resp stable. */
static void build_prop_lists(void)
{
    char items[CAM_LIST_MAX][12];
    int n, st;

    /* APERTURE ------------------------------------------------------------ */
    s_fnum_cam_n = 0;
    xSemaphoreTake(s_http_mutex, portMAX_DELAY);
    st = cam_get_impl("/get_camprop.cgi?com=getlist&propname=APERTURE");
    n = (st == 200 || st == 520) ? parse_item_list(s_resp, items, CAM_LIST_MAX, 12) : 0;
    xSemaphoreGive(s_http_mutex);
    for (int i = 0; i < n && s_fnum_cam_n < CAM_LIST_MAX; i++) {
        fnum_cam_t *v = &s_fnum_cam[s_fnum_cam_n++];
        memcpy(v->str, items[i], sizeof(v->str));
        int whole = atoi(v->str);
        const char *dot = strchr(v->str, '.');
        int frac = (dot && dot[1] >= '0' && dot[1] <= '9') ? (dot[1] - '0') : 0;
        v->x10 = whole * 10 + frac;
    }
    if (s_fnum_cam_n == 0) {
        /* getlist not supported — use 1/3-stop fallback table */
        int fb_n = ARRAY_SIZE(s_fb_fnum);
        if (fb_n > CAM_LIST_MAX) fb_n = CAM_LIST_MAX;
        for (int i = 0; i < fb_n; i++) {
            s_fnum_cam[i].x10 = s_fb_fnum[i].x10;
            strncpy(s_fnum_cam[i].str, s_fb_fnum[i].api, sizeof(s_fnum_cam[i].str) - 1);
            s_fnum_cam[i].str[sizeof(s_fnum_cam[i].str) - 1] = '\0';
        }
        s_fnum_cam_n = fb_n;
    }
    ESP_LOGI(TAG, "focalvalue: %d entries (first=%s last=%s)",
             s_fnum_cam_n,
             s_fnum_cam_n > 0 ? s_fnum_cam[0].str : "-",
             s_fnum_cam_n > 0 ? s_fnum_cam[s_fnum_cam_n - 1].str : "-");

    /* SHUTTER ------------------------------------------------------------- */
    s_shutter_cam_n = 0;
    xSemaphoreTake(s_http_mutex, portMAX_DELAY);
    st = cam_get_impl("/get_camprop.cgi?com=getlist&propname=SHUTTER");
    n = (st == 200 || st == 520) ? parse_item_list(s_resp, items, CAM_LIST_MAX, 12) : 0;
    xSemaphoreGive(s_http_mutex);
    for (int i = 0; i < n && s_shutter_cam_n < CAM_LIST_MAX; i++) {
        shutter_cam_t *v = &s_shutter_cam[s_shutter_cam_n++];
        memcpy(v->str, items[i], sizeof(v->str));
        const char *slash = strchr(v->str, '/');
        const char *quot  = strchr(v->str, '"');
        if (slash)      { v->num = atoi(v->str); v->denom = atoi(slash + 1); }
        else if (quot)  { v->num = atoi(v->str); v->denom = 1; }
        else            { v->num = 1;             v->denom = atoi(v->str); }
    }
    if (s_shutter_cam_n == 0) {
        int fb_n = ARRAY_SIZE(s_fb_shutter);
        if (fb_n > CAM_LIST_MAX) fb_n = CAM_LIST_MAX;
        for (int i = 0; i < fb_n; i++) {
            s_shutter_cam[i].num   = s_fb_shutter[i].num;
            s_shutter_cam[i].denom = s_fb_shutter[i].denom;
            strncpy(s_shutter_cam[i].str, s_fb_shutter[i].api, sizeof(s_shutter_cam[i].str) - 1);
            s_shutter_cam[i].str[sizeof(s_shutter_cam[i].str) - 1] = '\0';
        }
        s_shutter_cam_n = fb_n;
    }
    ESP_LOGI(TAG, "shutspeed: %d entries", s_shutter_cam_n);

    /* ISO ----------------------------------------------------------------- */
    s_iso_cam_n = 0;
    xSemaphoreTake(s_http_mutex, portMAX_DELAY);
    st = cam_get_impl("/get_camprop.cgi?com=getlist&propname=ISO");
    n = (st == 200 || st == 520) ? parse_item_list(s_resp, items, CAM_LIST_MAX, 8) : 0;
    xSemaphoreGive(s_http_mutex);
    for (int i = 0; i < n && s_iso_cam_n < CAM_LIST_MAX; i++) {
        iso_cam_t *v = &s_iso_cam[s_iso_cam_n++];
        memcpy(v->str, items[i], sizeof(v->str));
        v->val = (int32_t)atoi(v->str);
    }
    if (s_iso_cam_n == 0) {
        int fb_n = ARRAY_SIZE(s_fb_iso);
        if (fb_n > CAM_LIST_MAX) fb_n = CAM_LIST_MAX;
        for (int i = 0; i < fb_n; i++) {
            s_iso_cam[i].val = s_fb_iso[i].val;
            strncpy(s_iso_cam[i].str, s_fb_iso[i].api, sizeof(s_iso_cam[i].str) - 1);
            s_iso_cam[i].str[sizeof(s_iso_cam[i].str) - 1] = '\0';
        }
        s_iso_cam_n = fb_n;
    }
    ESP_LOGI(TAG, "isospeedvalue: %d entries", s_iso_cam_n);
}


static void build_exprev_list(void)
{
    s_exprev_cam_n = 0;
    xSemaphoreTake(s_http_mutex, portMAX_DELAY);
    int st = cam_get_impl("/get_camprop.cgi?com=desc&propname=EXPREV");
    if (st == 200 || st == 520) {
        ESP_LOGI(TAG, "EXPREV desc [%d]: %s", st, s_resp);
        char tokens[EXPREV_CAM_MAX][20];
        int n = parse_enum_list(s_resp, tokens, EXPREV_CAM_MAX);
        for (int i = 0; i < n && s_exprev_cam_n < EXPREV_CAM_MAX; i++) {
            strncpy(s_exprev_cam[s_exprev_cam_n].str, tokens[i],
                    sizeof(s_exprev_cam[0].str) - 1);
            s_exprev_cam[s_exprev_cam_n].str[sizeof(s_exprev_cam[0].str) - 1] = '\0';
            s_exprev_cam_n++;
        }
    }
    xSemaphoreGive(s_http_mutex);

    if (s_exprev_cam_n == 0) {
        static const char *fb[] = {
            "-3.0","-2.7","-2.3","-2.0","-1.7","-1.3","-1.0","-0.7","-0.3",
            "0.0",
            "+0.3","+0.7","+1.0","+1.3","+1.7","+2.0","+2.3","+2.7","+3.0",
        };
        for (int i = 0; i < (int)(sizeof(fb)/sizeof(fb[0])) && s_exprev_cam_n < EXPREV_CAM_MAX; i++) {
            strncpy(s_exprev_cam[s_exprev_cam_n].str, fb[i],
                    sizeof(s_exprev_cam[0].str) - 1);
            s_exprev_cam[s_exprev_cam_n].str[sizeof(s_exprev_cam[0].str) - 1] = '\0';
            s_exprev_cam_n++;
        }
    }
    if (s_exprev_idx >= s_exprev_cam_n) s_exprev_idx = 0;
    ESP_LOGI(TAG, "EXPREV: %d entries", s_exprev_cam_n);
}

/* Fire-and-forget cam_set_prop: spawns a task so liveview_loop is never blocked
   by the camera's HTTP server going silent during/after a mode change. */
typedef struct { char name[32]; char value[32]; } prop_set_req_t;

static void prop_set_task(void *arg)
{
    prop_set_req_t *req = (prop_set_req_t *)arg;
    cam_set_prop(req->name, req->value);
    free(req);
    vTaskDelete(NULL);
}

static void cam_set_prop_async(const char *name, const char *value)
{
    prop_set_req_t *req = malloc(sizeof(*req));
    if (!req) { ESP_LOGE(TAG, "cam_set_prop_async: alloc failed"); return; }
    strlcpy(req->name,  name,  sizeof(req->name));
    strlcpy(req->value, value, sizeof(req->value));
    if (xTaskCreate(prop_set_task, "prop_set", 3072, req, 3, NULL) != pdPASS) {
        ESP_LOGE(TAG, "cam_set_prop_async: task failed");
        free(req);
    }
}

/* field: 0=shutter, 1=aperture, 2=ISO, 3=exprev; mode: 0=P,1=A,2=S,3=M,4=iA */
static bool field_selectable(int field, int mode)
{
    if (mode == 4) return false;
    if (field == 0) return (mode == 2 || mode == 3);          /* shutter: S and M */
    if (field == 1) return (mode == 1 || mode == 3);          /* aperture: A and M */
    if (field == 2) return true;                               /* ISO: P, A, S, M */
    if (field == 3) return (mode == 0 || mode == 1 || mode == 2); /* exprev: P, A, S */
    return false;
}

static void adjust_field(int field, int delta)
{
    if (field == 0) {
        int n = s_shutter_cam_n;
        if (n == 0) { ESP_LOGW(TAG, "shutspeed list empty"); return; }

        /* Table is fastest→slowest; invert delta so right=faster, left=slower */
        delta = -delta;
        /* t_us = num*1e6/denom */
        int64_t cur_us = (int64_t)s_osd.shutter_num * 1000000
                       / (s_osd.shutter_denom ? s_osd.shutter_denom : 1);
        int idx = -1;
        for (int i = 0; i < n; i++) {
            if (s_shutter_cam[i].num   == s_osd.shutter_num &&
                s_shutter_cam[i].denom == s_osd.shutter_denom) { idx = i; break; }
        }
        if (idx >= 0) {
            idx += delta;
        } else {
            if (delta > 0) {
                idx = n - 1;
                for (int i = 0; i < n; i++) {
                    int64_t t = (int64_t)s_shutter_cam[i].num * 1000000 / s_shutter_cam[i].denom;
                    if (t > cur_us) { idx = i; break; }
                }
            } else {
                idx = 0;
                for (int i = n - 1; i >= 0; i--) {
                    int64_t t = (int64_t)s_shutter_cam[i].num * 1000000 / s_shutter_cam[i].denom;
                    if (t < cur_us) { idx = i; break; }
                }
            }
        }
        if (idx < 0) idx = 0;
        if (idx >= n) idx = n - 1;
        ESP_LOGI(TAG, "shutspeed: cur=%ld/%ld → %s",
                 (long)s_osd.shutter_num, (long)s_osd.shutter_denom, s_shutter_cam[idx].str);
        s_osd.shutter_num   = s_shutter_cam[idx].num;
        s_osd.shutter_denom = s_shutter_cam[idx].denom;
        cam_set_prop_async("SHUTTER", s_shutter_cam[idx].str);

    } else if (field == 1) {
        int n = s_fnum_cam_n;
        if (n == 0) { ESP_LOGW(TAG, "focalvalue list empty"); return; }

        int idx = -1;
        for (int i = 0; i < n; i++) {
            if (s_fnum_cam[i].x10 == s_osd.fnum_x10) { idx = i; break; }
        }
        if (idx >= 0) {
            idx += delta;
        } else {
            if (delta > 0) {
                idx = n - 1;
                for (int i = 0; i < n; i++) {
                    if (s_fnum_cam[i].x10 > s_osd.fnum_x10) { idx = i; break; }
                }
            } else {
                idx = 0;
                for (int i = n - 1; i >= 0; i--) {
                    if (s_fnum_cam[i].x10 < s_osd.fnum_x10) { idx = i; break; }
                }
            }
        }
        if (idx < 0) idx = 0;
        if (idx >= n) idx = n - 1;
        ESP_LOGI(TAG, "focalvalue: cur_x10=%ld → %s", (long)s_osd.fnum_x10, s_fnum_cam[idx].str);
        s_osd.fnum_x10 = s_fnum_cam[idx].x10;
        cam_set_prop_async("APERTURE", s_fnum_cam[idx].str);

    } else if (field == 2) {
        int n = s_iso_cam_n;
        if (n == 0) { ESP_LOGW(TAG, "isospeedvalue list empty"); return; }

        int idx = -1;
        for (int i = 0; i < n; i++) {
            if (s_iso_cam[i].val == s_osd.iso) { idx = i; break; }
        }
        if (idx >= 0) {
            idx += delta;
        } else {
            if (delta > 0) {
                idx = n - 1;
                for (int i = 0; i < n; i++) {
                    if (s_iso_cam[i].val > s_osd.iso) { idx = i; break; }
                }
            } else {
                idx = 0;
                for (int i = n - 1; i >= 0; i--) {
                    if (s_iso_cam[i].val < s_osd.iso) { idx = i; break; }
                }
            }
        }
        if (idx < 0) idx = 0;
        if (idx >= n) idx = n - 1;
        ESP_LOGI(TAG, "isospeedvalue: cur=%ld → %s", (long)s_osd.iso, s_iso_cam[idx].str);
        s_osd.iso = s_iso_cam[idx].val;
        cam_set_prop_async("ISO", s_iso_cam[idx].str);

    } else if (field == 3) {
        int n = s_exprev_cam_n;
        if (n == 0) { ESP_LOGW(TAG, "exprev list empty"); return; }

        int idx = s_exprev_idx + delta;
        if (idx < 0) idx = 0;
        if (idx >= n) idx = n - 1;
        ESP_LOGI(TAG, "exprev: %s → %s",
                 s_exprev_cam[s_exprev_idx].str, s_exprev_cam[idx].str);
        s_exprev_idx = idx;
        cam_set_prop_async("EXPREV", s_exprev_cam[idx].str);
    }
}

/* ── OPC push-event listener ──────────────────────────────────────────────── */

static int recv_exact(int sock, void *buf, int len)
{
    uint8_t *p = (uint8_t *)buf;
    int left = len;
    while (left > 0) {
        int r = recv(sock, p, left, 0);
        if (r <= 0) return -1;
        p    += r;
        left -= r;
    }
    return len;
}

static void parse_shutspeed_str(const char *val)
{
    const char *slash = strchr(val, '/');
    const char *quot  = strchr(val, '"');
    if (slash) {
        s_osd.shutter_num   = (int32_t)atoi(val);
        s_osd.shutter_denom = (int32_t)atoi(slash + 1);
    } else if (quot) {
        /* whole seconds: "2\"" → 2/1 */
        s_osd.shutter_num   = (int32_t)atoi(val);
        s_osd.shutter_denom = 1;
    } else {
        /* denominator-only: "250" → 1/250 */
        s_osd.shutter_num   = 1;
        s_osd.shutter_denom = (int32_t)atoi(val);
    }
}

static void parse_focalvalue_str(const char *val)
{
    int whole = atoi(val);
    const char *dot = strchr(val, '.');
    int frac = (dot && dot[1] >= '0' && dot[1] <= '9') ? (dot[1] - '0') : 0;
    s_osd.fnum_x10 = (int32_t)(whole * 10 + frac);
}

static void set_battery_str(const char *val)
{
    s_charging = false;
    const char *label = val;

    if      (strcmp(val, "FULL")         == 0) { label = "FULL"; }
    else if (strcmp(val, "LOW")          == 0) { label = "LOW"; }
    else if (strcmp(val, "WARNING")      == 0) { label = "WARN"; }
    else if (strcmp(val, "EMPTY")        == 0) { label = "EMPTY"; }
    else if (strcmp(val, "UNKNOWN")      == 0) { label = "UNK"; }
    else if (strcmp(val, "CHARGE")       == 0) { label = "CHRG";  s_charging = true; }
    else if (strcmp(val, "EMPTY_AC")     == 0) { label = "EMPTY"; s_charging = true; }
    else if (strcmp(val, "SUPPLY_FULL")  == 0) { label = "FULL";  s_charging = true; }
    else if (strcmp(val, "SUPPLY_LOW")   == 0) { label = "LOW";   s_charging = true; }
    else if (strcmp(val, "SUPPLY_WARNING")== 0){ label = "WARN";  s_charging = true; }

    strncpy(s_battery_str, label, sizeof(s_battery_str) - 1);
    s_battery_str[sizeof(s_battery_str) - 1] = '\0';
}

static void refresh_prop(const char *prop_name)
{
    char val[16] = {0};
    if (!cam_get_prop(prop_name, val, sizeof(val))) return;

    if (strcmp(prop_name, "SHUTTER") == 0)
        parse_shutspeed_str(val);
    else if (strcmp(prop_name, "APERTURE") == 0)
        parse_focalvalue_str(val);
    else if (strcmp(prop_name, "ISO") == 0)
        s_osd.iso = (int32_t)atoi(val);
    else if (strcmp(prop_name, "TAKEMODE") == 0) {
        for (int i = 0; i < NUM_MODES; i++) {
            if (strcmp(val, s_mode_api[i]) == 0) { s_shoot_mode = i; break; }
        }
    }
    else if (strcmp(prop_name, "WB") == 0) {
        for (int i = 0; i < s_wb_cam_n; i++) {
            if (strcmp(val, s_wb_cam[i].api) == 0) { s_wb_idx = i; break; }
        }
    }
    else if (strcmp(prop_name, "EXPREV") == 0) {
        for (int i = 0; i < s_exprev_cam_n; i++) {
            if (strcmp(val, s_exprev_cam[i].str) == 0) { s_exprev_idx = i; break; }
        }
    }
    else if (strcmp(prop_name, "BATTERY_LEVEL") == 0) {
        set_battery_str(val);
    }

    s_osd.valid = true;
    ESP_LOGI(TAG, "prop refreshed: %s = %s", prop_name, val);
}

static void battery_task(void *arg)
{
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(60000));
        char bv[12] = {0};
        if (cam_get_prop("BATTERY_LEVEL", bv, sizeof(bv))) {
            set_battery_str(bv);
            ESP_LOGI(TAG, "BATTERY_LEVEL: %s → %s", bv, s_battery_str);
        }
    }
}

static void af_task(void *arg)
{
    af_req_t req;
    for (;;) {
        if (xQueueReceive(s_af_queue, &req, portMAX_DELAY) != pdTRUE) continue;
        char url[80];
        snprintf(url, sizeof(url),
                 "/exec_takemotion.cgi?com=newstarttake&point=%04dx%04d",
                 req.lv_x, req.lv_y);
        ESP_LOGI(TAG, "touch AF → lv (%d,%d)", req.lv_x, req.lv_y);
        cam_get(url);
        /* OPC has no AF-only command. newstarttake always begins a full capture
           sequence. Calling newstoptake immediately aborts it after AF but before
           the shutter fires. 50 ms is enough for the AF result event to arrive. */
        vTaskDelay(pdMS_TO_TICKS(50));
        cam_get("/exec_takemotion.cgi?com=newstoptake");
    }
}

static void init_props_task(void *arg)
{
    build_exprev_list();   /* populates s_exprev_cam for adjustment; current idx comes from RTP ext */

    char batt_val[12] = {0};
    if (cam_get_prop("BATTERY_LEVEL", batt_val, sizeof(batt_val)))
        set_battery_str(batt_val);

    build_prop_lists();

    vTaskDelete(NULL);
}

/* Drains s_prop_queue and calls refresh_prop with a minimum gap between
   HTTP calls so the camera's embedded server isn't overwhelmed. */
static void prop_refresh_task(void *arg)
{
    char prop[PROP_NAME_MAX];
    for (;;) {
        if (xQueueReceive(s_prop_queue, prop, portMAX_DELAY) == pdTRUE)
            refresh_prop(prop);
    }
}

static void pushevent_task(void *arg)
{
    char path[64];

restart:;
    /* Tell camera to open its TCP event port; camera listens, we connect */
    snprintf(path, sizeof(path), "/start_pushevent.cgi?port=%d", PUSHEVENT_PORT);
    cam_get(path);
    /* Camera opens the TCP port after sending 200 OK; give it a moment */
    vTaskDelay(pdMS_TO_TICKS(300));

    int conn = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (conn < 0) { vTaskDelay(pdMS_TO_TICKS(2000)); goto restart; }

    struct sockaddr_in cam_addr = {
        .sin_family      = AF_INET,
        .sin_port        = htons(PUSHEVENT_PORT),
        .sin_addr.s_addr = inet_addr(CAM_IP),
    };
    if (connect(conn, (struct sockaddr *)&cam_addr, sizeof(cam_addr)) < 0) {
        ESP_LOGE(TAG, "pushevent connect failed: %d (%s)", errno, strerror(errno));
        close(conn);
        cam_get("/stop_pushevent.cgi");
        vTaskDelay(pdMS_TO_TICKS(2000));
        goto restart;
    }

    /* 30 s idle timeout — camera keeps the connection alive between events */
    struct timeval tv = { .tv_sec = 30, .tv_usec = 0 };
    setsockopt(conn, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    ESP_LOGI(TAG, "pushevent connected");

    uint8_t hdr[4];
    while (true) {
        if (recv_exact(conn, hdr, 4) < 0) break;

        uint8_t  app_id  = hdr[0];
        uint8_t  event   = hdr[1];
        uint16_t xml_len = ((uint16_t)hdr[2] << 8) | hdr[3];

        if (xml_len == 0) continue;

        char xml[257] = {0};
        if (xml_len >= sizeof(xml)) {
            /* Oversized event — drain without storing */
            uint8_t drain[64];
            int left = xml_len;
            while (left > 0) {
                int chunk = left < 64 ? left : 64;
                if (recv_exact(conn, drain, chunk) < 0) goto done;
                left -= chunk;
            }
            continue;
        }

        if (recv_exact(conn, xml, xml_len) < 0) break;
        ESP_LOGI(TAG, "pushevent appID=%u event=%u xml=[%.*s]",
                 app_id, event, (int)xml_len, xml);

        /* Event 101: AF result — location/size in liveview coords, result ok/ng */
        if (app_id == 2 && event == 101) {
            int new_color = 0;
            const char *rp = strstr(xml, "<result>");
            if (rp) {
                rp += 8;
                if   (strncmp(rp, "ok", 2) == 0) new_color = 1;
                else if (strncmp(rp, "ng", 2) == 0) new_color = 2;
            }
            const char *lp = strstr(xml, "<location>");
            const char *sp = strstr(xml, "<size>");
            if (lp && sp && new_color) {
                int lx = 0, ly = 0, sw = 0, sh = 0;
                sscanf(lp + 10, "%dx%d", &lx, &ly);
                sscanf(sp +  6, "%dx%d", &sw, &sh);
                s_af_lv_x = lx;  s_af_lv_y = ly;
                s_af_lv_w = sw;  s_af_lv_h = sh;
            }
            s_af_color = new_color;
        }

        /* Event 206: Camera Property Value Changed (appID=2) */
        if (app_id == 2 && event == 206) {
            const char *p = strstr(xml, "<prop>");
            if (p) {
                p += 6;
                const char *end = strstr(p, "</prop>");
                if (end) {
                    char prop[PROP_NAME_MAX] = {0};
                    int n = (int)(end - p);
                    if (n > 0 && n < PROP_NAME_MAX && s_prop_queue) {
                        memcpy(prop, p, n);
                        xQueueSend(s_prop_queue, prop, 0); /* drop if full */
                    }
                }
            }
        }
    }

done:
    ESP_LOGW(TAG, "pushevent disconnected");
    cam_get("/stop_pushevent.cgi");
    close(conn);
    vTaskDelay(pdMS_TO_TICKS(2000));
    goto restart;
}

/* ── Display init ─────────────────────────────────────────────────────────── */

static void display_init(void)
{
    i2c_drv_init();
    tca9554_init();

    tca9554_set(TCA9554_EXIO2, false);
    vTaskDelay(pdMS_TO_TICKS(100));
    tca9554_set(TCA9554_EXIO2, true);
    vTaskDelay(pdMS_TO_TICKS(100));

    spi_bus_config_t bus_cfg = {
        .sclk_io_num     = LCD_SCK,
        .data0_io_num    = LCD_D0,
        .data1_io_num    = LCD_D1,
        .data2_io_num    = LCD_D2,
        .data3_io_num    = LCD_D3,
        .data4_io_num    = -1,
        .data5_io_num    = -1,
        .data6_io_num    = -1,
        .data7_io_num    = -1,
        .max_transfer_sz = LCD_W * 80 * sizeof(uint16_t),
        .flags           = SPICOMMON_BUSFLAG_MASTER,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO));

    esp_lcd_panel_io_spi_config_t io_cfg = {
        .cs_gpio_num         = LCD_CS,
        .dc_gpio_num         = -1,
        .spi_mode            = 0,
        .pclk_hz             = LCD_SPI_CLK_HZ,
        .trans_queue_depth   = 10,
        .lcd_cmd_bits        = 32,
        .lcd_param_bits      = 8,
        .flags.quad_mode     = 1,
    };
    esp_lcd_panel_io_handle_t io_handle = NULL;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST, &io_cfg, &io_handle));

    spd2010_vendor_config_t vendor_cfg = { .flags.use_qspi_interface = 1 };
    esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num  = -1,
        .rgb_ele_order   = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel  = 16,
        .flags.reset_active_high = 0,
        .vendor_config   = &vendor_cfg,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_spd2010(io_handle, &panel_cfg, &s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(s_panel, true));

    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_13_BIT,
        .freq_hz         = 5000,
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .timer_num       = LEDC_TIMER_0,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
    ledc_channel_config_t ledc_ch = {
        .gpio_num   = LCD_BL,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_0,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 6553,
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_ch));

    ESP_LOGI(TAG, "display ready");
}

/* ── Splash / clear screen ────────────────────────────────────────────────── */

/* 8×8 bitmap font: bit N set → pixel N on (bit 0 = leftmost).
   Undefined entries render as blank; add chars as needed. */
static const uint8_t s_font8x8[96][8] = {
    [' '  - 0x20] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    ['"'  - 0x20] = {0x05,0x05,0x05,0x00,0x00,0x00,0x00,0x00},
    ['%'  - 0x20] = {0x03,0x03,0x08,0x04,0x02,0x18,0x18,0x00},
    ['+'  - 0x20] = {0x00,0x04,0x04,0x1F,0x04,0x04,0x00,0x00},
    ['-'  - 0x20] = {0x00,0x00,0x00,0x1F,0x00,0x00,0x00,0x00},
    ['.'  - 0x20] = {0x00,0x00,0x00,0x00,0x00,0x0C,0x0C,0x00},
    ['/'  - 0x20] = {0x10,0x08,0x08,0x04,0x04,0x02,0x02,0x00},
    ['0'  - 0x20] = {0x0E,0x11,0x11,0x11,0x11,0x11,0x0E,0x00},
    ['1'  - 0x20] = {0x04,0x06,0x04,0x04,0x04,0x04,0x1F,0x00},
    ['2'  - 0x20] = {0x0E,0x11,0x10,0x08,0x04,0x02,0x1F,0x00},
    ['3'  - 0x20] = {0x0E,0x11,0x10,0x0C,0x10,0x11,0x0E,0x00},
    ['4'  - 0x20] = {0x11,0x11,0x11,0x1F,0x10,0x10,0x10,0x00},
    ['5'  - 0x20] = {0x1F,0x01,0x01,0x0F,0x10,0x10,0x0F,0x00},
    ['6'  - 0x20] = {0x0E,0x01,0x01,0x0F,0x11,0x11,0x0E,0x00},
    ['7'  - 0x20] = {0x1F,0x10,0x08,0x04,0x04,0x04,0x04,0x00},
    ['8'  - 0x20] = {0x0E,0x11,0x11,0x0E,0x11,0x11,0x0E,0x00},
    ['9'  - 0x20] = {0x0E,0x11,0x11,0x1E,0x10,0x10,0x0E,0x00},
    ['A'  - 0x20] = {0x0E,0x11,0x11,0x1F,0x11,0x11,0x11,0x00},
    ['B'  - 0x20] = {0x0F,0x09,0x09,0x0F,0x11,0x11,0x0F,0x00},
    ['C'  - 0x20] = {0x3C,0x06,0x03,0x03,0x03,0x06,0x3C,0x00},
    ['D'  - 0x20] = {0x07,0x09,0x11,0x11,0x11,0x09,0x07,0x00},
    ['E'  - 0x20] = {0x1F,0x01,0x01,0x0F,0x01,0x01,0x1F,0x00},
    ['F'  - 0x20] = {0x1F,0x01,0x01,0x0F,0x01,0x01,0x01,0x00},
    ['G'  - 0x20] = {0x0E,0x01,0x01,0x19,0x11,0x11,0x0E,0x00},
    ['H'  - 0x20] = {0x11,0x11,0x11,0x1F,0x11,0x11,0x11,0x00},
    ['I'  - 0x20] = {0x0E,0x04,0x04,0x04,0x04,0x04,0x0E,0x00},
    ['L'  - 0x20] = {0x01,0x01,0x01,0x01,0x01,0x01,0x1F,0x00},
    ['M'  - 0x20] = {0x11,0x1B,0x15,0x11,0x11,0x11,0x11,0x00},
    ['N'  - 0x20] = {0x11,0x13,0x15,0x19,0x11,0x11,0x11,0x00},
    ['O'  - 0x20] = {0x0E,0x11,0x11,0x11,0x11,0x11,0x0E,0x00},
    ['P'  - 0x20] = {0x0F,0x11,0x11,0x0F,0x01,0x01,0x01,0x00},
    ['R'  - 0x20] = {0x0F,0x11,0x11,0x0F,0x05,0x09,0x11,0x00},
    ['S'  - 0x20] = {0x0E,0x01,0x01,0x0E,0x10,0x10,0x0E,0x00},
    ['T'  - 0x20] = {0x1F,0x04,0x04,0x04,0x04,0x04,0x04,0x00},
    ['U'  - 0x20] = {0x11,0x11,0x11,0x11,0x11,0x11,0x0E,0x00},
    ['W'  - 0x20] = {0x11,0x11,0x11,0x11,0x15,0x1B,0x11,0x00},
    ['c'  - 0x20] = {0x00,0x00,0x1E,0x02,0x02,0x02,0x1E,0x00},
    ['e'  - 0x20] = {0x00,0x00,0x1E,0x12,0x1E,0x02,0x1E,0x00},
    ['g'  - 0x20] = {0x00,0x00,0x1E,0x12,0x1E,0x10,0x1E,0x00},
    ['i'  - 0x20] = {0x0C,0x00,0x04,0x04,0x04,0x04,0x1E,0x00},
    ['n'  - 0x20] = {0x00,0x00,0x1E,0x12,0x12,0x12,0x12,0x00},
    ['o'  - 0x20] = {0x00,0x00,0x1E,0x12,0x12,0x12,0x1E,0x00},
    ['t'  - 0x20] = {0x02,0x02,0x1E,0x02,0x02,0x02,0x1C,0x00},
    /* Uppercase additions */
    ['J'  - 0x20] = {0x18,0x10,0x10,0x10,0x11,0x11,0x0E,0x00},
    ['K'  - 0x20] = {0x11,0x09,0x05,0x03,0x05,0x09,0x11,0x00},
    ['Q'  - 0x20] = {0x0E,0x11,0x11,0x11,0x15,0x09,0x1E,0x00},
    ['V'  - 0x20] = {0x11,0x11,0x11,0x0A,0x0A,0x04,0x00,0x00},
    ['X'  - 0x20] = {0x11,0x0A,0x04,0x04,0x0A,0x11,0x00,0x00},
    ['Y'  - 0x20] = {0x11,0x0A,0x04,0x04,0x04,0x04,0x00,0x00},
    ['Z'  - 0x20] = {0x1F,0x10,0x08,0x04,0x02,0x01,0x1F,0x00},
    /* Lowercase additions (x-height: pixels 1-4; ascenders: pixels 0-4) */
    ['a'  - 0x20] = {0x00,0x0E,0x10,0x1E,0x12,0x1E,0x00,0x00},
    ['b'  - 0x20] = {0x02,0x02,0x1E,0x12,0x12,0x1E,0x00,0x00},
    ['d'  - 0x20] = {0x10,0x10,0x1E,0x12,0x12,0x1E,0x00,0x00},
    ['f'  - 0x20] = {0x1C,0x02,0x0E,0x02,0x02,0x02,0x00,0x00},
    ['h'  - 0x20] = {0x02,0x02,0x1E,0x12,0x12,0x12,0x00,0x00},
    ['j'  - 0x20] = {0x08,0x00,0x08,0x08,0x08,0x0A,0x04,0x00},
    ['k'  - 0x20] = {0x02,0x02,0x12,0x0A,0x06,0x0A,0x12,0x00},
    ['l'  - 0x20] = {0x06,0x02,0x02,0x02,0x02,0x0E,0x00,0x00},
    ['m'  - 0x20] = {0x00,0x00,0x1B,0x15,0x15,0x15,0x00,0x00},
    ['p'  - 0x20] = {0x00,0x1E,0x12,0x12,0x1E,0x02,0x02,0x00},
    ['q'  - 0x20] = {0x00,0x1E,0x12,0x12,0x1E,0x10,0x10,0x00},
    ['r'  - 0x20] = {0x00,0x00,0x1E,0x02,0x02,0x02,0x00,0x00},
    ['s'  - 0x20] = {0x00,0x1E,0x02,0x0E,0x10,0x1E,0x00,0x00},
    ['u'  - 0x20] = {0x00,0x00,0x12,0x12,0x12,0x1E,0x00,0x00},
    ['v'  - 0x20] = {0x00,0x00,0x12,0x0A,0x0A,0x04,0x00,0x00},
    ['w'  - 0x20] = {0x00,0x00,0x11,0x15,0x0A,0x0A,0x00,0x00},
    ['x'  - 0x20] = {0x00,0x00,0x12,0x0C,0x0C,0x12,0x00,0x00},
    ['y'  - 0x20] = {0x00,0x11,0x0A,0x04,0x04,0x04,0x00,0x00},
    ['z'  - 0x20] = {0x00,0x00,0x1E,0x08,0x04,0x02,0x1E,0x00},
    /* Symbols */
    ['!'  - 0x20] = {0x04,0x04,0x04,0x04,0x04,0x00,0x04,0x00},
    ['#'  - 0x20] = {0x0A,0x0A,0x1F,0x0A,0x1F,0x0A,0x0A,0x00},
    ['$'  - 0x20] = {0x04,0x0E,0x05,0x0E,0x14,0x0E,0x04,0x00},
    ['&'  - 0x20] = {0x06,0x09,0x05,0x16,0x09,0x11,0x1E,0x00},
    ['('  - 0x20] = {0x08,0x04,0x02,0x02,0x02,0x04,0x08,0x00},
    [')'  - 0x20] = {0x02,0x04,0x08,0x08,0x08,0x04,0x02,0x00},
    ['*'  - 0x20] = {0x00,0x04,0x15,0x0E,0x15,0x04,0x00,0x00},
    [','  - 0x20] = {0x00,0x00,0x00,0x00,0x00,0x06,0x04,0x00},
    [':'  - 0x20] = {0x00,0x0C,0x0C,0x00,0x0C,0x0C,0x00,0x00},
    [';'  - 0x20] = {0x00,0x06,0x06,0x00,0x06,0x04,0x02,0x00},
    ['<'  - 0x20] = {0x08,0x04,0x02,0x01,0x02,0x04,0x08,0x00},
    ['='  - 0x20] = {0x00,0x00,0x1F,0x00,0x1F,0x00,0x00,0x00},
    ['>'  - 0x20] = {0x01,0x02,0x04,0x08,0x04,0x02,0x01,0x00},
    ['?'  - 0x20] = {0x0E,0x11,0x10,0x08,0x04,0x00,0x04,0x00},
    ['@'  - 0x20] = {0x0E,0x11,0x1D,0x15,0x1D,0x01,0x0E,0x00},
    ['['  - 0x20] = {0x0E,0x02,0x02,0x02,0x02,0x02,0x0E,0x00},
    [']'  - 0x20] = {0x0E,0x08,0x08,0x08,0x08,0x08,0x0E,0x00},
    ['^'  - 0x20] = {0x04,0x0A,0x11,0x00,0x00,0x00,0x00,0x00},
    ['_'  - 0x20] = {0x00,0x00,0x00,0x00,0x00,0x00,0x1F,0x00},
};

/* Draw text into fb at pixel (x0,y0) with given scale and color.
   Simple version: no background/outline, just pixels. */
static void splash_draw_text(uint16_t *fb, int x0, int y0, int scale,
                              const char *text, uint16_t color)
{
    const int cw  = 8 * scale;
    const int gap = scale;
    int cx = x0;
    for (int i = 0; text[i]; i++) {
        uint8_t c = (uint8_t)text[i];
        if (c >= 0x20 && c < 0x80) {
            for (int fr = 0; fr < 8; fr++) {
                uint8_t bits = s_font8x8[c - 0x20][fr];
                for (int fc = 0; fc < 8; fc++) {
                    if (!(bits & (1u << fc))) continue;
                    for (int sy = 0; sy < scale; sy++) {
                        int py = y0 + fr * scale + sy;
                        if (py < 0 || py >= LCD_H) continue;
                        for (int sx = 0; sx < scale; sx++) {
                            int px = cx + fc * scale + sx;
                            if (px >= 0 && px < LCD_W)
                                fb[py * LCD_W + px] = color;
                        }
                    }
                }
            }
        }
        cx += cw + gap;
    }
}

static void splash_flush(uint16_t *fb)
{
    const int STRIP_H = 40;
    for (int y = 0; y < LCD_H; y += STRIP_H) {
        int h = (y + STRIP_H <= LCD_H) ? STRIP_H : (LCD_H - y);
        esp_lcd_panel_draw_bitmap(s_panel, 0, y, LCD_W, y + h, fb + y * LCD_W);
    }
}

/* Draw a bordered button rectangle. */
static void splash_draw_button(uint16_t *fb, int bx, int by, int bw, int bh,
                                const char *label, uint16_t border, uint16_t fill)
{
    /* Fill */
    for (int row = by; row < by + bh; row++)
        for (int col = bx; col < bx + bw; col++)
            if (row >= 0 && row < LCD_H && col >= 0 && col < LCD_W)
                fb[row * LCD_W + col] = fill;
    /* 2 px border */
    for (int t = 0; t < 2; t++) {
        for (int col = bx - t; col <= bx + bw + t; col++) {
            if (col < 0 || col >= LCD_W) continue;
            int r0 = by - t, r1 = by + bh + t;
            if (r0 >= 0 && r0 < LCD_H) fb[r0 * LCD_W + col] = border;
            if (r1 >= 0 && r1 < LCD_H) fb[r1 * LCD_W + col] = border;
        }
        for (int row = by - t; row <= by + bh + t; row++) {
            if (row < 0 || row >= LCD_H) continue;
            int c0 = bx - t, c1 = bx + bw + t;
            if (c0 >= 0 && c0 < LCD_W) fb[row * LCD_W + c0] = border;
            if (c1 >= 0 && c1 < LCD_W) fb[row * LCD_W + c1] = border;
        }
    }
    /* Centered label */
    if (label) {
        const int scale = 2;
        const int cw = 8 * scale, gap = scale;
        int len = (int)strlen(label);
        int tw = len * cw + (len > 1 ? (len - 1) * gap : 0);
        int tx = bx + (bw - tw) / 2;
        int ty = by + (bh - 8 * scale) / 2;
        splash_draw_text(fb, tx, ty, scale, label, border);
    }
}

/* Render the "Connecting..." screen with the WiFi Setup button and push it to
   the LCD.  Pure render — no blocking, no touch polling.  The caller runs the
   poll loop so the WiFi connection can proceed in parallel. */
static void display_splash(void)
{
    uint16_t *fb = heap_caps_malloc((size_t)LCD_W * LCD_H * sizeof(uint16_t),
                                    MALLOC_CAP_SPIRAM);
    if (!fb) return;
    memset(fb, 0x00, (size_t)LCD_W * LCD_H * sizeof(uint16_t));

    const char *text  = "Connecting...";
    const int   scale = 2, cw = 8 * scale, gap = scale;
    int len    = (int)strlen(text);
    int text_w = len * cw + (len - 1) * gap;
    splash_draw_text(fb, (LCD_W - text_w) / 2, LCD_H / 2 - cw - 30,
                     scale, text, 0xFFFF);

    const uint16_t green = __builtin_bswap16(0x07E0);
    splash_draw_button(fb, (LCD_W - 160) / 2, LCD_H / 2 + 20, 160, 44,
                       "WiFi Setup", green, 0x0000);

    splash_flush(fb);
    heap_caps_free(fb);
}

/* ── TJpgDec callbacks ────────────────────────────────────────────────────── */

typedef struct {
    const uint8_t *data;
    size_t         len;
    size_t         pos;
    uint16_t      *fb;
    int            x_off;
    int            y_off;
    int            src_w;
    int            src_h;
    int            dst_w;
    int            dst_h;
} jpeg_ctx_t;

/* Precomputed source→destination coordinate boundaries, filled once per frame
   in decode_and_display and consumed by jpeg_outfunc. Sized for up to 640×480. */
static int s_xmap[642];
static int s_ymap[482];

static size_t jpeg_infunc(JDEC *jd, uint8_t *buf, size_t ndata)
{
    jpeg_ctx_t *ctx = (jpeg_ctx_t *)jd->device;
    size_t remain = ctx->len - ctx->pos;
    if (ndata > remain) ndata = remain;
    if (buf) memcpy(buf, ctx->data + ctx->pos, ndata);
    ctx->pos += ndata;
    return ndata;
}

static int jpeg_outfunc(JDEC *jd, void *bitmap, JRECT *rect)
{
    jpeg_ctx_t *ctx = (jpeg_ctx_t *)jd->device;
    uint16_t   *pix = (uint16_t *)bitmap;
    int blk_w = rect->right  - rect->left + 1;
    int blk_h = rect->bottom - rect->top  + 1;

    for (int row = 0; row < blk_h; row++) {
        int y0 = s_ymap[rect->top  + row];
        int y1 = s_ymap[rect->top  + row + 1];
        for (int col = 0; col < blk_w; col++) {
            uint16_t px = __builtin_bswap16(*pix++);
            int x0 = s_xmap[rect->left + col];
            int x1 = s_xmap[rect->left + col + 1];
            for (int dy = y0; dy < y1; dy++) {
                uint16_t *row_ptr = ctx->fb + dy * LCD_W;
                for (int dx = x0; dx < x1; dx++)
                    row_ptr[dx] = px;
            }
        }
    }
    return 1;
}

/* ── RTP extension header parser ──────────────────────────────────────────── */

static inline int32_t be32s(const uint8_t *p)
{
    return (int32_t)(((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16)
                   | ((uint32_t)p[2] <<  8) |  (uint32_t)p[3]);
}

static bool s_ext_logged = false;

static void parse_rtp_ext(const uint8_t *ext_hdr, int ext_words)
{
    /* ext_hdr: 2-byte profile | 2-byte length (words) | fields */
    const uint8_t *p   = ext_hdr + 4;
    const uint8_t *end = ext_hdr + 4 + ext_words * 4;

    while (p + 4 <= end) {
        uint16_t func_id     = ((uint16_t)p[0] << 8) | p[1];
        uint16_t field_words = ((uint16_t)p[2] << 8) | p[3];
        p += 4;
        int data_len = field_words * 4;
        if (p + data_len > end) break;

        if (!s_ext_logged) {
            int32_t w0 = field_words >= 1 ? be32s(p)     : 0;
            int32_t w1 = field_words >= 2 ? be32s(p + 4) : 0;
            int32_t w2 = field_words >= 3 ? be32s(p + 8) : 0;
            ESP_LOGI(TAG, "ext func=%u words=%u [0]=%ld [1]=%ld [2]=%ld",
                     func_id, field_words, (long)w0, (long)w1, (long)w2);
        }

        switch (func_id) {
        case 8:  /* shutspeedvalue: 3 words [min, max, cur], each packed as num(16)|denom(16) */
            if (field_words >= 3) {
                uint32_t cur        = (uint32_t)be32s(p + 8);
                s_osd.shutter_num   = (int32_t)(cur >> 16);
                s_osd.shutter_denom = (int32_t)(cur & 0xFFFF);
            }
            break;
        case 9:  /* focalvalue: max_fnum×10, min_fnum×10, cur_fnum×10 */
            if (field_words >= 3) {
                s_osd.fnum_x10 = be32s(p + 8);
            }
            break;
        case 12: /* isospeedvalue: cur_iso, auto_flag, ext_warning */
            if (field_words >= 1) {
                s_osd.iso = be32s(p);
            }
            break;
        case 10:  /* exprev: [max×10, min×10, cur×10] */
            if (field_words >= 3) {
                int32_t tenth = be32s(p + 8);
                for (int i = 0; i < s_exprev_cam_n; i++) {
                    if ((int32_t)lroundf(atof(s_exprev_cam[i].str) * 10.0f) == tenth) {
                        s_exprev_idx = i;
                        break;
                    }
                }
            }
            break;
        case 107:
            break; /* noise, not ring position */
        }
        p += data_len;
    }

    if (!s_ext_logged) s_ext_logged = true;
    s_osd.valid = true;
}

/* ── OSD overlay ──────────────────────────────────────────────────────────── */

/* Draw text into the framebuffer at (x0, y0) at given scale.
   inverted=true swaps colors (white bg, black text) to indicate selection. */
static void draw_osd_text(uint16_t *fb, int x0, int y0, int scale, const char *text, bool inverted, bool editable, uint16_t fg_color)
{
    const uint16_t bg    = inverted ? 0xFFFF : 0x0000;
    const uint16_t fg    = fg_color ? fg_color : (inverted ? 0x0000 : 0xFFFF);
    const uint16_t green = __builtin_bswap16(0x07E0);

    int len = 0;
    while (text[len]) len++;
    const int cw  = 8 * scale;
    const int ch  = 8 * scale;
    const int gap = scale;
    int text_w = len * cw + (len > 1 ? (len - 1) * gap : 0);

    /* Background fill — wide enough to cover the 2 px outline */
    for (int row = y0 - 5; row < y0 + ch + 3; row++) {
        if (row < 0 || row >= LCD_H) continue;
        for (int col = x0 - 3; col < x0 + text_w + 3; col++) {
            if (col >= 0 && col < LCD_W)
                fb[row * LCD_W + col] = bg;
        }
    }

    /* Green 2 px outline when editable and not currently selected */
    if (editable && !inverted) {
        for (int t = 0; t < 2; t++) {
            int bx0 = x0 - 2 - t, bx1 = x0 + text_w + 1 + t;
            int by0 = y0 - 4 - t, by1 = y0 + ch + 1 + t;
            for (int col = bx0; col <= bx1; col++) {
                if (col >= 0 && col < LCD_W) {
                    if (by0 >= 0 && by0 < LCD_H) fb[by0 * LCD_W + col] = green;
                    if (by1 >= 0 && by1 < LCD_H) fb[by1 * LCD_W + col] = green;
                }
            }
            for (int row = by0 + 1; row < by1; row++) {
                if (row >= 0 && row < LCD_H) {
                    if (bx0 >= 0 && bx0 < LCD_W) fb[row * LCD_W + bx0] = green;
                    if (bx1 >= 0 && bx1 < LCD_W) fb[row * LCD_W + bx1] = green;
                }
            }
        }
    }

    int cx = x0 + 3;
    for (int i = 0; i < len; i++) {
        uint8_t c = (uint8_t)text[i];
        if (c >= 0x20 && c < 0x80) {
            for (int fr = 0; fr < 8; fr++) {
                uint8_t bits = s_font8x8[c - 0x20][fr];
                for (int fc = 0; fc < 8; fc++) {
                    if (!(bits & (1u << fc))) continue;
                    for (int sy = 0; sy < scale; sy++) {
                        int py = y0 + fr * scale + sy;
                        if (py < 0 || py >= LCD_H) continue;
                        for (int sx = 0; sx < scale; sx++) {
                            int px = cx + fc * scale + sx;
                            if (px >= 0 && px < LCD_W)
                                fb[py * LCD_W + px] = fg;
                        }
                    }
                }
            }
        }
        cx += cw + gap;
    }
}

static void draw_osd_bottom(uint16_t *fb)
{
    if (!s_osd.valid) return;

    char shutter_str[10], fnum_str[8];

    /* Clamp to camera-realistic ranges before formatting */
    int sn = (s_osd.shutter_num   > 0 && s_osd.shutter_num   <    99) ? (int)s_osd.shutter_num   : 0;
    int sd = (s_osd.shutter_denom > 0 && s_osd.shutter_denom < 99999) ? (int)s_osd.shutter_denom : 0;
    int fn = (s_osd.fnum_x10      > 0 && s_osd.fnum_x10      < 999)  ? (int)s_osd.fnum_x10      : 0;

    /* Shutter speed */
    if (sd == 0 || sn == 0)
        snprintf(shutter_str, sizeof(shutter_str), "---");
    else if (sd == 1)
        snprintf(shutter_str, sizeof(shutter_str), "%d\"", sn);
    else if (sn == 1)
        snprintf(shutter_str, sizeof(shutter_str), "1/%d", sd);
    else
        snprintf(shutter_str, sizeof(shutter_str), "%d/%d", sn, sd);

    /* F-number */
    if (fn == 0)
        snprintf(fnum_str, sizeof(fnum_str), "F--");
    else
        snprintf(fnum_str, sizeof(fnum_str), "F%d.%d", fn / 10, fn % 10);

    char exprev_str[8];
    if (s_exprev_cam_n > 0)
        snprintf(exprev_str, sizeof(exprev_str), "%s", s_exprev_cam[s_exprev_idx].str);
    else
        snprintf(exprev_str, sizeof(exprev_str), "---");

    const int scale      = 2;
    const int cw         = 8 * scale;
    const int gap        = scale;
    const int y0         = 340;
    /* Three fields: shutter (103), exprev (206), aperture (309). */
    const int centers[3]    = {115, 206, 306};
    const char *strs[3]     = {shutter_str, exprev_str, fnum_str};
    const int field_ids[3]  = {0, 3, 1};

    /* Compute the circular display's x bounds at y0 so boxes stay visible. */
    {
        const int cc = LCD_W / 2, rc = LCD_W / 2 - 8;
        int dy  = y0 - LCD_H / 2;
        int dx2 = rc * rc - dy * dy;
        int dx  = (dx2 > 0) ? (int)sqrtf((float)dx2) : 0;
        int x_lo = cc - dx;   /* leftmost visible pixel at y0 */
        int x_hi = cc + dx;   /* rightmost visible pixel at y0 */

        for (int i = 0; i < 3; i++) {
            int fid    = field_ids[i];
            int len    = (int)strlen(strs[i]);
            int text_w = len * cw + (len > 1 ? (len - 1) * gap : 0);
            int x0     = centers[i] - text_w / 2;
            /* Clamp so the 3 px box padding stays within the circle. */
            if (x0 - 3 < x_lo)               x0 = x_lo + 3;
            if (x0 + text_w + 3 > x_hi)      x0 = x_hi - text_w - 3;
            draw_osd_text(fb, x0, y0, scale, strs[i],
                          (fid == s_selected_field), field_selectable(fid, s_shoot_mode), 0);
        }
    }
}


static void fill_outside_circle(uint16_t *fb, uint16_t color)
{
    const int cx = LCD_W / 2, cy = LCD_H / 2, r = LCD_W / 2 - 8;
    for (int y = 0; y < LCD_H; y++) {
        int dy = y - cy;
        int dx2 = r * r - dy * dy;
        int x_lo, x_hi;
        if (dx2 < 0) {
            x_lo = LCD_W; x_hi = 0;
        } else {
            int dx = (int)sqrtf((float)dx2);
            while (dx > 0 && dx * dx > dx2) dx--;
            while ((dx + 1) * (dx + 1) <= dx2) dx++;
            x_lo = cx - dx;
            x_hi = cx + dx + 1;
        }
        uint16_t *row = fb + (size_t)y * LCD_W;
        for (int x = 0; x < x_lo; x++) row[x] = color;
        for (int x = x_hi; x < LCD_W; x++) row[x] = color;
    }
}

/* ── Touch AF helpers ─────────────────────────────────────────────────────── */

/* Map a display-pixel tap to liveview coordinates (0–319 × 0–239).
   Returns false if the tap lands in a letterbox/border region outside the image. */
static bool tap_to_liveview(int tx, int ty, int *lv_x, int *lv_y)
{
    const int LV_W = 320, LV_H = 240;
    int dst_w, dst_h, x_off, y_off;
    if (s_display_mode == DISPLAY_FILL_WIDTH) {
        dst_w = LCD_W;
        dst_h = LV_H * LCD_W / LV_W;
        x_off = 0;
    } else {
        dst_w = LV_W;
        dst_h = LV_H;
        x_off = (LCD_W - LV_W) / 2;
    }
    y_off = (LCD_H - dst_h) / 2;

    int ix = tx - x_off;
    int iy = ty - y_off;
    if (ix < 0 || ix >= dst_w || iy < 0 || iy >= dst_h) return false;

    *lv_x = ix * LV_W / dst_w;
    *lv_y = iy * LV_H / dst_h;
    return true;
}

/* Inverse of tap_to_liveview: map a liveview rect to display pixel coords. */
static void liveview_to_display_rect(int lv_x, int lv_y, int lv_w, int lv_h,
                                     int *dx, int *dy, int *dw, int *dh)
{
    const int LV_W = 320, LV_H = 240;
    int dst_w, dst_h, x_off, y_off;
    if (s_display_mode == DISPLAY_FILL_WIDTH) {
        dst_w = LCD_W;
        dst_h = LV_H * LCD_W / LV_W;
        x_off = 0;
    } else {
        dst_w = LV_W;
        dst_h = LV_H;
        x_off = (LCD_W - LV_W) / 2;
    }
    y_off = (LCD_H - dst_h) / 2;

    *dx = x_off + lv_x * dst_w / LV_W;
    *dy = y_off + lv_y * dst_h / LV_H;
    *dw = lv_w  * dst_w / LV_W;
    *dh = lv_h  * dst_h / LV_H;
}

/* ── Decode + push one frame ──────────────────────────────────────────────── */

static void decode_and_display(const uint8_t *jpeg_data, int jpeg_len, uint16_t *fb)
{
    static uint8_t work[16384];

    jpeg_ctx_t ctx = {
        .data  = jpeg_data,
        .len   = (size_t)jpeg_len,
        .pos   = 0,
        .fb    = fb,
        .x_off = 0,
        .y_off = 0,
    };

    JDEC jd;
    if (jd_prepare(&jd, jpeg_infunc, work, sizeof(work), &ctx) != JDR_OK) {
        ESP_LOGE(TAG, "jd_prepare failed");
        return;
    }

    ctx.src_w = jd.width;
    ctx.src_h = jd.height;
    if (s_display_mode == DISPLAY_FILL_WIDTH) {
        ctx.dst_w = LCD_W;
        ctx.dst_h = (int)jd.height * LCD_W / (int)jd.width;
        ctx.x_off = 0;
    } else {
        ctx.dst_w = jd.width;
        ctx.dst_h = jd.height;
        ctx.x_off = (LCD_W - (int)jd.width) / 2;
    }
    ctx.y_off = (LCD_H - ctx.dst_h) / 2;

    /* Build source→destination coordinate tables once per frame (ceiling division
       maps each source boundary to the destination boundary it covers). Values are
       clamped to [0, LCD_W/H] so jpeg_outfunc needs no per-pixel bounds checks. */
    for (int i = 0; i <= ctx.src_w; i++) {
        int x = ctx.x_off + (i * ctx.dst_w + ctx.src_w - 1) / ctx.src_w;
        s_xmap[i] = (x < 0) ? 0 : (x > LCD_W) ? LCD_W : x;
    }
    for (int j = 0; j <= ctx.src_h; j++) {
        int y = ctx.y_off + (j * ctx.dst_h + ctx.src_h - 1) / ctx.src_h;
        s_ymap[j] = (y < 0) ? 0 : (y > LCD_H) ? LCD_H : y;
    }

    /* Clear letterbox rows if ring was drawn there and WiFi just came back */
    if (s_ring_on_fb) {
        memset(fb, 0, (size_t)ctx.y_off * LCD_W * sizeof(uint16_t));
        int bot = ctx.y_off + ctx.dst_h;
        if (bot < LCD_H)
            memset(fb + (size_t)bot * LCD_W, 0, (size_t)(LCD_H - bot) * LCD_W * sizeof(uint16_t));
        s_ring_on_fb = false;
    }

    if (jd_decomp(&jd, jpeg_outfunc, 0) != JDR_OK) {
        ESP_LOGE(TAG, "jd_decomp failed");
        return;
    }

    /* AF box overlay — green=succeeded, red=failed, absent when camera reports none */
    if (s_af_color == 1 || s_af_color == 2) {
        uint16_t af_clr = __builtin_bswap16(s_af_color == 1 ? 0x07E0 : 0xF800);
        int dx, dy, dw, dh;
        liveview_to_display_rect(s_af_lv_x, s_af_lv_y, s_af_lv_w, s_af_lv_h,
                                 &dx, &dy, &dw, &dh);
        for (int t = 0; t < 2; t++) {
            int x0 = dx - t, x1 = dx + dw + t;
            int y0 = dy - t, y1 = dy + dh + t;
            for (int col = x0; col <= x1; col++) {
                if (col >= 0 && col < LCD_W) {
                    if (y0 >= 0 && y0 < LCD_H) fb[y0 * LCD_W + col] = af_clr;
                    if (y1 >= 0 && y1 < LCD_H) fb[y1 * LCD_W + col] = af_clr;
                }
            }
            for (int row = y0 + 1; row < y1; row++) {
                if (row >= 0 && row < LCD_H) {
                    if (x0 >= 0 && x0 < LCD_W) fb[row * LCD_W + x0] = af_clr;
                    if (x1 >= 0 && x1 < LCD_W) fb[row * LCD_W + x1] = af_clr;
                }
            }
        }
    }

    /* OSD: shooting mode indicator — top centre, in the black letterbox above the image */
    {
        /* Width of the widest mode string at scale=3 — anchor for outline and battery */
        int max_mw = 0;
        for (int i = 0; i < NUM_MODES; i++) {
            int l = (int)strlen(s_mode_display[i]);
            int w = l * (8 * 3) + (l > 1 ? (l - 1) * 3 : 0);
            if (w > max_mw) max_mw = w;
        }

        /* Clear the full-width band first so no stale pixels from a wider previous mode name remain */
        for (int row = 7; row < 42; row++)
            memset(fb + row * LCD_W, 0, LCD_W * sizeof(uint16_t));
        const char *mtxt = s_mode_display[s_shoot_mode];
        int mlen = (int)strlen(mtxt);
        int mw   = mlen * (8 * 3) + (mlen > 1 ? (mlen - 1) * 3 : 0);
        draw_osd_text(fb, (LCD_W - mw) / 2 - 24, 12, 3, mtxt, false, false, 0);

        /* Battery level — right-justified to a fixed right edge so it never moves */
        if (s_battery_str[0] && s_battery_str[0] != '-') {
            int blen       = (int)strlen(s_battery_str);
            int bw         = blen * (8 * 2) + (blen > 1 ? (blen - 1) * 2 : 0);
            int batt_right = (LCD_W + max_mw) / 2 + 60;

            /* Pick base color by level */
            uint16_t base_clr;
            if (strcmp(s_battery_str, "FULL") == 0 || strcmp(s_battery_str, "CHRG") == 0)
                base_clr = __builtin_bswap16(0x07E0);        /* green  */
            else if (strcmp(s_battery_str, "LOW") == 0)
                base_clr = __builtin_bswap16(0xFD20);        /* orange */
            else
                base_clr = __builtin_bswap16(0xF800);        /* red (WARN, EMPTY, UNK) */

            uint16_t batt_clr = base_clr;
            bool blink_phase = (esp_timer_get_time() / 500000ULL) & 1;
            if (s_charging) {
                /* Charging: blink between base color and green at 1 Hz */
                batt_clr = blink_phase ? __builtin_bswap16(0x07E0) : base_clr;
            } else if (strcmp(s_battery_str, "WARN") == 0) {
                /* Warning: blink red on/off at 1 Hz */
                batt_clr = blink_phase ? base_clr : 0x0000;
            }

            draw_osd_text(fb, batt_right - bw, 20, 2, s_battery_str, false, false, batt_clr);
        }

        /* Fixed-size 2px green outline around mode tap zone — sized to the widest mode ("iA")
           so it never shifts as the mode changes. Drawn after text so it is never erased. */
        {
            const uint16_t green = __builtin_bswap16(0x07E0);
            int ox0 = (LCD_W - max_mw) / 2 - 24;
            for (int t = 0; t < 2; t++) {
                int bx0 = ox0 - 2 - t, bx1 = ox0 + max_mw + 1 + t;
                int by0 = 12  - 4 - t, by1 = 12  + 24    + 1 + t;
                for (int col = bx0; col <= bx1; col++) {
                    if (col >= 0 && col < LCD_W) {
                        if (by0 >= 0 && by0 < LCD_H) fb[by0 * LCD_W + col] = green;
                        if (by1 >= 0 && by1 < LCD_H) fb[by1 * LCD_W + col] = green;
                    }
                }
                for (int row = by0 + 1; row < by1; row++) {
                    if (row >= 0 && row < LCD_H) {
                        if (bx0 >= 0 && bx0 < LCD_W) fb[row * LCD_W + bx0] = green;
                        if (bx1 >= 0 && bx1 < LCD_W) fb[row * LCD_W + bx1] = green;
                    }
                }
            }
        }
    }
    /* OSD: ISO — top-left column (same x-centre as shutter, 103) */
    if (s_osd.valid) {
        char iso_str[8];
        int iso = (s_osd.iso > 0 && s_osd.iso < 99999) ? (int)s_osd.iso : 0;
        if (iso == 0) snprintf(iso_str, sizeof(iso_str), "---");
        else          snprintf(iso_str, sizeof(iso_str), "%d", iso);
        int ilen = (int)strlen(iso_str);
        int iw   = ilen * (8 * 2) + (ilen > 1 ? (ilen - 1) * 2 : 0);
        draw_osd_text(fb, 103 - iw / 2, 58, 2, iso_str, (s_selected_field == 2),
                      field_selectable(2, s_shoot_mode), 0);
    }
    /* OSD: white balance — top-right column (same x-centre as aperture, 309) */
    {
        const char *wbtxt = (s_wb_cam_n > 0) ? s_wb_cam[s_wb_idx].label : "---";
        int wblen = (int)strlen(wbtxt);
        int wbw   = wblen * (8 * 2) + (wblen > 1 ? (wblen - 1) * 2 : 0);
        draw_osd_text(fb, 309 - wbw / 2, 58, 2, wbtxt, false, (s_shoot_mode != 4), 0);
    }
    /* OSD: shutter / aperture — spread at bottom */
    draw_osd_bottom(fb);

    /* WiFi lost indicator — fill outside circle red */
    if (!s_wifi_connected) {
        fill_outside_circle(fb, __builtin_bswap16(0xF800));
        s_ring_on_fb = true;
    }

    const int STRIP_H = 80;
    for (int y = 0; y < LCD_H; y += STRIP_H) {
        int h = (y + STRIP_H <= LCD_H) ? STRIP_H : (LCD_H - y);
        esp_lcd_panel_draw_bitmap(s_panel, 0, y, LCD_W, y + h, fb + y * LCD_W);
    }
}

/* ── Postview ─────────────────────────────────────────────────────────────── */

/* Fetch the thumbnail of the last image on the camera and hold it on screen
   for one second.  Called from the liveview task; jpeg_buf (PSRAM) is reused
   as a scratch area for both the image-list XML and the thumbnail JPEG. */
/* ── Liveview loop ────────────────────────────────────────────────────────── */

static void liveview_loop(uint8_t *jpeg_buf, uint16_t *fb)
{
    static uint8_t pkt[PKT_BUF_SIZE];

restart:;
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) { ESP_LOGE(TAG, "socket: %d", errno); vTaskDelay(pdMS_TO_TICKS(1000)); goto restart; }

    struct sockaddr_in local = {
        .sin_family      = AF_INET,
        .sin_port        = htons(LV_PORT),
        .sin_addr.s_addr = INADDR_ANY,
    };
    if (bind(sock, (struct sockaddr *)&local, sizeof(local)) < 0) {
        ESP_LOGE(TAG, "bind: %d", errno); close(sock); vTaskDelay(pdMS_TO_TICKS(1000)); goto restart;
    }

    /* Large UDP receive buffer so packets queue while we're decoding */
    int rcvbuf = 128 * 1024;
    setsockopt(sock, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));

    struct timeval tv = { .tv_sec = 2, .tv_usec = 0 };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    /* Issue the three startup commands on one persistent TCP connection.
       A fresh TCP handshake per call plus the 150 ms inter-call throttle would
       add ~300 ms; keep-alive collapses that to a single handshake + no gaps. */
    {
        char lv_url[64];
        snprintf(lv_url, sizeof(lv_url),
                 "/exec_takemisc.cgi?com=startliveview&port=%d", LV_PORT);

        esp_http_client_config_t kcfg = {
            .url               = "http://" CAM_IP "/",
            .event_handler     = http_evt,
            .timeout_ms        = 2000,
            .keep_alive_enable = true,
        };
        xSemaphoreTake(s_http_mutex, portMAX_DELAY);
        http_throttle();
        esp_http_client_handle_t kc = esp_http_client_init(&kcfg);
        esp_http_client_set_header(kc, "User-Agent", "OlympusCameraKit");
        esp_http_client_set_header(kc, "X-Protocol",  "OlympusCameraKit");
        cam_get_on(kc, "/switch_cameramode.cgi?mode=rec");
        cam_get_on(kc, "/exec_takemisc.cgi?com=changelvqty&lvqty=0320x0240");
        cam_get_on(kc, lv_url);
        esp_http_client_cleanup(kc);
        s_http_last_us = esp_timer_get_time();
        xSemaphoreGive(s_http_mutex);
    }

    /* Start pushevent task here, after switch_cameramode + startliveview, so the
       camera's event channel is not torn down by a subsequent mode switch. */
    static bool s_pushevent_started = false;
    if (!s_pushevent_started) {
        s_pushevent_started = true;
        s_prop_queue = xQueueCreate(8, PROP_NAME_MAX);
        xTaskCreate(init_props_task,   "init_props", 4096, NULL, 3, NULL);
        xTaskCreate(prop_refresh_task, "prop_ref",   4096, NULL, 3, NULL);
        xTaskCreate(pushevent_task,    "pushevent",  4096, NULL, 4, NULL);
        xTaskCreate(battery_task,      "battery",    4096, NULL, 3, NULL);
        s_af_queue = xQueueCreate(1, sizeof(af_req_t));
        xTaskCreate(af_task, "touch_af", 4096, NULL, 4, NULL);
    }

    uint32_t cur_frame   = 0;
    int      jpeg_off    = 0;
    bool     started     = false;
    int      timeouts    = 0;
    uint32_t frames      = 0;
    uint32_t fps_frames  = 0;
    uint64_t fps_last_us = esp_timer_get_time();

    ESP_LOGI(TAG, "liveview started");

    while (true) {
        int n = recv(sock, pkt, sizeof(pkt), 0);

        if (n < 0) {
            if (++timeouts >= 3) {
                ESP_LOGW(TAG, "stream stalled — restarting");
                if (!s_ring_on_fb) {
                    fill_outside_circle(fb, __builtin_bswap16(0xF800));
                    s_ring_on_fb = true;
                    const int SH = 40;
                    for (int sy = 0; sy < LCD_H; sy += SH) {
                        int sh = (sy + SH <= LCD_H) ? SH : (LCD_H - sy);
                        esp_lcd_panel_draw_bitmap(s_panel, 0, sy, LCD_W, sy + sh, fb + sy * LCD_W);
                    }
                }
                cam_get("/exec_takemisc.cgi?com=stopliveview");
                close(sock);
                vTaskDelay(pdMS_TO_TICKS(500));
                goto restart;
            }
            continue;
        }
        /* Liveview recovered after a brief stall — the camera pauses the stream
           during capture, so this almost always means a picture was taken.
           Guard with frames > 10 so we don't trigger on the initial startup gap,
           and skip if another postview fetch is already queued. */
        timeouts = 0;
        if (n < 12) continue;

        bool     x     = (pkt[0] >> 4) & 1;
        bool     m     = (pkt[1] >> 7) & 1;
        uint32_t frame = ((uint32_t)pkt[4] << 24) | ((uint32_t)pkt[5] << 16)
                       | ((uint32_t)pkt[6] <<  8) |  (uint32_t)pkt[7];

        int hdr_end = 12;
        if (x && n > 16) {
            uint16_t ext_words = ((uint16_t)pkt[14] << 8) | pkt[15];
            hdr_end += 4 + ext_words * 4;
            parse_rtp_ext(pkt + 12, ext_words);
        }
        if (hdr_end >= n) continue;

        const uint8_t *payload     = pkt + hdr_end;
        int            payload_len = n - hdr_end;

        /* First packet of a new frame */
        if (x && (!started || frame != cur_frame)) {
            cur_frame = frame;
            jpeg_off  = 0;
            started   = true;
        }

        if (!started || frame != cur_frame) continue;

        if (jpeg_off + payload_len <= JPEG_BUF_SIZE) {
            memcpy(jpeg_buf + jpeg_off, payload, payload_len);
            jpeg_off += payload_len;
        } else {
            ESP_LOGW(TAG, "frame too large (%d), skipping", jpeg_off + payload_len);
            started = false;
            continue;
        }

        if (m) {
            if (s_tap_pending) {
                s_tap_pending = false;
                uint16_t tx = s_tap_x, ty = s_tap_y;
                ESP_LOGI(TAG, "tap x=%u y=%u", tx, ty);

                if (ty < 50) {
                    /* Mode indicator — top black bar (text y=12–36) */
                    s_shoot_mode = (s_shoot_mode + 1) % NUM_MODES;
                    ESP_LOGI(TAG, "shoot mode → %s", s_mode_display[s_shoot_mode]);
                    cam_set_prop_async("TAKEMODE", s_mode_api[s_shoot_mode]);
                    /* Do NOT call build_prop_lists() here: the camera's HTTP server
                       goes offline for several seconds after a mode change, and
                       the value lists loaded at startup remain valid across modes. */
                    if (s_selected_field >= 0 &&
                        !field_selectable(s_selected_field, s_shoot_mode))
                        s_selected_field = -1;
                } else if (ty < 90 && tx < LCD_W / 2) {
                    /* ISO — top-left column (text y=58–74; zone 50–89, +one row below text) */
                    if (field_selectable(2, s_shoot_mode)) {
                        s_selected_field = (s_selected_field == 2) ? -1 : 2;
                        ESP_LOGI(TAG, "selected field %d", s_selected_field);
                    }
                } else if (ty < 90) {
                    /* WB — top-right column (text y=58–74, +one row below) */
                    if (s_wb_cam_n > 0) {
                        s_wb_idx = (s_wb_idx + 1) % s_wb_cam_n;
                        ESP_LOGI(TAG, "WB → %s (%s)", s_wb_cam[s_wb_idx].label,
                                 s_wb_cam[s_wb_idx].api);
                        cam_set_prop_async("WB", s_wb_cam[s_wb_idx].api);
                    }
                } else if (ty >= 324) {
                    /* OSD strip (text y=340–356, +one row above): shutter/exprev/aperture */
                    int field;
                    if (tx < LCD_W / 3)         field = 0; /* shutter */
                    else if (tx < 2 * LCD_W / 3) field = 3; /* exprev */
                    else                          field = 1; /* aperture */
                    if (field_selectable(field, s_shoot_mode)) {
                        s_selected_field = (s_selected_field == field) ? -1 : field;
                        ESP_LOGI(TAG, "selected field %d", s_selected_field);
                    }
                } else if (s_selected_field >= 0) {
                    /* Main image with a field selected: left half decreases, right increases */
                    int delta = (tx >= (LCD_W / 2)) ? +1 : -1;
                    ESP_LOGI(TAG, "adjust field %d delta %d", s_selected_field, delta);
                    adjust_field(s_selected_field, delta);
                } else {
                    /* Tap on main image: queue touch AF — the af_task handles
                       the HTTP calls so the liveview loop keeps running. */
                    int lv_x, lv_y;
                    if (tap_to_liveview(tx, ty, &lv_x, &lv_y) && s_af_queue) {
                        s_af_color = 0;
                        af_req_t req = { lv_x, lv_y };
                        xQueueOverwrite(s_af_queue, &req);
                    }
                }
            }
            decode_and_display(jpeg_buf, jpeg_off, fb);
            frames++;
            fps_frames++;

            /* Log FPS every 5 s */
            {
                uint64_t now = esp_timer_get_time();
                uint64_t elapsed = now - fps_last_us;
                if (elapsed >= 5000000ULL) {
                    float fps = fps_frames * 1000000.0f / elapsed;
                    ESP_LOGI(TAG, "%.1f fps", fps);
                    fps_frames  = 0;
                    fps_last_us = now;
                }
            }

            started  = false;
            jpeg_off = 0;

            /* Drain any backlogged frames to keep latency low. Without
               this, RECVMBOX=64 lets seconds of frames queue up. The cost
               is ~1-2 fps when decode time ≈ frame period, but a viewfinder
               with no lag beats one with 12fps and a second of delay. */
            {
                int fl = fcntl(sock, F_GETFL, 0);
                fcntl(sock, F_SETFL, fl | O_NONBLOCK);
                while (recv(sock, pkt, sizeof(pkt), 0) > 0) {}
                fcntl(sock, F_SETFL, fl);
            }

        }
    }

    /* unreachable, but tidy up if ever reached */
    cam_get("/exec_takemisc.cgi?com=stopliveview");
    close(sock);
}

/* ── WiFi Setup UI ────────────────────────────────────────────────────────── */

/* Text width in pixels for given string, scale, and per-char gap. */
static int ui_text_w(const char *s, int scale)
{
    int len = (int)strlen(s);
    return len * 8 * scale + (len > 1 ? (len - 1) * scale : 0);
}

/* Fill a rectangle in PSRAM fb. */
static void ui_fill(uint16_t *fb, int x, int y, int w, int h, uint16_t color)
{
    for (int row = y; row < y + h; row++) {
        if (row < 0 || row >= LCD_H) continue;
        for (int col = x; col < x + w; col++)
            if (col >= 0 && col < LCD_W)
                fb[row * LCD_W + col] = color;
    }
}

static void ui_flush(uint16_t *fb)
{
    const int SH = 40;
    for (int y = 0; y < LCD_H; y += SH) {
        int h = (y + SH <= LCD_H) ? SH : (LCD_H - y);
        esp_lcd_panel_draw_bitmap(s_panel, 0, y, LCD_W, y + h, fb + y * LCD_W);
    }
}

/* Draw text centered horizontally in a box (box_x, box_w) at row ty. */
static void ui_text_center(uint16_t *fb, int box_x, int box_w, int ty,
                            int scale, const char *s, uint16_t color)
{
    int tw = ui_text_w(s, scale);
    splash_draw_text(fb, box_x + (box_w - tw) / 2, ty, scale, s, color);
}

/* Draw a bordered button and return true if the last tap fell within it. */
static void ui_button(uint16_t *fb, int x, int y, int w, int h,
                       const char *label, int lscale, uint16_t border, uint16_t fill)
{
    ui_fill(fb, x, y, w, h, fill);
    /* 2 px border */
    for (int t = 0; t < 2; t++) {
        for (int col = x - t; col <= x + w + t; col++) {
            if (col < 0 || col >= LCD_W) continue;
            if (y - t >= 0)       fb[(y - t)       * LCD_W + col] = border;
            if (y + h + t < LCD_H) fb[(y + h + t)  * LCD_W + col] = border;
        }
        for (int row = y - t; row <= y + h + t; row++) {
            if (row < 0 || row >= LCD_H) continue;
            if (x - t >= 0)       fb[row * LCD_W + (x - t)]       = border;
            if (x + w + t < LCD_W) fb[row * LCD_W + (x + w + t)]  = border;
        }
    }
    if (label) {
        int tw = ui_text_w(label, lscale);
        int tx = x + (w - tw) / 2;
        int ty2 = y + (h - 8 * lscale) / 2;
        splash_draw_text(fb, tx, ty2, lscale, label, border);
    }
}

static bool ui_hit(int tx, int ty, int x, int y, int w, int h)
{
    return tx >= x && tx < x + w && ty >= y && ty < y + h;
}

/* Wait for next tap, clearing s_tap_pending before returning. */
static void ui_wait_tap(uint16_t *out_x, uint16_t *out_y)
{
    s_tap_pending = false;
    while (!s_tap_pending)
        vTaskDelay(pdMS_TO_TICKS(20));
    s_tap_pending = false;
    if (out_x) *out_x = s_tap_x;
    if (out_y) *out_y = s_tap_y;
}

/* ── AP sort ──────────────────────────────────────────────────────────────── */

#include "esp_wifi.h"   /* wifi_ap_record_t already included transitively */

static int ap_cmp(const void *a, const void *b)
{
    const wifi_ap_record_t *ra = (const wifi_ap_record_t *)a;
    const wifi_ap_record_t *rb = (const wifi_ap_record_t *)b;
    bool a_air = (strncmp((char *)ra->ssid, "AIR", 3) == 0);
    bool b_air = (strncmp((char *)rb->ssid, "AIR", 3) == 0);
    if (a_air && !b_air) return -1;
    if (!a_air &&  b_air) return  1;
    return (int)rb->rssi - (int)ra->rssi;   /* stronger first */
}

/* ── Password keyboard screen ─────────────────────────────────────────────── */
/* Returns when user taps OK (saves creds + restarts) or Back.                 */

#define KB_KEY_W  37
#define KB_KEY_H  42
#define KB_GAP     3
/* Row Y positions for 4 keyboard rows */
#define KB_R0_Y  108
#define KB_R1_Y  153
#define KB_R2_Y  198
#define KB_R3_Y  248

/* Row x-starts for 10/9/8-key rows (keys are KB_KEY_W wide, KB_GAP apart) */
#define KB_R0_X    8   /* 10 keys: (412 - 10*37 - 9*3)/2 ≈ 8 */
#define KB_R1_X   27   /* 9  keys: center */
#define KB_R2_X   47   /* 8  keys: center */

static const char s_let_r0[] = "QWERTYUIOP";
static const char s_let_r1[] = "ASDFGHJKL";
static const char s_let_r2[] = "ZXCVBNM";
static const char s_num_r0[] = "1234567890";
static const char s_num_r1[] = "!@#$&*_-.";
static const char s_num_r2[] = "+=(),:?";

static void kb_draw(uint16_t *fb, const char *ssid, const char *pass,
                    bool num_mode, bool shifted)
{
    const uint16_t white  = 0xFFFF;
    const uint16_t gray   = __builtin_bswap16(0x39E7);  /* dark gray */
    const uint16_t green  = __builtin_bswap16(0x07E0);
    const uint16_t yellow = __builtin_bswap16(0xFFE0);

    memset(fb, 0x00, (size_t)LCD_W * LCD_H * sizeof(uint16_t));

    /* SSID line */
    char ssid_disp[32];
    snprintf(ssid_disp, sizeof(ssid_disp), "%.20s", ssid);
    splash_draw_text(fb, 8, 10, 2, "WiFi:", white);
    splash_draw_text(fb, 8 + ui_text_w("WiFi:", 2) + 4, 10, 2, ssid_disp, green);

    /* Password line */
    char pass_disp[48];
    int plen = (int)strlen(pass);
    /* show last ≤20 chars so it scrolls left as user types */
    const char *pshow = (plen > 20) ? pass + plen - 20 : pass;
    snprintf(pass_disp, sizeof(pass_disp), "Key:%s_", pshow);
    splash_draw_text(fb, 8, 42, 2, pass_disp, yellow);

    /* Separator */
    ui_fill(fb, 0, 72, LCD_W, 2, gray);

    /* Keyboard rows */
    const char *r0 = num_mode ? s_num_r0 : s_let_r0;
    const char *r1 = num_mode ? s_num_r1 : s_let_r1;
    const char *r2 = num_mode ? s_num_r2 : s_let_r2;

    struct { int x; const char *row; int n; int ky; } rows[] = {
        { KB_R0_X, r0, (int)strlen(r0), KB_R0_Y },
        { KB_R1_X, r1, (int)strlen(r1), KB_R1_Y },
        { KB_R2_X, r2, (int)strlen(r2), KB_R2_Y },
    };
    for (int ri = 0; ri < 3; ri++) {
        for (int ki = 0; ki < rows[ri].n; ki++) {
            int kx = rows[ri].x + ki * (KB_KEY_W + KB_GAP);
            char lbl[3] = { rows[ri].row[ki], 0, 0 };
            /* letter mode: show uppercase label, output shifts on shifted flag */
            if (!num_mode && lbl[0] >= 'A' && lbl[0] <= 'Z' && !shifted)
                lbl[0] = lbl[0] - 'A' + 'a';  /* show lowercase label when not shifted */
            ui_button(fb, kx, rows[ri].ky, KB_KEY_W, KB_KEY_H,
                      lbl, 2, white, gray);
        }
        /* Backspace at end of row 2 */
        if (ri == 2) {
            int kx = rows[ri].x + rows[ri].n * (KB_KEY_W + KB_GAP);
            ui_button(fb, kx, rows[ri].ky, KB_KEY_W, KB_KEY_H, "<", 2, white, gray);
        }
    }

    /* Special row: [123/ABC]  [SHF]  [SPACE]  [OK] */
    const uint16_t shift_clr = shifted ? yellow : white;
    ui_button(fb,   8, KB_R3_Y, 68, 50, num_mode ? "ABC" : "123", 1, white, gray);
    ui_button(fb,  82, KB_R3_Y, 68, 50, "SHF", 1, shift_clr, gray);
    ui_button(fb, 156, KB_R3_Y, 130, 50, " ", 2, white, gray);   /* space bar */
    ui_button(fb, 292, KB_R3_Y, 112, 50, "OK", 2, green, gray);
}

static void wifi_password_screen(uint16_t *fb, const char *ssid)
{
    char pass[65] = {0};
    int  plen     = 0;
    bool num_mode = false;
    bool shifted  = false;

    while (true) {
        kb_draw(fb, ssid, pass, num_mode, shifted);
        ui_flush(fb);

        uint16_t tx, ty;
        ui_wait_tap(&tx, &ty);

        /* Special row */
        if (ui_hit(tx, ty, 8, KB_R3_Y, 68, 50)) {
            num_mode = !num_mode;
            continue;
        }
        if (ui_hit(tx, ty, 82, KB_R3_Y, 68, 50)) {
            shifted = !shifted;
            continue;
        }
        if (ui_hit(tx, ty, 156, KB_R3_Y, 130, 50)) {
            if (plen < 64) { pass[plen++] = ' '; pass[plen] = '\0'; }
            continue;
        }
        if (ui_hit(tx, ty, 292, KB_R3_Y, 112, 50)) {
            /* OK — save and restart */
            wifi_creds_save(ssid, pass);
            esp_restart();
        }

        /* Keyboard rows */
        struct { int x; const char *row; int n; int ky; } rows[] = {
            { KB_R0_X, num_mode ? s_num_r0 : s_let_r0,
              (int)strlen(num_mode ? s_num_r0 : s_let_r0), KB_R0_Y },
            { KB_R1_X, num_mode ? s_num_r1 : s_let_r1,
              (int)strlen(num_mode ? s_num_r1 : s_let_r1), KB_R1_Y },
            { KB_R2_X, num_mode ? s_num_r2 : s_let_r2,
              (int)strlen(num_mode ? s_num_r2 : s_let_r2), KB_R2_Y },
        };
        bool handled = false;
        for (int ri = 0; ri < 3 && !handled; ri++) {
            if (!ui_hit(tx, ty, rows[ri].x, rows[ri].ky,
                        rows[ri].n * (KB_KEY_W + KB_GAP) + KB_KEY_W, KB_KEY_H))
                continue;
            /* Backspace at the end of row 2 */
            int bsp_x = rows[ri].x + rows[ri].n * (KB_KEY_W + KB_GAP);
            if (ri == 2 && ui_hit(tx, ty, bsp_x, rows[ri].ky, KB_KEY_W, KB_KEY_H)) {
                if (plen > 0) pass[--plen] = '\0';
                handled = true;
                break;
            }
            int ki = (tx - rows[ri].x) / (KB_KEY_W + KB_GAP);
            if (ki >= 0 && ki < rows[ri].n) {
                char ch = rows[ri].row[ki];
                /* Letter mode: apply shift */
                if (!num_mode && ch >= 'A' && ch <= 'Z') {
                    if (!shifted) ch = ch - 'A' + 'a';
                }
                if (plen < 64) { pass[plen++] = ch; pass[plen] = '\0'; }
                handled = true;
            }
        }
    }
}

/* ── AP scan & selection screen ───────────────────────────────────────────── */

#define AP_MAX       16
#define AP_ROW_H     48
#define AP_LIST_Y    65
#define AP_LIST_ROWS  6
#define AP_RESCAN_US 4000000LL   /* rescan interval: 4 s */

static volatile bool s_scan_done = false;

static void scan_done_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data)
{
    s_scan_done = true;
}

static void ap_list_draw(uint16_t *fb, wifi_ap_record_t *aps, int show_n,
                          bool scanning)
{
    const uint16_t white = 0xFFFF;
    const uint16_t green = __builtin_bswap16(0x07E0);
    const uint16_t gray  = __builtin_bswap16(0x39E7);
    const uint16_t dim   = __builtin_bswap16(0x7BEF);   /* mid-gray for hint */

    memset(fb, 0x00, (size_t)LCD_W * LCD_H * sizeof(uint16_t));

    /* Header + optional scanning badge */
    ui_text_center(fb, 0, LCD_W, 14, 2, "Select WiFi", white);
    if (scanning)
        splash_draw_text(fb, LCD_W - ui_text_w("...", 2) - 6, 14, 2, "...", dim);

    if (show_n == 0) {
        ui_text_center(fb, 0, LCD_W, LCD_H / 2 - 8, 2,
                       scanning ? "Scanning..." : "No networks", white);
    } else {
        for (int i = 0; i < show_n; i++) {
            int ry = AP_LIST_Y + i * AP_ROW_H;
            bool is_air = (strncmp((char *)aps[i].ssid, "AIR", 3) == 0);
            ui_fill(fb, 6, ry, 400, AP_ROW_H - 4, is_air ? gray : 0x0000);
            ui_fill(fb, 6,       ry,              400, 2, white);
            ui_fill(fb, 6,       ry + AP_ROW_H-6, 400, 2, white);
            ui_fill(fb, 6,       ry,              2, AP_ROW_H-4, white);
            ui_fill(fb, 6+400-2, ry,              2, AP_ROW_H-4, white);

            char ssid_buf[20];
            snprintf(ssid_buf, sizeof(ssid_buf), "%.17s", (char *)aps[i].ssid);
            splash_draw_text(fb, 14, ry + 8, 2, ssid_buf, is_air ? green : white);

            int rssi = aps[i].rssi;
            int bars = (rssi > -55) ? 3 : (rssi > -70) ? 2 : 1;
            for (int b = 0; b < 3; b++) {
                uint16_t bc = (b < bars) ? green : gray;
                int bh  = 6 + b * 5;
                int bby = ry + AP_ROW_H - 6 - bh;
                ui_fill(fb, 380 + b * 8, bby, 6, bh, bc);
            }
        }
    }

    ui_button(fb, (LCD_W - 140) / 2, 363, 140, 40, "Cancel", 2, white, 0x0000);
    ui_flush(fb);
}

static void wifi_scan_screen(uint16_t *fb, char *out_ssid)
{
    static wifi_ap_record_t aps[AP_MAX];
    int ap_n  = 0, show_n = 0;
    bool scanning  = true;   /* true while a scan is in flight */
    bool has_list  = false;  /* true once we have at least one result set */

    esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_SCAN_DONE,
                                scan_done_handler, NULL);

    /* Kick off first non-blocking scan */
    s_scan_done = false;
    wifi_scan_config_t scfg = { 0 };
    esp_wifi_scan_start(&scfg, false);
    int64_t last_scan_start_us = esp_timer_get_time();

    /* Initial "Scanning..." frame */
    ap_list_draw(fb, aps, 0, true);

    while (true) {
        /* ── Collect results when scan completes ───────────────────────── */
        if (s_scan_done) {
            s_scan_done = false;
            uint16_t n  = AP_MAX;
            esp_wifi_scan_get_ap_records(&n, aps);
            ap_n   = (int)n;
            show_n = ap_n > AP_LIST_ROWS ? AP_LIST_ROWS : ap_n;
            qsort(aps, ap_n, sizeof(wifi_ap_record_t), ap_cmp);
            scanning  = false;
            has_list  = true;
            ap_list_draw(fb, aps, show_n, false);
        }

        /* ── Kick off a rescan every AP_RESCAN_US ──────────────────────── */
        if (!scanning &&
            (esp_timer_get_time() - last_scan_start_us) > AP_RESCAN_US) {
            s_scan_done = false;
            esp_wifi_scan_start(&scfg, false);
            last_scan_start_us = esp_timer_get_time();
            scanning = true;
            /* Keep the existing list visible; just show the "..." badge */
            ap_list_draw(fb, aps, show_n, true);
        }

        /* ── Handle taps ───────────────────────────────────────────────── */
        if (s_tap_pending) {
            s_tap_pending = false;
            uint16_t tx = s_tap_x, ty = s_tap_y;

            if (ui_hit(tx, ty, (LCD_W - 140) / 2, 363, 140, 40)) {
                /* Cancel — stop any running scan before returning */
                esp_wifi_scan_stop();
                esp_event_handler_unregister(WIFI_EVENT, WIFI_EVENT_SCAN_DONE,
                                             scan_done_handler);
                out_ssid[0] = '\0';
                return;
            }

            if (has_list) {
                for (int i = 0; i < show_n; i++) {
                    int ry = AP_LIST_Y + i * AP_ROW_H;
                    if (ui_hit(tx, ty, 6, ry, 400, AP_ROW_H - 4)) {
                        esp_wifi_scan_stop();
                        esp_event_handler_unregister(WIFI_EVENT,
                                                     WIFI_EVENT_SCAN_DONE,
                                                     scan_done_handler);
                        strncpy(out_ssid, (char *)aps[i].ssid, 32);
                        out_ssid[32] = '\0';
                        return;
                    }
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/* Entry point for the full setup flow. Called after wifi_driver_init(). */
static void wifi_setup_flow(void)
{
    uint16_t *fb = heap_caps_malloc((size_t)LCD_W * LCD_H * sizeof(uint16_t),
                                    MALLOC_CAP_SPIRAM);
    if (!fb) return;

    char selected_ssid[33] = {0};
    wifi_scan_screen(fb, selected_ssid);

    if (selected_ssid[0] != '\0') {
        wifi_password_screen(fb, selected_ssid);
        /* wifi_password_screen restarts on OK; only returns on Back */
    }

    heap_caps_free(fb);
}

/* ── Entry point ──────────────────────────────────────────────────────────── */

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());

    s_http_mutex = xSemaphoreCreateMutex();

    display_init();
    touch_init();
    xTaskCreate(touch_task, "touch", 2048, NULL, 5, NULL);

    /* ── Setup mode: triggered by a flag written to NVS when the user tapped
       "WiFi Setup" on the connecting screen during a previous boot.  Checked
       first so we never start a connection we immediately have to tear down. */
    {
        nvs_handle_t h;
        uint8_t setup_flag = 0;
        if (nvs_open("airview", NVS_READWRITE, &h) == ESP_OK) {
            nvs_get_u8(h, "setup_req", &setup_flag);
            if (setup_flag) {
                nvs_set_u8(h, "setup_req", 0);
                nvs_commit(h);
            }
            nvs_close(h);
        }
        if (setup_flag) {
            wifi_driver_init();   /* bare init for scanning, no connect */
            wifi_setup_flow();    /* scan → AP list → keyboard → save+restart */
            esp_restart();        /* reached only if user cancelled */
        }
    }

    /* ── Normal boot: render connecting screen, then start WiFi.
       The 150 ms delay lets the camera's AP finish its own boot sequence;
       the screen render takes ~30 ms so total is safely within the 1100 ms
       window before the first WPA2 handshake attempt. */
    display_splash();
    vTaskDelay(pdMS_TO_TICKS(150));
    wifi_begin_connecting();   /* non-blocking — STA_START fires connect */

    /* ── Connecting-screen poll loop.
       WiFi handshake runs entirely in the background (lwIP / WPA2 stack tasks).
       We just check the event-group bit every 20 ms alongside the touch state.
       This adds zero latency to the connection. */
    const int BTN_X = (LCD_W - 160) / 2, BTN_Y = LCD_H / 2 + 20;
    const int BTN_W = 160, BTN_H = 44;
    s_tap_pending = false;
    for (;;) {
        if (xEventGroupGetBits(s_wifi_events) & WIFI_CONNECTED_BIT)
            break;   /* connected — proceed normally */

        if (s_tap_pending) {
            s_tap_pending = false;
            if (s_tap_x >= BTN_X && s_tap_x < BTN_X + BTN_W &&
                s_tap_y >= BTN_Y && s_tap_y < BTN_Y + BTN_H) {
                /* Write the flag and restart; WiFi driver starts fresh. */
                nvs_handle_t h;
                if (nvs_open("airview", NVS_READWRITE, &h) == ESP_OK) {
                    nvs_set_u8(h, "setup_req", 1);
                    nvs_commit(h);
                    nvs_close(h);
                }
                esp_restart();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    uint8_t  *jpeg_buf = heap_caps_malloc(JPEG_BUF_SIZE, MALLOC_CAP_SPIRAM);
    uint16_t *fb       = heap_caps_malloc((size_t)LCD_W * LCD_H * sizeof(uint16_t), MALLOC_CAP_SPIRAM);

    if (!jpeg_buf || !fb) {
        ESP_LOGE(TAG, "PSRAM alloc failed");
        return;
    }

    memset(fb, 0, (size_t)LCD_W * LCD_H * sizeof(uint16_t));

    liveview_loop(jpeg_buf, fb);
}

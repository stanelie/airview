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

#define WB_CAM_MAX 32
typedef struct { char api[20]; char label[8]; } wb_cam_t;
static wb_cam_t s_wb_cam[WB_CAM_MAX];
static int      s_wb_cam_n = 0;
static int      s_wb_idx   = 0;

#define EXPREV_CAM_MAX 32
typedef struct { char str[8]; } exprev_cam_t;
static exprev_cam_t s_exprev_cam[EXPREV_CAM_MAX];
static int          s_exprev_cam_n = 0;
static int          s_exprev_idx   = 0;

static char s_battery_str[8] = "---";

static volatile bool  s_tap_pending = false;
static volatile uint16_t s_tap_x   = 0;
static volatile uint16_t s_tap_y   = 0;

static volatile bool  s_postview_pending = false;
static char           s_recstate_prev[16] = "recstartable";

static void touch_task(void *arg)
{
    uint16_t x, y;
    while (true) {
        if (touch_poll_tap(&x, &y)) {
            s_tap_x       = x;
            s_tap_y       = y;
            s_tap_pending = true;
        }
        vTaskDelay(pdMS_TO_TICKS(20)); /* 50 Hz */
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


static int s_selected_field = -1;   /* -1=none, 0=shutter, 1=aperture, 2=ISO, 3=exprev */

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
        s_wifi_connected = true;
        xEventGroupSetBits(s_wifi_events, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init(void)
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
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                               wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                               wifi_event_handler, NULL));

    wifi_config_t wifi_cfg = {
        .sta = {
            .ssid           = WIFI_SSID,
            .password       = WIFI_PASS,
            .scan_method    = WIFI_FAST_SCAN,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    /* Disable power save — avoids beacon-miss disconnects with the camera's AP */
    esp_wifi_set_ps(WIFI_PS_NONE);
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "connecting to %s...", WIFI_SSID);
    xEventGroupWaitBits(s_wifi_events, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
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

/* Internal GET — caller must hold s_http_mutex */
static int cam_get_impl(const char *path)
{
    char url[256];
    snprintf(url, sizeof(url), "http://%s%s", CAM_IP, path);
    esp_http_client_config_t cfg = {
        .url = url, .event_handler = http_evt, .timeout_ms = 5000,
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
        .timeout_ms     = 5000,
    };
    xSemaphoreTake(s_http_mutex, portMAX_DELAY);
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
    xSemaphoreGive(s_http_mutex);
    return (status == 200 || status == 520);
}

/* Download binary data (JPEG, XML) directly into a caller-supplied buffer.
   Returns number of bytes received, or 0 on error. */
static int cam_get_binary(const char *path, uint8_t *buf, int buf_size)
{
    char url[256];
    snprintf(url, sizeof(url), "http://%s%s", CAM_IP, path);
    esp_http_client_config_t cfg = {
        .url = url, .timeout_ms = 8000,
    };
    xSemaphoreTake(s_http_mutex, portMAX_DELAY);
    esp_http_client_handle_t c = esp_http_client_init(&cfg);
    esp_http_client_set_header(c, "User-Agent", "OlympusCameraKit");
    esp_http_client_set_header(c, "X-Protocol", "OlympusCameraKit");
    int total = 0;
    if (esp_http_client_open(c, 0) == ESP_OK) {
        esp_http_client_fetch_headers(c);
        int n;
        while (total < buf_size - 1 &&
               (n = esp_http_client_read(c, (char *)buf + total, buf_size - 1 - total)) > 0)
            total += n;
    } else {
        ESP_LOGE(TAG, "cam_get_binary open failed: %s", path);
    }
    buf[total] = '\0';
    esp_http_client_close(c);
    esp_http_client_cleanup(c);
    xSemaphoreGive(s_http_mutex);
    return total;
}

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

static void build_wb_list(void)
{
    static const struct { const char *api; const char *lbl; } map[] = {
        {"WB_AUTO",           "AWB"},
        {"MWB_FINE",          "SUN"},   {"WB_FINE",     "SUN"},
        {"MWB_SHADE",         "SHADE"}, {"WB_SHADE",    "SHADE"},
        {"MWB_CLOUD",         "CLOUD"}, {"WB_CLOUD",    "CLOUD"},
        {"MWB_LAMP",          "TUNG"},  {"WB_LAMP",     "TUNG"},
        {"MWB_FLUORESCENCE1", "FLUO"},  {"WB_FLUORESCENT1","FLUO"},
        {"MWB_FLUORESCENCE2", "FLUO2"}, {"WB_FLUORESCENT2","FLUO2"},
        {"MWB_FLUORESCENCE3", "FLUO3"}, {"WB_FLUORESCENT3","FLUO3"},
        {"MWB_FLUORESCENCE4", "FLUO4"}, {"WB_FLUORESCENT4","FLUO4"},
        {"MWB_WATER_1",       "WATER"}, {"MWB_WATER_2", "WATR2"},
        {"WB_CUSTOM1",        "CUSTM"}, {"WB_CUSTOM2",  "CST2"},
        {"WB_CUSTOM3",        "CST3"},  {"WB_MWB",      "MWB"},
        {"WB_FLASH",          "FLASH"},
        {"MWB_3000","3000"},{"MWB_3300","3300"},{"MWB_4000","4000"},
        {"MWB_4500","4500"},{"MWB_5000","5000"},{"MWB_5500","5500"},
        {"MWB_6000","6000"},{"MWB_6500","6500"},{"MWB_7500","7500"},
    };
    s_wb_cam_n = 0;

    xSemaphoreTake(s_http_mutex, portMAX_DELAY);
    int st = cam_get_impl("/get_camprop.cgi?com=desc&propname=WB");
    if (st == 200 || st == 520) {
        ESP_LOGI(TAG, "WB desc [%d]: %s", st, s_resp);
        char tokens[WB_CAM_MAX][20];
        int n = parse_enum_list(s_resp, tokens, WB_CAM_MAX);
        for (int i = 0; i < n && s_wb_cam_n < WB_CAM_MAX; i++) {
            wb_cam_t *v = &s_wb_cam[s_wb_cam_n++];
            strncpy(v->api, tokens[i], sizeof(v->api) - 1);
            v->api[sizeof(v->api) - 1] = '\0';
            const char *lbl = NULL;
            for (int j = 0; j < (int)(sizeof(map)/sizeof(map[0])); j++) {
                if (strcmp(tokens[i], map[j].api) == 0) { lbl = map[j].lbl; break; }
            }
            if (lbl) {
                strncpy(v->label, lbl, sizeof(v->label) - 1);
            } else {
                strncpy(v->label, tokens[i], sizeof(v->label) - 1);
            }
            v->label[sizeof(v->label) - 1] = '\0';
        }
    }
    xSemaphoreGive(s_http_mutex);

    if (s_wb_cam_n == 0) {
        /* Fallback — populated after first readback reveals actual values */
        static const struct { const char *api; const char *lbl; } fb[] = {
            {"WB_AUTO", "AWB"},
        };
        for (int i = 0; i < (int)(sizeof(fb)/sizeof(fb[0])); i++) {
            strncpy(s_wb_cam[i].api,   fb[i].api, sizeof(s_wb_cam[0].api)   - 1);
            strncpy(s_wb_cam[i].label, fb[i].lbl, sizeof(s_wb_cam[0].label) - 1);
        }
        s_wb_cam_n = sizeof(fb)/sizeof(fb[0]);
    }
    if (s_wb_idx >= s_wb_cam_n) s_wb_idx = 0;
    ESP_LOGI(TAG, "WB: %d entries", s_wb_cam_n);
    for (int i = 0; i < s_wb_cam_n; i++)
        ESP_LOGI(TAG, "  WB[%d] %s = %s", i, s_wb_cam[i].api, s_wb_cam[i].label);
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
            "+0.0",
            "+0.3","+0.7","+1.0","+1.3","+1.7","+2.0","+2.7","+3.0",
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
        cam_set_prop("SHUTTER", s_shutter_cam[idx].str);
        s_osd.shutter_num   = s_shutter_cam[idx].num;
        s_osd.shutter_denom = s_shutter_cam[idx].denom;
        /* Readback: verify the camera accepted the new value */
        { char rb[16] = {0}; if (cam_get_prop("SHUTTER", rb, sizeof(rb)))
            ESP_LOGI(TAG, "SHUTTER readback: %s", rb);
        }

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
        cam_set_prop("APERTURE", s_fnum_cam[idx].str);
        s_osd.fnum_x10 = s_fnum_cam[idx].x10;
        /* Readback: verify the camera accepted the new value */
        { char rb[16] = {0}; if (cam_get_prop("APERTURE", rb, sizeof(rb)))
            ESP_LOGI(TAG, "APERTURE readback: %s", rb);
        }

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
        cam_set_prop("ISO", s_iso_cam[idx].str);
        s_osd.iso = s_iso_cam[idx].val;
        /* Readback: verify the camera accepted the new value */
        { char rb[12] = {0}; if (cam_get_prop("ISO", rb, sizeof(rb)))
            ESP_LOGI(TAG, "ISO readback: %s", rb);
        }

    } else if (field == 3) {
        int n = s_exprev_cam_n;
        if (n == 0) { ESP_LOGW(TAG, "exprev list empty"); return; }

        int idx = s_exprev_idx + delta;
        if (idx < 0) idx = 0;
        if (idx >= n) idx = n - 1;
        ESP_LOGI(TAG, "exprev: %s → %s",
                 s_exprev_cam[s_exprev_idx].str, s_exprev_cam[idx].str);
        cam_set_prop("EXPREV", s_exprev_cam[idx].str);
        s_exprev_idx = idx;
        /* Readback: verify the camera accepted the new value */
        { char rb[12] = {0}; if (cam_get_prop("EXPREV", rb, sizeof(rb)))
            ESP_LOGI(TAG, "EXPREV readback: %s", rb);
        }
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
    /* Strip "SUPPLY_" prefix (camera on external power) then remap known labels */
    const char *label = (strncmp(val, "SUPPLY_", 7) == 0) ? val + 7 : val;
    if (strcmp(label, "WARNING") == 0) label = "LOW";
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
    else if (strcmp(prop_name, "RECSTATE") == 0) {
        bool was_busy  = strcmp(s_recstate_prev, "recstartable") != 0;
        bool now_ready = strcmp(val, "recstartable") == 0;
        if (was_busy && now_ready)
            s_postview_pending = true;
        strncpy(s_recstate_prev, val, sizeof(s_recstate_prev) - 1);
        s_recstate_prev[sizeof(s_recstate_prev) - 1] = '\0';
    }

    s_osd.valid = true;
    ESP_LOGI(TAG, "prop refreshed: %s = %s", prop_name, val);
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
                    char prop[32] = {0};
                    int n = (int)(end - p);
                    if (n > 0 && n < (int)sizeof(prop)) {
                        memcpy(prop, p, n);
                        refresh_prop(prop);
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
        .max_transfer_sz = LCD_W * 40 * sizeof(uint16_t),
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
};

static void display_splash(void)
{
    /* Render into a PSRAM framebuffer then send in 40-row strips — same path
       as the liveview loop, avoiding the D-cache coherency issue that occurs
       when sending many small internal-SRAM strips via DMA. */
    uint16_t *fb = heap_caps_malloc((size_t)LCD_W * LCD_H * sizeof(uint16_t),
                                    MALLOC_CAP_SPIRAM);
    if (!fb) return;
    memset(fb, 0x00, (size_t)LCD_W * LCD_H * sizeof(uint16_t));

    const char *text  = "Connecting...";
    const int   scale = 2;
    const int   cw    = 8 * scale;
    const int   ch    = 8 * scale;
    const int   gap   = 1 * scale;
    int text_len = (int)strlen(text);
    int text_w   = text_len * cw + (text_len - 1) * gap;
    int text_x   = (LCD_W - text_w) / 2;
    int text_y   = (LCD_H - ch)    / 2;

    int cx = text_x;
    for (int i = 0; i < text_len; i++) {
        uint8_t c = (uint8_t)text[i];
        if (c >= 0x20 && c < 0x80) {
            for (int fr = 0; fr < 8; fr++) {
                uint8_t bits = s_font8x8[c - 0x20][fr];
                for (int fc = 0; fc < 8; fc++) {
                    if (!(bits & (1u << fc))) continue;
                    for (int sy = 0; sy < scale; sy++) {
                        int py = text_y + fr * scale + sy;
                        if (py < 0 || py >= LCD_H) continue;
                        for (int sx = 0; sx < scale; sx++) {
                            int px = cx + fc * scale + sx;
                            if (px >= 0 && px < LCD_W)
                                fb[py * LCD_W + px] = 0xFFFF;
                        }
                    }
                }
            }
        }
        cx += cw + gap;
    }

    const int STRIP_H = 40;
    for (int y = 0; y < LCD_H; y += STRIP_H) {
        int h = (y + STRIP_H <= LCD_H) ? STRIP_H : (LCD_H - y);
        esp_lcd_panel_draw_bitmap(s_panel, 0, y, LCD_W, y + h, fb + y * LCD_W);
    }

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

    /* Precompute destination pixel boundaries for this MCU block.
       Ceiling division ensures every destination pixel is covered for
       both upscaling and downscaling: x_own = [xd[col], xd[col+1]). */
    int xd[17], yd[17];
    for (int i = 0; i <= blk_w; i++) {
        int sx  = rect->left + i;
        xd[i] = ctx->x_off + (sx * ctx->dst_w + ctx->src_w - 1) / ctx->src_w;
    }
    for (int i = 0; i <= blk_h; i++) {
        int sy  = rect->top + i;
        yd[i] = ctx->y_off + (sy * ctx->dst_h + ctx->src_h - 1) / ctx->src_h;
    }

    for (int row = 0; row < blk_h; row++) {
        for (int col = 0; col < blk_w; col++) {
            uint16_t px = __builtin_bswap16(*pix++);
            for (int dy = yd[row]; dy < yd[row + 1]; dy++) {
                if (dy < 0 || dy >= LCD_H) continue;
                for (int dx = xd[col]; dx < xd[col + 1]; dx++) {
                    if (dx >= 0 && dx < LCD_W)
                        ctx->fb[dy * LCD_W + dx] = px;
                }
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
static void draw_osd_text(uint16_t *fb, int x0, int y0, int scale, const char *text, bool inverted, bool editable)
{
    const uint16_t bg    = inverted ? 0xFFFF : 0x0000;
    const uint16_t fg    = inverted ? 0x0000 : 0xFFFF;
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

    int cx = x0;
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
    const int centers[3]    = {103, 206, 309};
    const char *strs[3]     = {shutter_str, exprev_str, fnum_str};
    const int field_ids[3]  = {0, 3, 1};

    for (int i = 0; i < 3; i++) {
        int fid    = field_ids[i];
        int len    = (int)strlen(strs[i]);
        int text_w = len * cw + (len > 1 ? (len - 1) * gap : 0);
        draw_osd_text(fb, centers[i] - text_w / 2, y0, scale, strs[i],
                      (fid == s_selected_field), field_selectable(fid, s_shoot_mode));
    }
}

/* ── Drawing helpers ──────────────────────────────────────────────────────── */

static void draw_circle(uint16_t *fb, int cx, int cy, int r, uint16_t color)
{
    int x = 0, y = r, d = 1 - r;
    while (x <= y) {
        int px[8] = {cx+x, cx-x, cx+x, cx-x, cx+y, cx-y, cx+y, cx-y};
        int py[8] = {cy+y, cy+y, cy-y, cy-y, cy+x, cy+x, cy-x, cy-x};
        for (int i = 0; i < 8; i++)
            if ((unsigned)px[i] < (unsigned)LCD_W && (unsigned)py[i] < (unsigned)LCD_H)
                fb[py[i] * LCD_W + px[i]] = color;
        if (d < 0) d += 2 * x + 3; else { d += 2 * (x - y) + 5; y--; }
        x++;
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
    static uint8_t work[4096];

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
        draw_osd_text(fb, (LCD_W - mw) / 2 - 24, 12, 3, mtxt, false, false);

        /* Battery level — right-justified to a fixed right edge so it never moves */
        if (s_battery_str[0] && s_battery_str[0] != '-') {
            int blen      = (int)strlen(s_battery_str);
            int bw        = blen * (8 * 2) + (blen > 1 ? (blen - 1) * 2 : 0);
            int batt_right = (LCD_W + max_mw) / 2 + 60; /* fixed, past outline right edge */
            draw_osd_text(fb, batt_right - bw, 20, 2, s_battery_str, false, false);
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
                      field_selectable(2, s_shoot_mode));
    }
    /* OSD: white balance — top-right column (same x-centre as aperture, 309) */
    {
        const char *wbtxt = (s_wb_cam_n > 0) ? s_wb_cam[s_wb_idx].label : "---";
        int wblen = (int)strlen(wbtxt);
        int wbw   = wblen * (8 * 2) + (wblen > 1 ? (wblen - 1) * 2 : 0);
        draw_osd_text(fb, 309 - wbw / 2, 58, 2, wbtxt, false, (s_shoot_mode != 4));
    }
    /* OSD: shutter / aperture — spread at bottom */
    draw_osd_bottom(fb);

    /* WiFi lost indicator — fill outside circle red */
    if (!s_wifi_connected) {
        fill_outside_circle(fb, __builtin_bswap16(0xF800));
        s_ring_on_fb = true;
    }

    const int STRIP_H = 40;
    for (int y = 0; y < LCD_H; y += STRIP_H) {
        int h = (y + STRIP_H <= LCD_H) ? STRIP_H : (LCD_H - y);
        esp_lcd_panel_draw_bitmap(s_panel, 0, y, LCD_W, y + h, fb + y * LCD_W);
    }
}

/* ── Postview ─────────────────────────────────────────────────────────────── */

/* Fetch the thumbnail of the last image on the camera and hold it on screen
   for one second.  Called from the liveview task; jpeg_buf (PSRAM) is reused
   as a scratch area for both the image-list XML and the thumbnail JPEG. */
static void fetch_and_show_postview(uint8_t *jpeg_buf, uint16_t *fb)
{
    static uint8_t work[4096];

    s_postview_pending = false;

    /* Give the camera a moment to finish writing the file. */
    vTaskDelay(pdMS_TO_TICKS(500));

    /* Fetch the directory listing.  The camera uses /DCIM/100OLYMP/ by
       convention; on error we log and bail out gracefully. */
    int list_len = cam_get_binary("/get_imglist.cgi?DIR=/DCIM/100OLYMP/",
                                  jpeg_buf, JPEG_BUF_SIZE);
    if (list_len <= 0) {
        ESP_LOGW(TAG, "postview: image list empty");
        return;
    }

    /* Scan forward through the XML for the last PATH="..." attribute. */
    char *last_path = NULL;
    char *p = (char *)jpeg_buf;
    while ((p = strstr(p, "PATH=\"")) != NULL) {
        last_path = p + 6;
        p++;
    }
    if (!last_path) {
        ESP_LOGW(TAG, "postview: no PATH in listing: %.*s", list_len > 256 ? 256 : list_len,
                 (char *)jpeg_buf);
        return;
    }
    char *quote_end = strchr(last_path, '"');
    if (!quote_end) return;

    char img_path[64];
    int  path_len = (int)(quote_end - last_path);
    if (path_len <= 0 || path_len >= (int)sizeof(img_path)) return;
    memcpy(img_path, last_path, (size_t)path_len);
    img_path[path_len] = '\0';

    /* Fetch the thumbnail JPEG (jpeg_buf is safe to reuse — img_path is copied). */
    char thumb_path[96];
    snprintf(thumb_path, sizeof(thumb_path), "/get_thumbnail.cgi?PATH=%s", img_path);
    int jpeg_len = cam_get_binary(thumb_path, jpeg_buf, JPEG_BUF_SIZE);
    if (jpeg_len < 100) {
        ESP_LOGW(TAG, "postview: thumbnail too small (%d bytes) for %s", jpeg_len, img_path);
        return;
    }

    ESP_LOGI(TAG, "postview: %s  %d bytes", img_path, jpeg_len);

    /* Decode thumbnail into framebuffer (black background, scaled to fill width). */
    jpeg_ctx_t ctx = {
        .data  = jpeg_buf,
        .len   = (size_t)jpeg_len,
        .pos   = 0,
        .fb    = fb,
        .x_off = 0,
        .y_off = 0,
    };
    JDEC jd;
    if (jd_prepare(&jd, jpeg_infunc, work, sizeof(work), &ctx) != JDR_OK) {
        ESP_LOGE(TAG, "postview: jd_prepare failed");
        return;
    }
    ctx.src_w = jd.width;
    ctx.src_h = jd.height;
    ctx.dst_w = LCD_W;
    ctx.dst_h = (int)jd.height * LCD_W / (int)jd.width;
    ctx.y_off = (LCD_H - ctx.dst_h) / 2;

    memset(fb, 0, (size_t)LCD_W * LCD_H * sizeof(uint16_t));
    if (jd_decomp(&jd, jpeg_outfunc, 0) != JDR_OK) {
        ESP_LOGE(TAG, "postview: jd_decomp failed");
        return;
    }

    const int STRIP_H = 40;
    for (int y = 0; y < LCD_H; y += STRIP_H) {
        int h = (y + STRIP_H <= LCD_H) ? STRIP_H : (LCD_H - y);
        esp_lcd_panel_draw_bitmap(s_panel, 0, y, LCD_W, y + h, fb + y * LCD_W);
    }

    vTaskDelay(pdMS_TO_TICKS(1000));

    /* Mark the ring-on-fb flag so the next liveview frame properly repaints
       the letterbox areas (postview left them black, which is correct, but
       the s_ring_on_fb guard in decode_and_display won't know to clear them). */
    s_ring_on_fb = true;
}

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

    cam_get("/switch_cameramode.cgi?mode=rec");
    cam_get("/exec_takemisc.cgi?com=changelvqty&lvqty=0320x0240");

    char url[64];
    snprintf(url, sizeof(url), "/exec_takemisc.cgi?com=startliveview&port=%d", LV_PORT);
    cam_get(url);

    uint32_t cur_frame  = 0;
    int      jpeg_off   = 0;
    bool     started    = false;
    int      timeouts   = 0;
    uint32_t frames     = 0;

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
                    cam_set_prop("TAKEMODE", s_mode_api[s_shoot_mode]);
                    build_prop_lists();
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
                        cam_set_prop("WB", s_wb_cam[s_wb_idx].api);
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
                    /* Tap on main image: touch AF at tapped point (focus only, shot cancelled) */
                    int lv_x, lv_y;
                    if (tap_to_liveview(tx, ty, &lv_x, &lv_y)) {
                        s_af_color = 0;
                        char url[80];
                        snprintf(url, sizeof(url),
                                 "/exec_takemotion.cgi?com=newstarttake&point=%04dx%04d",
                                 lv_x, lv_y);
                        ESP_LOGI(TAG, "touch AF → lv (%d,%d)", lv_x, lv_y);
                        cam_get(url);
                        vTaskDelay(pdMS_TO_TICKS(300));
                        cam_get("/exec_takemotion.cgi?com=newstoptake");
                    }
                }
            }
            decode_and_display(jpeg_buf, jpeg_off, fb);
            frames++;
            if (frames % 30 == 0)
                ESP_LOGI(TAG, "%" PRIu32 " frames", frames);
            started  = false;
            jpeg_off = 0;

            if (s_postview_pending)
                fetch_and_show_postview(jpeg_buf, fb);
        }
    }

    /* unreachable, but tidy up if ever reached */
    cam_get("/exec_takemisc.cgi?com=stopliveview");
    close(sock);
}

/* ── Entry point ──────────────────────────────────────────────────────────── */

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());

    s_http_mutex = xSemaphoreCreateMutex();

    display_init();
    touch_init();
    xTaskCreate(touch_task, "touch", 2048, NULL, 5, NULL);
    display_splash();

    wifi_init();
    cam_get("/get_connectmode.cgi");
    cam_get("/switch_cameramode.cgi?mode=rec");

    xTaskCreate(pushevent_task, "pushevent", 4096, NULL, 4, NULL);

    /* Read current shooting mode from camera */
    char mode_val[8] = "P";
    if (cam_get_prop("TAKEMODE", mode_val, sizeof(mode_val))) {
        for (int i = 0; i < NUM_MODES; i++) {
            if (strcmp(mode_val, s_mode_api[i]) == 0) {
                s_shoot_mode = i;
                break;
            }
        }
    }
    ESP_LOGI(TAG, "shoot mode: %s", s_mode_display[s_shoot_mode]);

    /* Fetch camera's WB value list and read current WB */
    build_wb_list();
    char wb_val[20] = {0};
    if (cam_get_prop("WB", wb_val, sizeof(wb_val))) {
        for (int i = 0; i < s_wb_cam_n; i++) {
            if (strcmp(wb_val, s_wb_cam[i].api) == 0) { s_wb_idx = i; break; }
        }
    }
    ESP_LOGI(TAG, "WB current: %s", s_wb_cam_n > 0 ? s_wb_cam[s_wb_idx].label : "?");

    /* Fetch camera's EXPREV value list and read current value */
    build_exprev_list();
    char exprev_val[12] = {0};
    if (cam_get_prop("EXPREV", exprev_val, sizeof(exprev_val))) {
        for (int i = 0; i < s_exprev_cam_n; i++) {
            if (strcmp(exprev_val, s_exprev_cam[i].str) == 0) { s_exprev_idx = i; break; }
        }
        ESP_LOGI(TAG, "EXPREV current: %s", exprev_val);
    }

    /* Read current battery level */
    {
        char batt_val[12] = {0};
        if (cam_get_prop("BATTERY_LEVEL", batt_val, sizeof(batt_val))) {
            set_battery_str(batt_val);
            ESP_LOGI(TAG, "BATTERY_LEVEL: %s → %s", batt_val, s_battery_str);
        }
    }

    /* Query camera's permitted value lists for adjustable properties */
    build_prop_lists();

    uint8_t  *jpeg_buf = heap_caps_malloc(JPEG_BUF_SIZE, MALLOC_CAP_SPIRAM);
    uint16_t *fb       = heap_caps_malloc((size_t)LCD_W * LCD_H * sizeof(uint16_t), MALLOC_CAP_SPIRAM);

    if (!jpeg_buf || !fb) {
        ESP_LOGE(TAG, "PSRAM alloc failed");
        return;
    }

    memset(fb, 0, (size_t)LCD_W * LCD_H * sizeof(uint16_t));

    liveview_loop(jpeg_buf, fb);
}

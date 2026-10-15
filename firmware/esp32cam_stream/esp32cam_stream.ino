// ESP32-CAM (AI Thinker) -> live MJPEG stream over WiFi
// Stream URL:  http://<printed-ip>/stream      Test page: http://<printed-ip>/
// Arduino IDE: Board = "AI Thinker ESP32-CAM", PSRAM enabled (default for this board)

#include "esp_camera.h"
#include <WiFi.h>
#include "esp_http_server.h"

// ---- EDIT THESE ----
const char* WIFI_SSID = "YOUR_WIFI_NAME";
const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";
// --------------------

// AI Thinker pin map
#define PWDN_GPIO_NUM   32
#define RESET_GPIO_NUM  -1
#define XCLK_GPIO_NUM    0
#define SIOD_GPIO_NUM   26
#define SIOC_GPIO_NUM   27
#define Y9_GPIO_NUM     35
#define Y8_GPIO_NUM     34
#define Y7_GPIO_NUM     39
#define Y6_GPIO_NUM     36
#define Y5_GPIO_NUM     21
#define Y4_GPIO_NUM     19
#define Y3_GPIO_NUM     18
#define Y2_GPIO_NUM      5
#define VSYNC_GPIO_NUM  25
#define HREF_GPIO_NUM   23
#define PCLK_GPIO_NUM   22

static esp_err_t index_handler(httpd_req_t *req) {
  const char* html = "<html><body style='background:#111;color:#eee;font-family:sans-serif'>"
                     "<h3>ESP32-CAM stream</h3><img src='/stream' style='max-width:100%'></body></html>";
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t *fb = NULL;
  esp_err_t res = httpd_resp_set_type(req, "multipart/x-mixed-replace;boundary=frame");
  if (res != ESP_OK) return res;
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  char part[64];
  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) { res = ESP_FAIL; }
    else {
      size_t hlen = snprintf(part, sizeof(part),
        "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n", (unsigned)fb->len);
      res = httpd_resp_send_chunk(req, "--frame\r\n", 9);
      if (res == ESP_OK) res = httpd_resp_send_chunk(req, part, hlen);
      if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char*)fb->buf, fb->len);
      if (res == ESP_OK) res = httpd_resp_send_chunk(req, "\r\n", 2);
      esp_camera_fb_return(fb);
    }
    if (res != ESP_OK) break;   // client disconnected
  }
  return res;
}

void startServer() {
  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  cfg.server_port = 80;
  httpd_handle_t server = NULL;
  httpd_uri_t index_uri  = { .uri = "/",       .method = HTTP_GET, .handler = index_handler,  .user_ctx = NULL };
  httpd_uri_t stream_uri = { .uri = "/stream", .method = HTTP_GET, .handler = stream_handler, .user_ctx = NULL };
  if (httpd_start(&server, &cfg) == ESP_OK) {
    httpd_register_uri_handler(server, &index_uri);
    httpd_register_uri_handler(server, &stream_uri);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println();

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk  = XCLK_GPIO_NUM;
  config.pin_pclk  = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href  = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;   // if compile error, rename to pin_sscb_sda / pin_sscb_scl
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn  = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size   = FRAMESIZE_QVGA;   // 320x240: fast and enough for obstacle detection
  config.jpeg_quality = 12;               // lower number = better quality, slower
  config.grab_mode    = CAMERA_GRAB_LATEST;

  if (psramFound()) {
    config.fb_count = 2;
    config.fb_location = CAMERA_FB_IN_PSRAM;
  } else {
    config.fb_count = 1;
    config.fb_location = CAMERA_FB_IN_DRAM;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed 0x%x (check ribbon cable and power)\n", err);
    return;
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED) { delay(400); Serial.print("."); }
  WiFi.setSleep(false);   // lower latency

  startServer();
  Serial.println();
  Serial.print("Stream ready: http://");
  Serial.print(WiFi.localIP());
  Serial.println("/stream");
}

void loop() { delay(10000); }

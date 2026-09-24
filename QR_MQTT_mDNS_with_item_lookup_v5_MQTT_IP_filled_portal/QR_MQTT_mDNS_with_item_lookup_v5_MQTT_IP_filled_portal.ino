/*
  ESP32-CAM QR Code Reader + Gate Status -> MQTT (AGV inbound)

  Board:     AI Thinker ESP32-CAM (OV3660 sensor)
  Libraries: WiFi (bundled with ESP32 core), PubSubClient, quirc (patched per
             project notes: stdlib/string headers added, ps_malloc -> malloc)

  What this sketch does
  ----------------------
  - Runs the original QR_Warehouse camera web server (live MJPEG stream,
    on-page QR result / gate status, LED flash slider) unchanged.
  - Reads an SPST switch on GATE_SWITCH_PIN as the gate/lane status
    (GATE 1 / GATE 2), same wiring and pull-up logic as before.
  - On every SUCCESSFUL QR decode, publishes MQTT:
        topic:   agv/R1/inbound
        payload: {"serial_code":"<decoded QR text>","pickup_location":"<GATE 1|GATE 2>"}
  - De-duplicates: if the newly decoded QR text is the same as the last one
    that was actually published, it is NOT sent again. A new send is
    triggered again only once a *different* QR code is decoded.

  Notes / things to double-check for your setup
  ----------------------------------------------
  - Wi-Fi credentials below are the QR_Warehouse sketch's ("cam" network).
    The original MQTT test sketch used a different network ("Aml Office 4th") -
    update `ssid`/`password` to whichever network the broker is actually on.
  - MQTT payload keys reuse the existing "agv/R1/inbound" schema
    (serial_code / pickup_location). The gate string here is "GATE 1"/"GATE 2"
    (from the SPST switch), which differs in format from the "Gate_1" sample
    in the original MQTT sketch - rename on whichever side needs to match.
  - Decoding and MQTT publishing run on different FreeRTOS tasks/cores
    (QR decode on core 0, Wi-Fi/MQTT in loop() on core 1), so the handoff
    between them is protected with a mutex.
*/

#include "esp_camera.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "quirc.h"
#include <WiFi.h>
#include <WiFiManager.h>
#include "esp_http_server.h"
#include <PubSubClient.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <ESPmDNS.h>
/* ======================================== */

/* ======================================== AI-Thinker ESP32-CAM GPIO pin mapping (hard-coded - this project only targets this board) */
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22
/* ======================================== */

// LEDs GPIO
#define LED_OnBoard 4

// Gate select switch (SPST) GPIO
// One leg to GPIO13, other leg to GND.
// Uses internal pull-up: switch OFF (open) -> HIGH -> "GATE 1"
//                         switch ON  (closed to GND) -> LOW -> "GATE 2"
#define GATE_SWITCH_PIN 13

// Task handle. Created once in setup() and never deleted - the task parks on
// ws_run while a browser is streaming instead of being killed and recreated.
TaskHandle_t QRCodeReader_Task = NULL;

/* ======================================== Variables declaration */
struct quirc *q = NULL;
uint8_t *image = NULL;
struct quirc_code code;
struct quirc_data data;
quirc_decode_error_t err;
String QRCodeResult = "";
String QRCodeResultSend = "";
/* ======================================== */

/* ======================================== Gate switch status (for the on-page display) */
String gateStatus = "Gate_1";
/* ======================================== */

/* ======================================== Item master list: decoded QR text -> (item name, category).
   Mirrors the Python ITEMS list used elsewhere in the project.
   NOTE: unlike AVR boards, the ESP32's flash is memory-mapped, so this table
   doesn't need PROGMEM/pgm_read_* to read it - plain const access is fine
   and keeps the lookup function simple. */
struct ItemRecord {
  const char* code;
  const char* name;
  const char* category;
};

const ItemRecord ITEM_TABLE[] = {
  {"E0001", "Arduino UNO",           "Electronics"},
  {"E0002", "Arduino NANO",          "Electronics"},
  {"E0003", "Arduino MEGA",          "Electronics"},
  {"E0004", "ESP8266",               "Electronics"},
  {"E0005", "ESP32",                 "Electronics"},
  {"E0006", "ESP32-CAM",             "Electronics"},
  {"E0007", "STM32",                 "Electronics"},
  {"E0008", "Raspberry PI",          "Electronics"},
  {"E0009", "Resistors",             "Electronics"},
  {"E0010", "Capacitors",            "Electronics"},
  {"M0001", "Motors",                "Mechanical Parts"},
  {"M0002", "Gears",                 "Mechanical Parts"},
  {"M0003", "Bearings",              "Mechanical Parts"},
  {"M0004", "Shafts",                "Mechanical Parts"},
  {"M0005", "Screws",                "Mechanical Parts"},
  {"M0006", "Nuts",                  "Mechanical Parts"},
  {"M0007", "Brackets",              "Mechanical Parts"},
  {"M0008", "Pulleys",               "Mechanical Parts"},
  {"M0009", "Springs",               "Mechanical Parts"},
  {"M0010", "Washers",               "Mechanical Parts"},
  {"R0001", "Steel Sheets",          "Raw Materials"},
  {"R0002", "Aluminum Sheets",       "Raw Materials"},
  {"R0003", "Plastic Sheets",        "Raw Materials"},
  {"R0004", "Copper Wire",           "Raw Materials"},
  {"R0005", "Rubber Sheets",         "Raw Materials"},
  {"R0006", "PVC Pipes",             "Raw Materials"},
  {"R0007", "Wood Panels",           "Raw Materials"},
  {"R0008", "Acrylic Sheets",        "Raw Materials"},
  {"R0009", "Stainless Steel Rods",  "Raw Materials"},
  {"R0010", "Glass sheets",          "Raw Materials"},
  {"F0001", "Electric Fans",         "Final Products"},
  {"F0002", "LED Lamps",             "Final Products"},
  {"F0003", "Power Adapters",        "Final Products"},
  {"F0004", "Power Supply Units",    "Final Products"},
  {"F0005", "Smart Watches",         "Final Products"},
  {"F0006", "Digital Clocks",        "Final Products"},
  {"F0007", "IoT Devices",           "Final Products"},
  {"F0008", "Digital Thermometers",  "Final Products"},
  {"F0009", "Bluetooth Speakers",    "Final Products"},
  {"F0010", "Wireless Earphones",    "Final Products"},
};
const size_t ITEM_TABLE_COUNT = sizeof(ITEM_TABLE) / sizeof(ITEM_TABLE[0]);

/* Looks up a decoded QR code in ITEM_TABLE. Returns true and fills outName /
   outCategory on a match; returns false (leaving them untouched) otherwise -
   e.g. for "", "Decoding FAILED", or a QR payload that isn't a known code. */
bool lookupItem(const String &code, String &outName, String &outCategory) {
  for (size_t i = 0; i < ITEM_TABLE_COUNT; i++) {
    if (code == ITEM_TABLE[i].code) {
      outName = ITEM_TABLE[i].name;
      outCategory = ITEM_TABLE[i].category;
      return true;
    }
  }
  return false;
}
/* ======================================== */

/* ======================================== */
volatile bool ws_run = false;  // written by the HTTP task, read by the QR task
/* ======================================== */

/* ======================================== Wi-Fi setup portal (WiFiManager) - ADJUSTABLE SETTINGS
   Give each of your 3 ESP32-CAMs a unique number (1, 2, 3) so their setup
   hotspots don't collide when multiple are being configured at the same
   time. This board is camera 3. Only the single most-recently configured
   network is kept in flash - WiFiManager's default, nothing extra needed. */
#define DEVICE_ID 3
#define CONFIG_TIMEOUT_SECONDS 20   // how long to try the saved network before opening the portal
#define PORTAL_TIMEOUT_SECONDS 120  // how long the setup portal stays open before giving up
#define HOTSPOT_SSID_BASE "ESP32-CAM-Setup"
#define HOTSPOT_PASSWORD  "setup1234"

String buildHotspotSsid() {
  return String(HOTSPOT_SSID_BASE) + "-" + String(DEVICE_ID);
}

// Set true by WiFiManager only when the setup portal form is actually
// submitted (not on a silent reconnect to an already-saved network).
bool shouldSaveMqttHost = false;
void onWiFiManagerSave() {
  shouldSaveMqttHost = true;
}
/* ======================================== */

/* ======================================== MQTT (inbound reporting) */
#include <Preferences.h>
Preferences prefs;
String mqttHost = "10.203.122.32";  // default; overridden by saved value / setup portal
const uint16_t MQTT_PORT = 1883;

const char* ROBOT_ID = "R1";
const char* INBOUND_TOPIC = "agv/R1/inbound";
const char* QRIP_TOPIC = "cam/qrip";  // one-shot: this device's current IP, sent on each MQTT (re)connect

WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

unsigned long lastMqttAttempt = 0;
const unsigned long mqttRetryInterval = 5000; // ms between reconnect attempts
/* ======================================== */

/* ======================================== Shared state between the QR-decode context
   (QRCodeReader_Task on core 0, or the stream_handler HTTP task) and the
   MQTT-publish context (Arduino loop(), core 1). Access is mutex-protected. */
SemaphoreHandle_t qrDataMutex = NULL;
SemaphoreHandle_t decodeMutex = NULL;  // guards the shared quirc globals above
String lastSentQR = "";     // last payload actually confirmed published (for de-dup)
String pendingQR = "";      // payload waiting to be published
String pendingGate = "";    // gate status captured at the moment of decode
bool   qrPendingSend = false;
/* ======================================== */

/* ======================================== */
#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";
/* ======================================== */

/* ======================================== Empty handle to esp_http_server */
httpd_handle_t index_httpd = NULL;
httpd_handle_t stream_httpd = NULL;
/* ======================================== */

/* ======================================== HTML code for index / main page */
static const char PROGMEM INDEX_HTML[] = R"rawliteral(
<html>

  <head>
  
    <title>ESP32-CAM QR Code Reader Stream Web Server</title>
    
    <meta name="viewport" content="width=device-width, initial-scale=1">
    
    <style>

      * { box-sizing: border-box; }

      body {
        font-family: -apple-system, 'Segoe UI', Roboto, Arial, sans-serif;
        margin: 0;
        /* fluid padding: 16px on a phone, 28px on a desktop, no media query */
        padding: clamp(14px, 3vw, 28px) clamp(14px, 3vw, 32px) 24px;
        background: #f1f5f9;
        color: #0f172a;
      }

      /* ----------------------------------- Top bar */
      .topbar {
        display: flex;
        justify-content: space-between;
        align-items: center;
        flex-wrap: wrap;
        gap: 16px;
        margin-bottom: 16px;
        text-align: left;
      }

      .eyebrow {
        font-size: 12px;
        font-weight: 700;
        letter-spacing: 1.5px;
        text-transform: uppercase;
        color: #2563eb;
        margin: 0 0 6px;
      }

      h1 {
        font-size: clamp(20px, 5vw, 30px);
        font-weight: 700;
        margin: 0;
        color: #0f172a;
      }

      .topbar-actions {
        display: flex;
        align-items: center;
        gap: 12px;
        flex-wrap: wrap;
      }

      .pill {
        display: inline-flex;
        align-items: center;
        gap: 8px;
        background: #dcfce7;
        color: #15803d;
        font-weight: 600;
        font-size: 14px;
        padding: 8px 16px;
        border-radius: 999px;
      }

      .pill .dot {
        width: 8px;
        height: 8px;
        border-radius: 50%;
        background: #22c55e;
        display: inline-block;
      }

      /* ----------------------------------- LED Flash (now a compact inline
         control in the top bar instead of its own card in the status grid -
         it's a control, not a status readout, so it doesn't belong grouped
         with QR/Category/Item/Gate). */
      .led-flash {
        display: inline-flex;
        align-items: center;
        gap: 10px;
        background: #fff;
        padding: 8px 16px;
        border-radius: 999px;
        box-shadow: 0 1px 3px rgba(0,0,0,0.06);
      }

      .led-flash label {
        font-size: 12px;
        font-weight: 700;
        letter-spacing: 0.5px;
        text-transform: uppercase;
        color: #64748b;
        white-space: nowrap;
      }

      .led-flash .slider { width: 110px; margin-top: 0; }

      .btn-dark {
        background: #0f172a;
        color: #fff;
        border: none;
        padding: 10px 20px;
        border-radius: 10px;
        font-weight: 600;
        font-size: 14px;
        cursor: pointer;
        text-decoration: none;
        display: inline-block;
      }

      .btn-dark:hover { background: #1e293b; }

      /* ----------------------------------- Cards */
      .card {
        background: #fff;
        border-radius: 16px;
        padding: clamp(12px, 2.5vw, 18px) clamp(14px, 2.5vw, 20px);
        box-shadow: 0 1px 3px rgba(0,0,0,0.06);
        margin-bottom: 16px;
        text-align: left;
      }

      .card-title {
        font-size: 12px;
        font-weight: 700;
        letter-spacing: 0.5px;
        text-transform: uppercase;
        color: #64748b;
        margin: 0 0 10px;
      }

      .grid {
        display: grid;
        /* min(180px,100%) so the track can never be wider than the screen -
           plain minmax(180px,1fr) is what pushes a 320px-wide phone into
           horizontal scrolling. Kept narrower than before (was 220px) now
           that the grid only holds the 4 status cards, so they line up in
           a single row on typical laptop widths instead of one wrapping
           awkwardly onto its own line. */
        grid-template-columns: repeat(auto-fit, minmax(min(180px, 100%), 1fr));
        gap: clamp(12px, 2.5vw, 20px);
        margin-bottom: 16px;
      }

      /* ----------------------------------- Stream Viewer */
      img#vdstream {
        width: 100%;
        max-width: 380px;   /* trimmed from 480px - the QVGA (320x240) source
                                doesn't need to be shown larger than this, and
                                the shorter box leaves more of the page visible
                                above the fold */
        aspect-ratio: 4 / 3; /* QVGA shape - keeps the box the right size while
                                the stream is loading, retrying or broken,
                                instead of the giant dark rectangle */
        object-fit: contain;
        display: block;
        margin: 0 auto;
        border-radius: 10px;
        background: #0f172a;
      }

      /* ----------------------------------- Slider */
      .slidecontainer {
        width: 100%;
      }

      .slider {
        -webkit-appearance: none;
        width: 100%;
        height: 8px;
        border-radius: 5px;
        background: #e2e8f0;
        outline: none;
        margin-top: 8px;
      }

      .slider::-webkit-slider-thumb {
        -webkit-appearance: none;
        appearance: none;
        width: 20px;
        height: 20px;
        border-radius: 50%;
        background: #2563eb;
        cursor: pointer;
      }

      .slider::-moz-range-thumb {
        width: 20px;
        height: 20px;
        border-radius: 50%;
        background: #2563eb;
        border: none;
        cursor: pointer;
      }
      /* ----------------------------------- */

      /* ----------------------------------- Result value boxes */
      .value-box {
        background: #0f172a;
        color: #38bdf8;
        font-weight: 700;
        font-size: 16px;
        text-align: center;
        padding: 12px;
        border-radius: 10px;
        min-height: 20px;
        word-break: break-word;
      }

      /* Category-colored value boxes, matching the warehouse robots
         dashboard's shelf-map palette so an item's color reads the same on
         both pages. Applied to the Category and Item boxes together (via
         JS, once the category is known) so a scanned item's two boxes are
         visually grouped; the QR result and Gate Status boxes always keep
         the default dark styling above. */
      .value-box.cat-electronics { background: #f4d35e; color: #0f172a; }
      .value-box.cat-mechanical  { background: #8fcb7b; color: #0f172a; }
      .value-box.cat-final       { background: #a9a0d1; color: #0f172a; }
      .value-box.cat-raw         { background: #f2925c; color: #0f172a; }

      /* ----------------------------------- Device IP bar (footer-style strip
         at the bottom of the status area, above the action buttons) */
      .ip-bar {
        display: flex;
        align-items: center;
        justify-content: center;
        gap: 10px;
        flex-wrap: wrap;
        padding: 10px 16px;
      }

      .ip-bar .card-title { margin: 0; }

      .ip-bar .value-box {
        display: inline-block;
        padding: 6px 14px;
        font-size: 14px;
      }

      /* ----------------------------------- Action buttons */
      .actions {
        display: flex;
        gap: 12px;
        justify-content: center;
        flex-wrap: wrap;
      }

      .btn-outline {
        background: #fff;
        border: 1px solid #cbd5e1;
        color: #0f172a;
        padding: 10px 20px;
        border-radius: 10px;
        font-weight: 600;
        font-size: 14px;
        cursor: pointer;
      }

      .btn-outline:hover { background: #f1f5f9; }

    </style>
    
  </head>
  
  <body>

    <div class="topbar">
      <div>
        <p class="eyebrow">Live Vision Monitor</p>
        <h1>QR Code Reader with ESP32CAM</h1>
      </div>
      <div class="topbar-actions">
        <span class="pill"><span class="dot"></span>Camera service active</span>
        <div class="led-flash">
          <label for="mySlider">LED Flash</label>
          <input type="range" min="0" max="20" value="0" class="slider" id="mySlider">
        </div>
        <a class="btn-dark" href="http:&#47;&#47;localhost:5173/">Back to warehouse</a>
      </div>
    </div>

    <div class="card">
      <p class="card-title">Live Stream</p>
      <img src="" id="vdstream">
    </div>

    <div class="grid">
      <div class="card">
        <p class="card-title">QR Code Scan Result</p>
        <div class="value-box" id="showqrcodeval"></div>
      </div>

      <div class="card">
        <p class="card-title">Category</p>
        <div class="value-box" id="showcategoryval"></div>
      </div>

      <div class="card">
        <p class="card-title">Item</p>
        <div class="value-box" id="showitemval"></div>
      </div>

      <div class="card">
        <p class="card-title">Gate Status</p>
        <div class="value-box" id="showgateval"></div>
      </div>
    </div>

    <div class="card ip-bar">
      <p class="card-title">Device IP Address</p>
      <div class="value-box" id="showipval"></div>
    </div>

    <div class="actions">
      <button type="button" class="btn-outline" onclick="send_btn_cmd('clr')">Clear Result</button>
    </div>
    
    <script>
      /* ----------------------------------- Video stream, with automatic retry.
         A refresh leaves the previous /stream connection open for a moment, so
         the new request can be refused (HTTP 500) or time out. The old one-shot
         src assignment gave up there and the box stayed black forever; now any
         error just re-requests the stream every 2s until it comes back.
         The ?t= cache-buster stops the browser reusing the dead response. */
      var vd = document.getElementById("vdstream");

      function streamURL() {
        /* split so the source never contains adjacent slash characters -
           see the note above this block about the ctags/prototype issue */
        var proto = window.location.protocol + '/' + '/';
        return proto + window.location.hostname + ':81/stream?t=' + Date.now();
      }

      function startStream() {
        vd.src = streamURL();
      }

      vd.onerror = function() { setTimeout(startStream, 2000); };
      startStream();

      /* Re-request when the tab comes back to the foreground: mobile
         browsers suspend the MJPEG socket on background/lock and it does
         not resume on its own. */
      document.addEventListener("visibilitychange", function() {
        if (!document.hidden) startStream();
      });
      /* ----------------------------------- */
      
      var slider = document.getElementById("mySlider");
      
      /* ----------------------------------- Variable declaration and timer to display QR Code reading results. */
      var myTmr;
      start_timer();
      /* ----------------------------------- */

      /* :::::::::::::::::::::::::::::::::::::::::::::::: Update the current slider value (each time you drag the slider handle) */
      slider.oninput = function() {
        let slider_pwm_val = "S," + slider.value;
        send_cmd(slider_pwm_val);
      }
      /* :::::::::::::::::::::::::::::::::::::::::::::::: */

      /* :::::::::::::::::::::::::::::::::::::::::::::::: Function for sending commands */
      function send_cmd(cmds) {
        var xhr = new XMLHttpRequest();
        xhr.open("GET", "/action?go=" + cmds, true);
        xhr.send();
      }
      /* :::::::::::::::::::::::::::::::::::::::::::::::: */

      /* :::::::::::::::::::::::::::::::::::::::::::::::: Start and stop the timer */
      function start_timer() {
        myTmr = setInterval(myTimer, 500)
      }
      
      function stop_timer() {
        clearInterval(myTmr)
      }
      /* :::::::::::::::::::::::::::::::::::::::::::::::: */

      /* :::::::::::::::::::::::::::::::::::::::::::::::: Timer to get QR Code reading result + category
         + item name + gate status + device IP in a single request (was 2
         separate XHRs to 2 separate handlers every 500ms; now 1 request to
         /status, split on the client). Fewer HTTP round trips = lighter
         load on the ESP32's web server, which leaves more headroom for the
         MJPEG stream to run smoothly.
         Response format: "<qr code>|<category>|<item name>|<gate status>|<ip>" */
      function myTimer() {
        var xhttp = new XMLHttpRequest();
        xhttp.onreadystatechange = function() {
          if (this.readyState == 4 && this.status == 200) {
            var parts = this.responseText.split("|");
            document.getElementById("showqrcodeval").innerHTML = parts[0];
            document.getElementById("showcategoryval").innerHTML = parts[1];
            document.getElementById("showitemval").innerHTML = parts[2];
            document.getElementById("showgateval").innerHTML = parts[3];
            document.getElementById("showipval").innerHTML = parts[4];
            applyCategoryStyle(parts[1]);
          }
        };
        xhttp.open("GET", "/status", true);
        xhttp.send();
      }
      /* :::::::::::::::::::::::::::::::::::::::::::::::: */

      /* :::::::::::::::::::::::::::::::::::::::::::::::: Colors the Category
         and Item boxes to match the given category, using the same palette
         as the warehouse robots dashboard's shelf map. Falls back to the
         default dark box (no class) for an empty/unrecognized category. */
      var CATEGORY_CLASS = {
        "Electronics":      "cat-electronics",
        "Mechanical Parts": "cat-mechanical",
        "Final Products":   "cat-final",
        "Raw Materials":    "cat-raw"
      };

      function applyCategoryStyle(category) {
        var catBox = document.getElementById("showcategoryval");
        var itemBox = document.getElementById("showitemval");
        for (var key in CATEGORY_CLASS) {
          catBox.classList.remove(CATEGORY_CLASS[key]);
          itemBox.classList.remove(CATEGORY_CLASS[key]);
        }
        var cls = CATEGORY_CLASS[category];
        if (cls) {
          catBox.classList.add(cls);
          itemBox.classList.add(cls);
        }
      }
      /* :::::::::::::::::::::::::::::::::::::::::::::::: */

      /* :::::::::::::::::::::::::::::::::::::::::::::::: Function to send commands to the ESP32 Cam whenever the button is clicked. */
      function send_btn_cmd(cmds) {
        let btn_cmd = "B," + cmds;
        send_cmd(btn_cmd);
      }
      /* :::::::::::::::::::::::::::::::::::::::::::::::: */

    </script>
  
  </body>
  
</html>
)rawliteral";
/* ======================================== */

/* ________________________________________________________________________________ Index handler function to be called during GET or uri request */
static esp_err_t index_handler(httpd_req_t *req){
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, (const char *)INDEX_HTML, strlen(INDEX_HTML));
}
/* ________________________________________________________________________________ */

/* ________________________________________________________________________________ Shared QR-decode step used by both stream_handler() and QRCodeReader().
   Copies fb into quirc's own buffer, decodes, and reports via dumpData() on
   success. Only touches fb during the copy, so the caller is free to
   return/reuse fb immediately afterwards. */
static void decodeQRFromFrame(camera_fb_t *fb) {
  // q / image / code / data are globals shared by this task and the stream
  // handler. They can overlap for a frame or two right when a stream starts
  // or ends (one context calling quirc_destroy() while the other is mid-
  // decode = heap corruption), so serialize the whole decode.
  // code/data stay global on purpose: quirc_data alone is ~9KB, too big for
  // this task's 10KB stack.
  if (xSemaphoreTake(decodeMutex, portMAX_DELAY) != pdTRUE) return;

  q = quirc_new();
  if (q == NULL) {
    Serial.print("can't create quirc object\r\n");
    xSemaphoreGive(decodeMutex);
    return;
  }

  quirc_resize(q, fb->width, fb->height);
  image = quirc_begin(q, NULL, NULL);
  memcpy(image, fb->buf, fb->len);
  quirc_end(q);
  image = NULL;

  int count = quirc_count(q);
  if (count > 0) {
    quirc_extract(q, 0, &code);
    err = quirc_decode(&code, &data);

    if (err) {
      QRCodeResult = "Decoding FAILED";
      Serial.println(QRCodeResult);
    } else {
      Serial.printf("Decoding successful:\n");
      dumpData(&data);
    }
    Serial.println();
  }
  quirc_destroy(q);
  q = NULL;
  xSemaphoreGive(decodeMutex);
}
/* ________________________________________________________________________________ */

/* ________________________________________________________________________________ stream handler function to be called during GET or uri request. */
static esp_err_t stream_handler(httpd_req_t *req){
  // Guard against a second /stream request (e.g. a page refresh landing
  // before the previous connection is torn down) racing this one. The
  // rejected request is no longer fatal for the page: the browser retries
  // automatically every 2s (see vd.onerror in INDEX_HTML).
  static volatile bool streamActive = false;
  if (streamActive) {
    Serial.println("stream_handler: a stream is already active, rejecting");
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }
  streamActive = true;

  // ROOT CAUSE OF "camera never comes back after a refresh":
  // this used to vTaskDelete(QRCodeReader_Task) here. That task is almost
  // always inside esp_camera_fb_get() or quirc_new()/quirc_destroy() at that
  // moment, so deleting it strands a camera frame buffer and leaks the quirc
  // allocation. After a refresh or two the driver has no buffer left to hand
  // out and every esp_camera_fb_get() returns NULL - the stream dies and
  // stays dead until a power cycle.
  // The task is no longer deleted or recreated: it just parks itself while
  // ws_run is true (see QRCodeReader()), so nothing is ever killed mid-call.
  ws_run = true;
  Serial.print("stream_handler running on core ");
  Serial.println(xPortGetCoreID());

  camera_fb_t * fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t * _jpg_buf = NULL;
  char part_buf[64];
  uint32_t frame_counter = 0;
  const uint32_t DECODE_EVERY_N_FRAMES = 3; // scan for QR on 1 out of every N streamed frames

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if(res != ESP_OK){
    streamActive = false;
    ws_run = false;   // QR task resumes on its own
    return res;
  }

  /* ---------------------------------------- Loop to show streaming video from ESP32 Cam camera and read QR Code. */
  while(true){
    bool tryDecode = false;

    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed (stream_handler)");
      res = ESP_FAIL;
    } else {
      frame_counter++;
      tryDecode = (frame_counter % DECODE_EVERY_N_FRAMES == 0);

      // fb->buf gets copied into quirc's own buffer inside decodeQRFromFrame(),
      // so decoding happens after the JPEG conversion below has released fb -
      // the camera driver gets its buffer back sooner either way.
      camera_fb_t *fbForDecode = fb;

      if(fb->width > 200){
        if(fb->format != PIXFORMAT_JPEG){
          bool jpeg_converted = frame2jpg(fb, 24, &_jpg_buf, &_jpg_buf_len);
          if (tryDecode) decodeQRFromFrame(fbForDecode);
          esp_camera_fb_return(fb);
          fb = NULL;
          if(!jpeg_converted){
            Serial.println("JPEG compression failed");
            res = ESP_FAIL;
          }
        } else {
          _jpg_buf_len = fb->len;
          _jpg_buf = fb->buf;
          if (tryDecode) decodeQRFromFrame(fbForDecode);
        }
      } else if (tryDecode) {
        decodeQRFromFrame(fbForDecode);
      }
    }
    if(res == ESP_OK){
      size_t hlen = snprintf(part_buf, sizeof(part_buf), _STREAM_PART, _jpg_buf_len);
      res = httpd_resp_send_chunk(req, part_buf, hlen);
    }
    if(res == ESP_OK){
      res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    }
    if(res == ESP_OK){
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    }
    if(fb){
      esp_camera_fb_return(fb);
      fb = NULL;
      _jpg_buf = NULL;
    } else if(_jpg_buf){
      free(_jpg_buf);
      _jpg_buf = NULL;
    }
    if(res != ESP_OK){
      break;
    }
  }
  /* ---------------------------------------- */
  // Client disconnected (or a send failed) - clearing ws_run is all it takes
  // to hand the camera back: the QR-reader task is parked on this flag, not
  // deleted, so it picks up on its next 100ms tick.
  ws_run = false;
  streamActive = false;
  return res;
}
/* ________________________________________________________________________________ */

/* ________________________________________________________________________________ cmd handler function to be called during GET or uri request. */
static esp_err_t cmd_handler(httpd_req_t *req){
  char*  buf;
  size_t buf_len;
  char variable[32] = {0,};
   
  buf_len = httpd_req_get_url_query_len(req) + 1;
  if (buf_len > 1) {
    buf = (char*)malloc(buf_len);
    if(!buf){
      httpd_resp_send_500(req);
      return ESP_FAIL;
    }
    if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
      if (httpd_query_key_value(buf, "go", variable, sizeof(variable)) == ESP_OK) {
      } else {
        free(buf);
        httpd_resp_send_404(req);
        return ESP_FAIL;
      }
    } else {
      free(buf);
      httpd_resp_send_404(req);
      return ESP_FAIL;
    }
    free(buf);
  } else {
    httpd_resp_send_404(req);
    return ESP_FAIL;
  }
 
  int res = 0;

  Serial.print("Incoming command : ");
  Serial.println(variable);
  Serial.println();
  String getData = String(variable);
  String resultData = getValue(getData, ',', 0);

  /* ---------------------------------------- Controlling the LEDs on the ESP32 Cam board with PWM. */
  // Example :
  // Incoming command = S,10
  // S = Slider
  // 10 = slider value
  // I set the slider value range from 0 to 20.
  // Then the slider value is changed from 0 - 20 or vice versa to 0 - 255 or vice versa.
  if (resultData == "S") {
    resultData = getValue(getData, ',', 1);
    int pwm = map(resultData.toInt(), 0, 20, 0, 255);
    ledcAttach(4, 5000, 8);   // pin, frequency, resolution
    ledcWrite(4, pwm);        // use pin number directly, not channel
  }
  /* ---------------------------------------- */

  /* ---------------------------------------- Clean the result of reading the QR Code. */
  // Incoming Command = B,clr
  // B = Button
  // clr = Command to clean the results of reading the QR Code.
  if (resultData == "B") {
    resultData = getValue(getData, ',', 1);
    if (resultData == "clr") {
      QRCodeResult = "";
    }
  }
  /* ---------------------------------------- */
  
  if(res){
    return httpd_resp_send_500(req);
  }
 
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, NULL, 0);
}
/* ________________________________________________________________________________ */

/* ________________________________________________________________________________ Reads the SPST gate switch. Shared by the on-page poll and the MQTT report. */
String readGateStatus() {
  // Switch OFF (open, pin pulled HIGH)        -> GATE 1
  // Switch ON  (closed to GND, pin reads LOW) -> GATE 2
  return (digitalRead(GATE_SWITCH_PIN) == LOW) ? "Gate_2" : "Gate_1";
}
/* ________________________________________________________________________________ */

/* ________________________________________________________________________________ status handler: returns "<qr result>|<category>|<item name>|<gate status>|<ip>"
   in one response. Replaces the old /getqrcodeval + /getgateval pair so the
   page's 500ms poll only costs one HTTP round trip instead of two. Category
   and item name are looked up from ITEM_TABLE each poll, so they always
   track whatever QRCodeResultSend currently holds (including going blank
   right after a "Clear Result" click). The IP is read fresh each poll too,
   so the page shows the current address even if it changes after a
   Wi-Fi reconnect. */
static esp_err_t status_handler(httpd_req_t *req){
  if (QRCodeResult != "Decoding FAILED") QRCodeResultSend = QRCodeResult;
  gateStatus = readGateStatus();

  String itemName = "";
  String itemCategory = "";
  if (QRCodeResultSend.length() > 0) {
    lookupItem(QRCodeResultSend, itemName, itemCategory);
    // lookupItem() leaves itemName/itemCategory as "" on no match, so an
    // unrecognized QR code just shows blank Category/Item boxes rather than
    // a stale value from a previous scan.
  }

  String combined = QRCodeResultSend + "|" + itemCategory + "|" + itemName + "|" + gateStatus
                   + "|" + WiFi.localIP().toString();
  httpd_resp_send(req, combined.c_str(), HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}
/* ________________________________________________________________________________ */

/* ________________________________________________________________________________ Subroutine for starting the web server / startCameraServer. */
void startCameraWebServer(){
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;

  // The unset fields below (is_websocket, handle_ws_control_frames,
  // supported_subprotocol) already default to false/NULL for a struct
  // initializer - no need to state them explicitly on every handler.
  httpd_uri_t index_uri = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = index_handler,
    .user_ctx  = NULL
  };

  httpd_uri_t cmd_uri = {
    .uri       = "/action",
    .method    = HTTP_GET,
    .handler   = cmd_handler,
    .user_ctx  = NULL
  };

  httpd_uri_t status_uri = {
    .uri       = "/status",
    .method    = HTTP_GET,
    .handler   = status_handler,
    .user_ctx  = NULL
  };

  httpd_uri_t stream_uri = {
    .uri       = "/stream",
    .method    = HTTP_GET,
    .handler   = stream_handler,
    .user_ctx  = NULL
  };

  if (httpd_start(&index_httpd, &config) == ESP_OK) {
      httpd_register_uri_handler(index_httpd, &index_uri);
      httpd_register_uri_handler(index_httpd, &cmd_uri);
      httpd_register_uri_handler(index_httpd, &status_uri);
  }

  config.server_port += 1;
  config.ctrl_port += 1;
  // Without these, a client that vanishes without closing cleanly (phone
  // locked, Wi-Fi dropped, tab killed) leaves stream_handler blocked in
  // httpd_resp_send_chunk() forever, and every later request - including the
  // one from your refreshed page - waits behind it. 5s is plenty for a LAN.
  config.send_wait_timeout = 5;
  config.recv_wait_timeout = 5;
  config.lru_purge_enable = true;
  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
      httpd_register_uri_handler(stream_httpd, &stream_uri);
  }

  Serial.println();
  Serial.println("Camera Server started successfully");
  Serial.print("Camera Stream Ready! Go to: http://");
  Serial.println(WiFi.localIP());
  Serial.println();
}
/* ________________________________________________________________________________ */

/* ________________________________________________________________________________ Connect / reconnect to Wi-Fi via WiFiManager setup portal (restarts the board if the portal itself times out with no configuration). */
void connectWiFi() {
  WiFi.mode(WIFI_STA);
  Serial.println("------------");

  // Load previously-saved MQTT host (if any) before showing the portal,
  // so the field is pre-filled with the last value you entered rather
  // than always resetting to the hardcoded default.
  prefs.begin("cfg", true);  // read-only
  mqttHost = prefs.getString("mqtt_host", mqttHost);
  prefs.end();

  WiFiManager wm;
  wm.setConfigPortalTimeout(PORTAL_TIMEOUT_SECONDS);
  wm.setConnectTimeout(CONFIG_TIMEOUT_SECONDS);
  wm.setSaveConfigCallback(onWiFiManagerSave);

  // Extra field on the Wi-Fi setup page for the MQTT broker IP, same as
  // typing in a Wi-Fi password - pre-filled with the current value.
  WiFiManagerParameter customMqttHost("mqtt_host", "MQTT Broker IP", mqttHost.c_str(), 40);
  wm.addParameter(&customMqttHost);

  String hotspotSsid = buildHotspotSsid();
  bool connected = wm.autoConnect(hotspotSsid.c_str(), HOTSPOT_PASSWORD);

  if (!connected) {
    Serial.println("Failed to connect and portal timed out - restarting.");
    delay(1000);
    ESP.restart();
  }

  // Only persist a new MQTT host if the portal form was actually
  // submitted this time (shouldSaveMqttHost) - a silent reconnect to an
  // already-saved Wi-Fi network never touches this field, so mqttHost
  // stays whatever was loaded above.
  if (shouldSaveMqttHost) {
    mqttHost = String(customMqttHost.getValue());
    prefs.begin("cfg", false);  // read-write
    prefs.putString("mqtt_host", mqttHost);
    prefs.end();
    shouldSaveMqttHost = false;
  }
  Serial.println("MQTT broker IP: " + mqttHost);

  digitalWrite(LED_OnBoard, LOW);
  Serial.println("");
  Serial.println("WiFi connected: " + WiFi.SSID());
  Serial.println("------------");
  Serial.println("");

  if (MDNS.begin("esp32camqr")) {
  Serial.println("mDNS responder started: http://esp32camqr.local");
  } else {
    Serial.println("mDNS responder failed to start");
  }
  MDNS.addService("http", "tcp", 80);
  /* ::::::::::::::::: */
}
/* ________________________________________________________________________________ */

/* ________________________________________________________________________________ Non-blocking MQTT reconnect + keepalive; called every loop() pass. */
void serviceMqtt() {
  if (!mqtt.connected()) {
    unsigned long now = millis();
    if (now - lastMqttAttempt >= mqttRetryInterval) {
      lastMqttAttempt = now;
      String clientId = "agv-" + String(ROBOT_ID) + "-esp32cam";
      Serial.print("Connecting to MQTT broker...");
      if (mqtt.connect(clientId.c_str())) {
        Serial.println(" connected");
        reportQrIp();  // let the broker know which IP to reach this camera at
      } else {
        Serial.print(" failed, rc=");
        Serial.print(mqtt.state());
        Serial.println(", will retry");
      }
    }
    return; // not connected yet - nothing else to service this pass
  }
  mqtt.loop();
}
/* ________________________________________________________________________________ */

/* ________________________________________________________________________________ Publishes {"serial_code","pickup_location"} for one decoded QR + gate status. */
bool reportInbound(const char* qrPayload, const char* gateStatusStr) {
  // Item codes and gate labels here are plain text; escape quotes/backslashes
  // first if you ever allow QR payloads that could contain them.
  String payload = "{\"serial_code\":\"" + String(qrPayload)
                 + "\",\"pickup_location\":\"" + String(gateStatusStr)
                 + "\"}";

  bool sent = mqtt.publish(INBOUND_TOPIC, payload.c_str());
  Serial.print("Inbound MQTT payload: ");
  Serial.println(payload);
  Serial.println(sent ? "Published" : "Publish failed");
  return sent;
}
/* ________________________________________________________________________________ */

/* ________________________________________________________________________________ Publishes this device's current IP address to QRIP_TOPIC, e.g.
   {"qrip":"192.168.1.42"}. Called once right after every successful MQTT
   (re)connect (see serviceMqtt()) - not on a timer - since the IP only
   needs to reach the broker once per session, and re-sending on each
   reconnect also covers the rare case where DHCP hands out a new address.
   Two bugs from the original draft, fixed here:
     - WiFi.localIP() is an IPAddress, not something String() can wrap
       directly - use .toString() to get a String out of it.
     - the payload variable being built was named "payload", but the
       publish() call referenced an undefined "qrip_payload" - publish
       whatever variable you actually built the JSON into. */
bool reportQrIp() {
  String payload = "{\"qrip\":\"" + WiFi.localIP().toString() + "\"}";

  bool sent = mqtt.publish(QRIP_TOPIC, payload.c_str());
  Serial.print("QR IP MQTT payload: ");
  Serial.println(payload);
  Serial.println(sent ? "Published" : "Publish failed");
  return sent;
}
/* ________________________________________________________________________________ */

/* ________________________________________________________________________________ Called from dumpData() on every successful decode. De-dupes against the
   last payload actually published, and snapshots the gate switch at the
   same instant the QR was read. */
void queueMqttReport(const String &qrPayload) {
  String gate = readGateStatus();

  if (xSemaphoreTake(qrDataMutex, portMAX_DELAY) == pdTRUE) {
    if (qrPayload != lastSentQR) {
      pendingQR = qrPayload;
      pendingGate = gate;
      qrPendingSend = true;
    }
    xSemaphoreGive(qrDataMutex);
  }
}
/* ________________________________________________________________________________ */

/* ________________________________________________________________________________ Called every loop() pass: sends a queued QR+gate report, if any. */
void serviceMqttReport() {
  if (!mqtt.connected()) return; // stays queued until the link is back

  String qrToSend, gateToSend;
  bool doSend = false;

  if (xSemaphoreTake(qrDataMutex, portMAX_DELAY) == pdTRUE) {
    if (qrPendingSend) {
      qrToSend = pendingQR;
      gateToSend = pendingGate;
      doSend = true;
    }
    xSemaphoreGive(qrDataMutex);
  }

  if (doSend) {
    if (reportInbound(qrToSend.c_str(), gateToSend.c_str())) {
      // Only mark as sent once publish succeeds, and only then update
      // lastSentQR - so a dropped publish gets retried automatically the
      // next time the same code is decoded again.
      if (xSemaphoreTake(qrDataMutex, portMAX_DELAY) == pdTRUE) {
        lastSentQR = qrToSend;
        qrPendingSend = false;
        xSemaphoreGive(qrDataMutex);
      }
    }
  }
}
/* ________________________________________________________________________________ */

/* ________________________________________________________________________________ VOID SETUP() */
void setup() {
  // put your setup code here, to run once:

  // Disable brownout detector.
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  /* ---------------------------------------- Init serial communication speed (baud rate). */
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();
  /* ---------------------------------------- */

  pinMode(LED_OnBoard, OUTPUT);

  /* ---------------------------------------- Gate select switch. */
  // Internal pull-up enabled: pin reads HIGH when the switch is open (OFF),
  // and reads LOW when the switch closes the circuit to GND (ON).
  pinMode(GATE_SWITCH_PIN, INPUT_PULLUP);
  /* ---------------------------------------- */

  /* ---------------------------------------- Camera configuration. */
  Serial.println("Start configuring and initializing the camera...");
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_GRAYSCALE;
  config.frame_size = FRAMESIZE_QVGA;
  config.jpeg_quality = 15;
  if (psramFound()) {
    config.fb_count = 2;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.grab_mode = CAMERA_GRAB_LATEST;
  } else {
    config.fb_count = 1;
  }

  // Pulse PWDN before init: on a cold/brownout boot the OV3660 can come up
  // in a state the SCCB probe misses on the first try, which is the usual
  // culprit behind "camera sometimes fails to load". A few retries with a
  // power-down pulse between them is cheap insurance; only restart the
  // board if it still won't come up.
  esp_err_t err = ESP_FAIL;
  for (int attempt = 1; attempt <= 3 && err != ESP_OK; attempt++) {
    if (attempt > 1 && PWDN_GPIO_NUM != -1) {
      pinMode(PWDN_GPIO_NUM, OUTPUT);
      digitalWrite(PWDN_GPIO_NUM, HIGH);
      delay(50);
      digitalWrite(PWDN_GPIO_NUM, LOW);
      delay(50);
    }
    err = esp_camera_init(&config);
    if (err != ESP_OK) {
      Serial.printf("Camera init attempt %d failed with error 0x%x\n", attempt, err);
      esp_camera_deinit();
      delay(200);
    }
  }
  if (err != ESP_OK) {
    Serial.printf("Camera init failed after retries (0x%x) - restarting.\n", err);
    ESP.restart();
  }
  
  sensor_t * s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_QVGA);
  s->set_vflip(s, 1);     // flip vertically (fixes upside-down)
  s->set_hmirror(s, 0);   // mirror horizontally (fixes left-right if needed)
  
  Serial.println("Configure and initialize the camera successfully.");
  Serial.println();
  /* ---------------------------------------- */

  /* ---------------------------------------- Connect to Wi-Fi. */
  connectWiFi();
  /* ---------------------------------------- */

  /* ---------------------------------------- MQTT setup. */
  qrDataMutex = xSemaphoreCreateMutex();
  decodeMutex = xSemaphoreCreateMutex();
  if (qrDataMutex == NULL || decodeMutex == NULL) {
    Serial.println("Failed to create mutexes - restarting.");
    ESP.restart();
  }
  mqtt.setServer(mqttHost.c_str(), MQTT_PORT);
  // Connection itself is handled non-blocking from loop() via serviceMqtt(),
  // so setup() doesn't stall waiting on the broker.
  /* ---------------------------------------- */

  // Start camera web server
  startCameraWebServer(); 
  
  // Calls the createTaskQRCodeReader() subroutine.
  createTaskQRCodeReader();
}
/* ________________________________________________________________________________ */

void loop() {
  // put your main code here, to run repeatedly:

  /* ---------------------------------------- Wi-Fi / MQTT link maintenance. */
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi(); // blocking short retry w/ restart-on-timeout, same as setup()
  } else {
    serviceMqtt();        // non-blocking MQTT reconnect + keepalive
    serviceMqttReport();  // publish a decoded QR + gate status, if one is queued
  }
  /* ---------------------------------------- */

  // Handing the camera back to QRCodeReader_Task when the stream ends is now
  // done directly at the bottom of stream_handler(), the instant its loop
  // exits - no need to poll for it here anymore.
}
/* ________________________________________________________________________________ */

/* ________________________________________________________________________________ create "QRCodeReader_Task" using the xTaskCreatePinnedToCore() function */
void createTaskQRCodeReader() {
  xTaskCreatePinnedToCore(
             QRCodeReader,          /* Task function. */
             "QRCodeReader_Task",   /* name of task. */
             10000,                 /* Stack size of task */
             NULL,                  /* parameter of the task */
             1,                     /* priority of the task */
             &QRCodeReader_Task,    /* Task handle to keep track of created task */
             0);                    /* pin task to core 0 */
}
/* ________________________________________________________________________________ */

/* ________________________________________________________________________________ QRCodeReader() */
void QRCodeReader( void * pvParameters ){
  Serial.print("QRCodeReader running on core ");
  Serial.println(xPortGetCoreID());

  // Runs for the lifetime of the board. While a browser is streaming
  // (ws_run == true) it parks instead of touching the camera - stream_handler
  // does the decoding during that time. Nothing deletes or recreates this
  // task, so no camera call or quirc allocation is ever killed half-finished.
  for (;;) {
      if (ws_run) {
        vTaskDelay(pdMS_TO_TICKS(100));
        continue;
      }

      camera_fb_t * fb = esp_camera_fb_get();
      if (!fb) {
        Serial.println("Camera capture failed (QRCodeReader())");
        vTaskDelay(pdMS_TO_TICKS(100)); // back off; the old bare continue spun
                                        // flat out and starved the web server
        continue;
      }

      decodeQRFromFrame(fb);

      // fb->buf is fully copied into quirc's own buffer inside
      // decodeQRFromFrame() - return it to the driver now rather than
      // holding it through decode (per project convention: return the
      // frame buffer before inference).
      esp_camera_fb_return(fb);

      vTaskDelay(pdMS_TO_TICKS(10)); // yield so the HTTP tasks get a slot
  }
}
/* ________________________________________________________________________________ */

/* ________________________________________________________________________________ Function to display the results of reading the QR Code on the serial monitor. */
void dumpData(const struct quirc_data *data)
{
  Serial.printf("-Version: %d\n", data->version);
  Serial.printf("-ECC level: %c\n", "MLHQ"[data->ecc_level]);
  Serial.printf("-Mask: %d\n", data->mask);
  Serial.printf("-Length: %d\n", data->payload_len);
  Serial.printf("-Payload: %s\n", data->payload);
  
  QRCodeResult = (const char *)data->payload;

  queueMqttReport(QRCodeResult);
}
/* ________________________________________________________________________________ */


String getValue(String data, char separator, int index) {
  int found = 0;
  int strIndex[] = { 0, -1 };
  int maxIndex = data.length() - 1;
  
  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

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
#include "esp_http_server.h"
#include <PubSubClient.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
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
#define LED_Green   12
#define LED_Blue    13

// Gate select switch (SPST) GPIO
// One leg to GPIO13, other leg to GND.
// Uses internal pull-up: switch OFF (open) -> HIGH -> "GATE 1"
//                         switch ON  (closed to GND) -> LOW -> "GATE 2"
#define GATE_SWITCH_PIN 13

// creating a task handle
TaskHandle_t QRCodeReader_Task;

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

/* ======================================== */
bool ws_run = false;
int wsLive_val = 0;
int last_wsLive_val;
byte get_wsLive_interval = 0;
bool get_wsLive_val = true;
/* ======================================== */

/* ======================================== Variables for millis() */
unsigned long previousMillis = 0;
const long interval = 1000;
/* ======================================== */

/* ======================================== Replace with your network credentials */
// const char* ssid = "cam";
// const char* password = "screenshare123";
const char* ssid = "Aml Office 4th";
const char* password = "Aml@2026";

/* ======================================== */

/* ======================================== MQTT (inbound reporting) */
const char* MQTT_HOST = "192.168.110.21";  // IP address of the Mosquitto server
const uint16_t MQTT_PORT = 1883;

const char* ROBOT_ID = "R1";
const char* INBOUND_TOPIC = "agv/R1/inbound";

WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

unsigned long lastMqttAttempt = 0;
const unsigned long mqttRetryInterval = 5000; // ms between reconnect attempts
/* ======================================== */

/* ======================================== Shared state between the QR-decode context
   (QRCodeReader_Task on core 0, or the stream_handler HTTP task) and the
   MQTT-publish context (Arduino loop(), core 1). Access is mutex-protected. */
SemaphoreHandle_t qrDataMutex = NULL;
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
    
      body { font-family: Arial; text-align: center; margin:0px auto; padding-top: 10px;}

      /* ----------------------------------- Slider */
      .slidecontainer {
        width: 100%;
      }

      .slider {
        -webkit-appearance: none;
        width: 50%;
        height: 10px;
        border-radius: 5px;
        background: #d3d3d3;
        outline: none;
        opacity: 0.7;
        -webkit-transition: .2s;
        transition: opacity .2s;
      }

      .slider:hover {
        opacity: 1;
      }

      .slider::-webkit-slider-thumb {
        -webkit-appearance: none;
        appearance: none;
        width: 20px;
        height: 20px;
        border-radius: 50%;
        background: #04AA6D;
        cursor: pointer;
      }

      .slider::-moz-range-thumb {
        width: 20px;
        height: 20px;
        border-radius: 50%;
        background: #04AA6D;
        cursor: pointer;
      }
      /* ----------------------------------- */

      /* ----------------------------------- Stream Viewer */
      img {
        width: auto ;
        max-width: 100% ;
        height: auto ; 
      }
      /* ----------------------------------- */
      
    </style>
    
  </head>
  
  <body>
  
    <h3>QR Code Reader with ESP32CAM</h3>
    
    <img src="" id="vdstream">
    
    <br><br>
    
    <div class="slidecontainer">
      <span style="font-size:15;">LED Flash &nbsp;</span>
      <input type="range" min="0" max="20" value="0" class="slider" id="mySlider">
    </div>

    <br>

    <p>QR Code Scan Result :</p>
    <div style="padding: 5px; border: 3px solid #075264; text-align: center; width: 70%; margin: auto; color:#0A758F;" id="showqrcodeval"></div>

    <br>

    <p>Gate Status :</p>
    <div style="padding: 5px; border: 3px solid #075264; text-align: center; width: 70%; margin: auto; color:#0A758F; font-weight: bold;" id="showgateval"></div>

    <br>

    <button type="button" onclick="CopyQRCodeRslt()">Copy Result</button>
    <button type="button" onclick="send_btn_cmd('clr')">Clear Result</button>
    
    <script>
      /* ----------------------------------- Calls the video stream link and displays it */ 
      window.onload = document.getElementById("vdstream").src = window.location.href.slice(0, -1) + ":81/stream";
      /* ----------------------------------- */
      
      var slider = document.getElementById("mySlider");
      
      /* ----------------------------------- Variable declaration and timer to display QR Code reading results. */
      var myTmr;
      let qrcodeval = "...";
      let gateval = "...";
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

      /* :::::::::::::::::::::::::::::::::::::::::::::::: Timer to get QR Code reading result and display it. */
      function myTimer() {
        getQRCodeVal();
        textQRCodeVal();
        getGateVal();
        textGateVal();
      }
      /* :::::::::::::::::::::::::::::::::::::::::::::::: */

      /* :::::::::::::::::::::::::::::::::::::::::::::::: Function to display the results of reading the QR Code. */
      function textQRCodeVal() {
        document.getElementById("showqrcodeval").innerHTML = qrcodeval;
      }
      /* :::::::::::::::::::::::::::::::::::::::::::::::: */

      /* :::::::::::::::::::::::::::::::::::::::::::::::: Function to display the gate status. */
      function textGateVal() {
        document.getElementById("showgateval").innerHTML = gateval;
      }
      /* :::::::::::::::::::::::::::::::::::::::::::::::: */

      /* :::::::::::::::::::::::::::::::::::::::::::::::: Function to send commands to the ESP32 Cam whenever the button is clicked. */
      function send_btn_cmd(cmds) {
        let btn_cmd = "B," + cmds;
        send_cmd(btn_cmd);
      }
   
      function CopyQRCodeRslt() {
        // Create new element
        var el = document.createElement('textarea');
        // Set value (string to be copied)
        el.value = qrcodeval;
        // Set non-editable to avoid focus and move outside of view
        el.setAttribute('readonly', '');
        el.style = {position: 'absolute', left: '-9999px'};
        document.body.appendChild(el);
        // Select text inside element
        el.select();
        // Copy text to clipboard
        document.execCommand('copy');
        // Remove temporary element
        document.body.removeChild(el);
        /* Alert the copied text */
        alert("The result of reading the QR Code has been copied.");
      }
      /* :::::::::::::::::::::::::::::::::::::::::::::::: */

      /* :::::::::::::::::::::::::::::::::::::::::::::::: Function to get QR Code reading results. */
      function getQRCodeVal() {
        var xhttp = new XMLHttpRequest();
        xhttp.onreadystatechange = function() {
          if (this.readyState == 4 && this.status == 200) {
            qrcodeval = this.responseText;
          }
        };
        xhttp.open("GET", "/getqrcodeval", true);
        xhttp.send();
      }
      /* :::::::::::::::::::::::::::::::::::::::::::::::: */

      /* :::::::::::::::::::::::::::::::::::::::::::::::: Function to get the gate switch status. */
      function getGateVal() {
        var xhttp2 = new XMLHttpRequest();
        xhttp2.onreadystatechange = function() {
          if (this.readyState == 4 && this.status == 200) {
            gateval = this.responseText;
          }
        };
        xhttp2.open("GET", "/getgateval", true);
        xhttp2.send();
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

/* ________________________________________________________________________________ stream handler function to be called during GET or uri request. */
static esp_err_t stream_handler(httpd_req_t *req){
  ws_run = true;
  vTaskDelete(QRCodeReader_Task);
  Serial.print("stream_handler running on core ");
  Serial.println(xPortGetCoreID());

  camera_fb_t * fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t * _jpg_buf = NULL;
  char * part_buf[64];
  uint32_t frame_counter = 0;
  const uint32_t DECODE_EVERY_N_FRAMES = 3; // scan for QR on 1 out of every N streamed frames

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if(res != ESP_OK){
    return res;
  }

  /* ---------------------------------------- Loop to show streaming video from ESP32 Cam camera and read QR Code. */
  while(true){
    ws_run = true;
    bool tryDecode = false;

    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed (stream_handler)");
      res = ESP_FAIL;
    } else {
      frame_counter++;
      tryDecode = (frame_counter % DECODE_EVERY_N_FRAMES == 0);

      if (tryDecode) {
        q = quirc_new();
        if (q == NULL){
          Serial.print("can't create quirc object\r\n");
          tryDecode = false;
        } else {
          quirc_resize(q, fb->width, fb->height);
          image = quirc_begin(q, NULL, NULL);
          memcpy(image, fb->buf, fb->len);
          quirc_end(q);
          image = NULL;
          // fb->buf is now fully copied into quirc's own buffer, so the
          // actual quirc_count/extract/decode work below doesn't need fb
          // anymore - it's deferred until after fb has been returned/used
          // for the JPEG frame, so the camera driver gets its buffer back
          // sooner (per project convention: return fb before inference).
        }
      }

      if(fb->width > 200){
        if(fb->format != PIXFORMAT_JPEG){
          bool jpeg_converted = frame2jpg(fb, 24, &_jpg_buf, &_jpg_buf_len);
          esp_camera_fb_return(fb);
          fb = NULL;
          if(!jpeg_converted){
            Serial.println("JPEG compression failed");
            res = ESP_FAIL;
          }
        } else {
          _jpg_buf_len = fb->len;
          _jpg_buf = fb->buf;
        }
      }

      if (tryDecode) {
        int count = quirc_count(q);
        if (count > 0) {
          quirc_extract(q, 0, &code);
          err = quirc_decode(&code, &data);

          if (err){
            QRCodeResult = "Decoding FAILED";
            Serial.println(QRCodeResult);
          } else {
            Serial.printf("Decoding successful:\n");
            dumpData(&data);
          } 
          Serial.println();
        } 
        quirc_destroy(q);
      }
    }
    if(res == ESP_OK){
      size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
      res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
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
    
    wsLive_val++;
    if (wsLive_val > 999) wsLive_val = 0;
  }
  /* ---------------------------------------- */
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

/* ________________________________________________________________________________ qrcoderslt handler function to be called during GET or uri request. */
static esp_err_t qrcoderslt_handler(httpd_req_t *req){
  if (QRCodeResult != "Decoding FAILED") QRCodeResultSend = QRCodeResult;
  httpd_resp_send(req, QRCodeResultSend.c_str(), HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}
/* ________________________________________________________________________________ */

/* ________________________________________________________________________________ Reads the SPST gate switch. Shared by the on-page poll and the MQTT report. */
String readGateStatus() {
  // Switch OFF (open, pin pulled HIGH)        -> GATE 1
  // Switch ON  (closed to GND, pin reads LOW) -> GATE 2
  return (digitalRead(GATE_SWITCH_PIN) == LOW) ? "Gate_2" : "Gate_1";
}
/* ________________________________________________________________________________ */

/* ________________________________________________________________________________ gate handler function to be called during GET or uri request. */
static esp_err_t gate_handler(httpd_req_t *req){
  gateStatus = readGateStatus();
  httpd_resp_send(req, gateStatus.c_str(), HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}
/* ________________________________________________________________________________ */

/* ________________________________________________________________________________ Subroutine for starting the web server / startCameraServer. */
void startCameraWebServer(){
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;

  httpd_uri_t index_uri = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = index_handler,
    .user_ctx  = NULL,
    .is_websocket = false,
    .handle_ws_control_frames = false,
    .supported_subprotocol = NULL
  };

  httpd_uri_t cmd_uri = {
    .uri       = "/action",
    .method    = HTTP_GET,
    .handler   = cmd_handler,
    .user_ctx  = NULL,
    .is_websocket = false,
    .handle_ws_control_frames = false,
    .supported_subprotocol = NULL
  };

  httpd_uri_t qrcoderslt_uri = {
    .uri       = "/getqrcodeval",
    .method    = HTTP_GET,
    .handler   = qrcoderslt_handler,
    .user_ctx  = NULL,
    .is_websocket = false,
    .handle_ws_control_frames = false,
    .supported_subprotocol = NULL
  };

  httpd_uri_t stream_uri = {
    .uri       = "/stream",
    .method    = HTTP_GET,
    .handler   = stream_handler,
    .user_ctx  = NULL,
    .is_websocket = false,
    .handle_ws_control_frames = false,
    .supported_subprotocol = NULL
  };

  httpd_uri_t gate_uri = {
    .uri       = "/getgateval",
    .method    = HTTP_GET,
    .handler   = gate_handler,
    .user_ctx  = NULL,
    .is_websocket = false,
    .handle_ws_control_frames = false,
    .supported_subprotocol = NULL
  };
    
  if (httpd_start(&index_httpd, &config) == ESP_OK) {
      httpd_register_uri_handler(index_httpd, &index_uri);
      httpd_register_uri_handler(index_httpd, &cmd_uri);
      httpd_register_uri_handler(index_httpd, &qrcoderslt_uri);
      httpd_register_uri_handler(index_httpd, &gate_uri);
  }

  config.server_port += 1;
  config.ctrl_port += 1;  
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

/* ________________________________________________________________________________ Connect / reconnect to Wi-Fi (blocking, restarts the board on a long timeout). */
void connectWiFi() {
  WiFi.mode(WIFI_STA);
  Serial.println("------------");
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  /* ::::::::::::::::: The process of connecting ESP32 CAM with WiFi Hotspot / WiFi Router. */
  /*
   * The process timeout of connecting ESP32 CAM with WiFi Hotspot / WiFi Router is 20 seconds.
   * If within 20 seconds the ESP32 CAM has not been successfully connected to WiFi, the ESP32 CAM will restart.
   */
  int connecting_process_timed_out = 20; //--> 20 = 20 seconds.
  connecting_process_timed_out = connecting_process_timed_out * 2;
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(250);
    digitalWrite(LED_OnBoard, LOW);
    delay(250);
    if(connecting_process_timed_out > 0) connecting_process_timed_out--;
    if(connecting_process_timed_out == 0) {
      delay(1000);
      ESP.restart();
    }
  }
  digitalWrite(LED_OnBoard, LOW);
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("------------");
  Serial.println("");
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

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
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
  if (qrDataMutex == NULL) {
    Serial.println("Failed to create qrDataMutex - restarting.");
    ESP.restart();
  }
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
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

  unsigned long currentMillis = millis();

  /* ---------------------------------------- Wi-Fi / MQTT link maintenance. */
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi(); // blocking short retry w/ restart-on-timeout, same as setup()
  } else {
    serviceMqtt();        // non-blocking MQTT reconnect + keepalive
    serviceMqttReport();  // publish a decoded QR + gate status, if one is queued
  }
  /* ---------------------------------------- */

  /* ---------------------------------------- Condition to check whether the index page is being accessed or not. */
  // If the index page is not accessed, the loop will call the "QRCodeReader()" Subroutine.
  // If the index page is accessed, the Loop will not call the "QRCodeReader()" Subroutine.
  // I created this condition because the camera cannot be accessed at the same time by the index page and the "QRCodeReader()" subroutine.
  // Meanwhile, I designed this project to be able to read the QR Code when the index page is accessed or the index page is not being accessed.
  // Even if I have to share camera data, I haven't found the best way to share camera data to the "QRCodeReader()" sub routine and stream function.
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    if (ws_run == true) {
      if (get_wsLive_val == true) {
        last_wsLive_val = wsLive_val;
        get_wsLive_val = false;
      }
   
      get_wsLive_interval++;
      if (get_wsLive_interval > 2) {
        get_wsLive_interval = 0;
        get_wsLive_val = true;
        if (wsLive_val == last_wsLive_val) {
          ws_run = false;
          last_wsLive_val = 0;
          createTaskQRCodeReader();
        }
      }
    }
  }
  /* ---------------------------------------- */
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

  while(!ws_run){
      camera_fb_t * fb = NULL;
      q = quirc_new();
      
      if (q == NULL){
        Serial.print("can't create quirc object\r\n");  
        continue;
      }
      
      fb = esp_camera_fb_get();
      if (!fb)
      {
        Serial.println("Camera capture failed (QRCodeReader())");
        quirc_destroy(q);
        continue;
      }
      
      quirc_resize(q, fb->width, fb->height);
      image = quirc_begin(q, NULL, NULL);
      memcpy(image, fb->buf, fb->len);
      quirc_end(q);

      // fb->buf is fully copied into quirc's own buffer above - return it to
      // the driver now rather than holding it through the decode below
      // (per project convention: return the frame buffer before inference).
      esp_camera_fb_return(fb);
      fb = NULL;
      image = NULL;

      int count = quirc_count(q);
      if (count > 0) {
        //Serial.println(count);
        quirc_extract(q, 0, &code);
        err = quirc_decode(&code, &data);
    
        if (err){
          QRCodeResult = "Decoding FAILED";
          Serial.println(QRCodeResult);
        } else {
          Serial.printf("Decoding successful:\n");
          dumpData(&data);
        } 
        Serial.println();
      } 

      quirc_destroy(q);
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

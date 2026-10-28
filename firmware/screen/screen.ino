#include <WiFi.h>
#include <HTTPClient.h>
#include <TFT_eSPI.h> 
#include <TJpg_Decoder.h>
#include "qrcode.h"

// --- TOUCH LIBRARIES ---
#include <XPT2046_Touchscreen.h>
#include <SPI.h>

// ================= CONFIGURATION =================
const char* WIFI_SSID     = "CHANGE_ME_SSID";
const char* WIFI_PASSWORD = "CHANGE_ME_PASS";

// IMPORTANT: Point to the SNAPSHOT endpoint, not the stream
// This ensures the server sends Content-Length headers required by the CYD
const char* SNAPSHOT_URL  = "http://192.168.4.1:8080/snapshot/cyd"; 

// SERIAL COMMS TO PI
#define RX_PIN 22 
#define TX_PIN 27 

// --- CYD TOUCH PINS ---
#define XPT_CS   33
#define XPT_IRQ  36
#define XPT_MOSI 32
#define XPT_MISO 39
#define XPT_CLK  25

// ================= GLOBALS =================
SPIClass mySpi(VSPI);
XPT2046_Touchscreen ts(XPT_CS, XPT_IRQ);
TFT_eSPI tft = TFT_eSPI();
struct QRCode qrcode;
#define QR_VERSION 12 

// State Management
enum State { MENU, SHOW_QR, SHOW_STREAM };
State currentState = MENU;

// Serial Parsing
String inputString = "";
bool stringComplete = false;

// Optimization: Re-use the HTTP Client to keep TCP connection alive
HTTPClient http; 

// ================= CALLBACKS =================

// JpgDecoder callback to render to screen
bool tft_output(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t* bitmap) {
  if (y >= tft.height()) return 0;
  tft.pushImage(x, y, w, h, bitmap);
  return 1;
}

// ================= SETUP =================
void setup() {
  // 1. Backlight On
  pinMode(21, OUTPUT); 
  digitalWrite(21, HIGH);

  // 2. Serial Init
  Serial.begin(115200); 
  Serial2.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);

  // 3. Touch Init
  mySpi.begin(XPT_CLK, XPT_MISO, XPT_MOSI, XPT_CS);
  if (!ts.begin(mySpi)) { 
    Serial.println("Touch Controller Failed!"); 
  } else {
    Serial.println("Touch Controller Started."); 
    ts.setRotation(1); 
  }

  // 4. Screen Init
  tft.init();
  tft.setRotation(1); 
  tft.fillScreen(TFT_BLACK);
  tft.setSwapBytes(true); 

  // 5. Decoder Init
  TJpgDec.setJpgScale(1);
  TJpgDec.setCallback(tft_output);

  // 6. WiFi Init
  setupWiFi(); 
  
  // 7. Start Menu
  drawMenu(); 
}

// ================= LOOP =================
void loop() {
  // 1. Check Serial (Commands from Pi)
  while (Serial2.available()) {
    char inChar = (char)Serial2.read();
    if (inChar == '\n') {
      stringComplete = true; 
    } else {
      inputString += inChar;
    }
  }

  if (stringComplete) {
    processIncoming(inputString);
    inputString = "";
    stringComplete = false;
  }

  // 2. Handle Touch Input
  handleTouch();

  // 3. Render Stream (If Active)
  if (currentState == SHOW_STREAM) {
    drawCameraFrame();
    // A tiny delay yields CPU to WiFi stack, preventing watchdog crashes
    delay(1); 
  }
}

// ================= FUNCTIONS =================

void drawMenu() {
  currentState = MENU;
  tft.fillScreen(TFT_BLACK);
  
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(60, 20);
  tft.println("ROBOT CONTROL");

  // Button 1: QR Code
  tft.fillRect(40, 60, 240, 60, TFT_BLUE);
  tft.drawRect(40, 60, 240, 60, TFT_WHITE);
  tft.setCursor(90, 80);
  tft.println("GET QR CODE");

  // Button 2: Stream
  tft.fillRect(40, 140, 240, 60, TFT_RED);
  tft.drawRect(40, 140, 240, 60, TFT_WHITE);
  tft.setCursor(80, 160);
  tft.println("LIVE CAMERA");
}

void handleTouch() {
  if (ts.touched()) {
    TS_Point p = ts.getPoint();

    // Calibration Mapping for CYD 2.8"
    int t_x = map(p.x, 200, 3700, 0, 320);
    int t_y = map(p.y, 240, 3800, 0, 240);

    // Simple Debounce
    delay(150); 

    if (currentState == MENU) {
      // Button 1: QR
      if (t_x > 40 && t_x < 280 && t_y > 60 && t_y < 120) {
        tft.fillScreen(TFT_BLACK);
        tft.setCursor(40, 100);
        tft.println("Requesting QR...");
        Serial2.print("<REQ_QR>\n"); 
      }
      
      // Button 2: Stream
      if (t_x > 40 && t_x < 280 && t_y > 140 && t_y < 200) {
        startStream();
      }
    }
    else {
      // Tap anywhere to go back to Menu
      // Important: Close HTTP connection when leaving stream to save Pi resources
      http.end(); 
      drawMenu();
      delay(500); 
    }
  }
}

void startStream() {
  currentState = SHOW_STREAM;
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(80, 100);
  tft.println("Connecting...");
  
  // Reuse the connection ("Keep-Alive")
  // This makes snapshots behave like a stream
  http.setReuse(true);
  http.begin(SNAPSHOT_URL);
  http.setTimeout(500); // Don't hang if Pi is slow
}

void drawCameraFrame() {
  if (WiFi.status() != WL_CONNECTED) {
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.setCursor(0, 0); tft.println("WiFi Lost");
    setupWiFi();
    return;
  }

  // GET request
  int httpCode = http.GET();

  if (httpCode == HTTP_CODE_OK) {
    // This works because /snapshot/cyd sends a real file size
    int len = http.getSize();
    
    // Safety check for empty or huge garbage packets
    if (len > 0 && len < 20000) { 
      // Allocate buffer
      uint8_t* jpgBuffer = (uint8_t*)malloc(len);
      
      if (jpgBuffer) {
        // Read data
        WiFiClient* stream = http.getStreamPtr();
        int bytesRead = 0;
        while (bytesRead < len) {
          int r = stream->readBytes(jpgBuffer + bytesRead, len - bytesRead);
          if (r == 0) break; // Connection died
          bytesRead += r;
        }

        // Draw
        if (bytesRead == len) {
          TJpgDec.drawJpg(0, 0, jpgBuffer, len);
        }
        
        // Clean up
        free(jpgBuffer);
      }
    }
  } else {
    // If we get an error (e.g. Pi restarting), close and wait a bit
    http.end();
    delay(100);
    http.begin(SNAPSHOT_URL);
  }
  
  // Note: We DO NOT call http.end() here. 
  // We keep the socket open for the next loop iteration (Reuse=true).
}

void processIncoming(String cmd) {
  cmd.trim();
  
  if (cmd.startsWith("QR:")) {
    String json = cmd.substring(3); 
    currentState = SHOW_QR;
    drawLargeQR(json);
  }
}

void drawLargeQR(String payload) {
    tft.fillScreen(TFT_WHITE);
    
    // Generate QR
    uint8_t* qrcodeData = (uint8_t*)malloc(qrcode_getBufferSize(QR_VERSION));
    if (qrcodeData) {
        qrcode_initText(&qrcode, qrcodeData, QR_VERSION, 0, payload.c_str());
        
        // Auto-scale logic
        int scale = 2;
        if (qrcode.size * 3 < 240) scale = 3;
        if (qrcode.size * 4 < 240) scale = 4;
        
        int offset_x = (320 - (qrcode.size * scale)) / 2;
        int offset_y = (240 - (qrcode.size * scale)) / 2;

        for (uint8_t y = 0; y < qrcode.size; y++) {
            for (uint8_t x = 0; x < qrcode.size; x++) {
                if (qrcode_getModule(&qrcode, x, y)) {
                    tft.fillRect(offset_x + (x * scale), offset_y + (y * scale), scale, scale, TFT_BLACK);
                }
            }
        }
        free(qrcodeData);
        
        tft.setTextColor(TFT_BLACK, TFT_WHITE);
        tft.setTextSize(1);
        tft.setCursor(10, 220);
        tft.println("Tap to Exit");
    } else {
        tft.println("QR RAM Error");
    }
}

void setupWiFi() {
  tft.println("Connecting WiFi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi Connected");
    Serial.println(WiFi.localIP());
  } else {
    tft.println("WiFi Failed");
  }
}
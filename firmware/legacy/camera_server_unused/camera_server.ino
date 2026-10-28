/*
 * ESP32-CAM Server
 * 
 * Provides wireless camera streaming with multiple resolution options.
 * Features:
 * - WiFi connectivity and HTTP server
 * - Multiple resolution support (low, medium, high)
 * - RESTful API endpoints for image capture
 * - Optimized for robotic vision applications
 * 
 * Hardware:
 * - ESP32-CAM module
 * - OV2640 camera sensor
 * 
 * API Endpoints:
 * - /cam-lo.jpg  - Low resolution (320x240)
 * - /cam-mid.jpg - Medium resolution (350x530)  
 * - /cam-hi.jpg  - High resolution (800x600)
 */

#include <WebServer.h>
#include <WiFi.h>
#include <esp32cam.h>

// Network configuration
const char* WIFI_SSID = "CHANGE_ME_SSID";
const char* WIFI_PASSWORD = "CHANGE_ME_PASS";

// Web server
WebServer server(80);

IPAddress local_IP(192, 168, 4, 20); 
IPAddress gateway(192, 168, 4, 1);   
IPAddress subnet(255, 255, 255, 0);  
IPAddress primaryDNS(8, 8, 8, 8); // Optional (not needed if offline)


// Camera resolutions
static auto lowRes = esp32cam::Resolution::find(320, 240);
static auto mediumRes = esp32cam::Resolution::find(640, 480);
static auto highRes = esp32cam::Resolution::find(800, 600);

// Function declarations
void setupWiFi();
void setupCamera();
void setupServer();
void serveJpegImage();
void handleLowResolution();
void handleMediumResolution(); 
void handleHighResolution();
void handleNotFound();

void setup() {
    Serial.begin(115200);
    Serial.println();
    Serial.println("ESP32-CAM Server Starting...");
    
    setupCamera();
    setupWiFi();
    setupServer();
    
    Serial.println("ESP32-CAM Server Ready!");
    Serial.printf("Camera stream available at: http://%s\n", WiFi.localIP().toString().c_str());
    Serial.println("Available endpoints:");
    Serial.println("  /cam-lo.jpg  - Low resolution (320x240)");
    Serial.println("  /cam-mid.jpg - Medium resolution (350x530)");
    Serial.println("  /cam-hi.jpg  - High resolution (800x600)");
}

void loop() {
    server.handleClient();
    delay(1);
}

void setupWiFi() {

  // 1. Disconnect and turn off radio to clear cached state
    WiFi.disconnect(true);  // "true" clears the credentials from memory
    WiFi.mode(WIFI_OFF);
    delay(100);             // Tiny breath to let the hardware reset

   // WiFi.persistent(false);
    WiFi.mode(WIFI_STA);
      WiFi.setSleep(false); 
          WiFi.setTxPower(WIFI_POWER_11dBm);
    // Static IP Setup
   if (!WiFi.config(local_IP, gateway, subnet)) {
        Serial.println("STA Failed to configure");
    }

    Serial.print("Attempting to connect to SSID: ");
    Serial.println(WIFI_SSID);

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    int retry_count = 0;
    while (WiFi.status() != WL_CONNECTED && retry_count < 20) {
        delay(500);
        Serial.print(".");
        retry_count++;
    }
    
    Serial.println();
        if (WiFi.status() == WL_CONNECTED) {
        Serial.print("WiFi connected! IP: ");
        Serial.println(WiFi.localIP());
        
        // Print the signal strength (RSSI)
        long rssi = WiFi.RSSI();
        Serial.print("Signal Strength (RSSI): ");
        Serial.print(rssi);
        Serial.println(" dBm");
    } else {
        Serial.print("WiFi Failed! Error Code: ");
        Serial.println(WiFi.status());
    }
}

void setupCamera() {
    using namespace esp32cam;
    
    Config cfg;
    cfg.setPins(pins::AiThinker);
    cfg.setResolution(highRes);
    cfg.setBufferCount(2);
    cfg.setJpeg(80);
    
    bool ok = Camera.begin(cfg);
    if (ok) {
        Serial.println("Camera initialization successful");
    } else {
        Serial.println("Camera initialization failed!");
        Serial.println("Check camera connections and power supply");
        while (true) {
            delay(1000);
        }
    }
}

void setupServer() {
    // Register endpoint handlers
    server.on("/cam-lo.jpg", HTTP_GET, handleLowResolution);
    server.on("/cam-mid.jpg", HTTP_GET, handleMediumResolution);
    server.on("/cam-hi.jpg", HTTP_GET, handleHighResolution);
    server.onNotFound(handleNotFound);
    
    server.begin();
    Serial.println("HTTP server started on port 80");
}

void serveJpegImage() {
    auto frame = esp32cam::capture();
    
    if (frame == nullptr) {
        Serial.println("Camera capture failed");
        server.send(503, "text/plain", "Camera capture failed");
        return;
    }
    
    Serial.printf("Captured frame: %dx%d, %d bytes\n", 
                  frame->getWidth(), 
                  frame->getHeight(),
                  static_cast<int>(frame->size()));
    
    // Set response headers
    server.setContentLength(frame->size());
    server.send(200, "image/jpeg");
    
    // Stream image data
    WiFiClient client = server.client();
    frame->writeTo(client);
}

void handleLowResolution() {
    Serial.println("Request: Low resolution image");
    
    if (!esp32cam::Camera.changeResolution(lowRes)) {
        Serial.println("Failed to change to low resolution");
        server.send(500, "text/plain", "Failed to change camera resolution");
        return;
    }
    
    serveJpegImage();
}

void handleMediumResolution() {
    Serial.println("Request: Medium resolution image");
    
    if (!esp32cam::Camera.changeResolution(mediumRes)) {
        Serial.println("Failed to change to medium resolution");
        server.send(500, "text/plain", "Failed to change camera resolution");
        return;
    }
    
    serveJpegImage();
}

void handleHighResolution() {
    Serial.println("Request: High resolution image");
    
    if (!esp32cam::Camera.changeResolution(highRes)) {
        Serial.println("Failed to change to high resolution");
        server.send(500, "text/plain", "Failed to change camera resolution");
        return;
    }
    
    serveJpegImage();
}

void handleNotFound() {
    String message = "ESP32-CAM Server\n\n";
    message += "Available endpoints:\n";
    message += "/cam-lo.jpg  - Low resolution (320x240)\n";
    message += "/cam-mid.jpg - Medium resolution (350x530)\n";
    message += "/cam-hi.jpg  - High resolution (800x600)\n\n";
    message += "Request: " + server.uri() + " not found\n";
    
    server.send(404, "text/plain", message);
}
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Adafruit_SSD1306.h>
#include <ArduinoJson.h>
#include <MAX30105.h>
#include <WiFiManager.h> 

#define I2C_SDA 21
#define I2C_SCL 22
#define BUZZER_PIN 13
#define LED_PIN 12
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
MAX30105 particleSensor;
WebServer server(80);

// --- Biến toàn cục chia sẻ ---
volatile float shared_BPM = 0;
volatile float shared_SpO2 = 0;
volatile float shared_IR_Filtered = 0;
volatile bool handDetected = false;

// Dữ liệu đồ thị OLED
#define WAVE_WIDTH 128
int waveIR[WAVE_WIDTH];

#include "webpage.h"

// Task Handle
TaskHandle_t TaskSensor;

// --- Callback khi vào chế độ cấu hình WiFi ---
// Hàm này được gọi khi không tìm thấy WiFi cũ, ESP32 phát WiFi để cấu hình
void configModeCallback (WiFiManager *myWiFiManager) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("WIFI MANAGER MODE");
  display.println("-----------------");
  display.println("Connect to WiFi:");
  display.setCursor(0, 30);
  display.setTextSize(1); 
  display.println("ESP32_Pulse_Config"); // Tên WiFi do ESP phát ra
  display.println("\nIP: 192.168.4.1");
  display.display();
  
  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP());
}

// --- CORE 0: Task Xử lý Cảm biến (Giữ nguyên logic cũ) ---
void TaskSensorCode(void * parameter) {
  if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {
    Serial.println("Sensor Init Failed in Task!");
    vTaskDelete(NULL);
  }
  
  particleSensor.setup(); 
  particleSensor.setPulseAmplitudeRed(0x24);
  particleSensor.setPulseAmplitudeIR(0x24);
  particleSensor.setPulseAmplitudeGreen(0);
  particleSensor.setSampleRate(100);
  particleSensor.setADCRange(4096);
  particleSensor.setPulseWidth(411);
  particleSensor.setFIFOAverage(4);

  const unsigned long sampleIntervalUs = 10000; // 100Hz
  unsigned long lastSampleMicros = 0;
  float dc_ir = 0, dc_red = 0;
  const float alpha_dc = 0.95;
  
  float irPrev = 0, irPrev2 = 0, yPrev = 0, yPrev2 = 0;
  const float b0 = 0.0201, b1 = 0.0, b2 = -0.0201;
  const float a1 = -1.5610, a2 = 0.6414;

  unsigned long lastPeakTime = 0;
  float recentIntervals[8];
  int intervalIdx = 0, intervalsCount = 0;
  float currentBPM = 0;
  float avgR = 0; 
  float currentSpO2 = 0;

  for(;;) {
    unsigned long now = micros();
    if (now - lastSampleMicros >= sampleIntervalUs) {
      lastSampleMicros += sampleIntervalUs;

      particleSensor.check(); 
      while (particleSensor.available()) {
        long irRaw = particleSensor.getFIFOIR();
        long redRaw = particleSensor.getFIFORed();
        particleSensor.nextSample(); 
          Serial.print("IR Value: ");
          Serial.println(irRaw); 

        if (irRaw < 7000) { 
           handDetected = false;
           shared_IR_Filtered = 0;
           shared_BPM = 0;
           shared_SpO2 = 0;
           dc_ir = 0; dc_red = 0; 
           irPrev = 0; irPrev2 = 0; yPrev = 0; yPrev2 = 0;
           intervalsCount = 0;
           continue; 
        } else {
           if (!handDetected) {
             handDetected = true;
             dc_ir = irRaw; dc_red = redRaw; 
           }
        }

        float irf = irRaw; 
        float redf = redRaw; 

        dc_ir = alpha_dc * dc_ir + (1.0 - alpha_dc) * irf;
        dc_red = alpha_dc * dc_red + (1.0 - alpha_dc) * redf;
        float ac_ir = irf - dc_ir;
        float ac_red = redf - dc_red;

        float irFiltered = b0 * ac_ir + b1 * irPrev + b2 * irPrev2 - a1 * yPrev - a2 * yPrev2;
        irPrev2 = irPrev; irPrev = ac_ir; 
        yPrev2 = yPrev; yPrev = irFiltered;

        shared_IR_Filtered = irFiltered; 

        static float lastVal = 0;
        static bool isPeak = false;
        if (irFiltered < lastVal && lastVal > 20 && !isPeak) { 
           unsigned long t = millis();
           if (t - lastPeakTime > 300) { 
              float interval = (t - lastPeakTime) / 1000.0;
              lastPeakTime = t;
              recentIntervals[intervalIdx] = interval;
              intervalIdx = (intervalIdx + 1) % 8;
              if (intervalsCount < 8) intervalsCount++;

              float sum = 0;
              for (int i=0; i<intervalsCount; i++) sum += recentIntervals[i];
              float mean = sum / intervalsCount;
              currentBPM = 60.0 / mean; // tính Bpm thô
              //
              tone(BUZZER_PIN, 2000, 50); 
           }
           isPeak = true;
        } else if (irFiltered > lastVal) {
           isPeak = false;
        }
        lastVal = irFiltered;

        float R = (fabs(ac_red) / dc_red) / (fabs(ac_ir) / dc_ir);
        if (R > 0.2 && R < 3.0) { 
           avgR = 0.98 * avgR + 0.02 * R; 
           float spo2 = 107.0 - 14.0 * avgR;
           spo2 = constrain(spo2, 0, 100);
           currentSpO2 = spo2;
        }
        shared_BPM = shared_BPM * 0.9 + currentBPM * 0.1;
        shared_SpO2 = shared_SpO2 * 0.95 + currentSpO2 * 0.05;
      }
    }
    vTaskDelay(1); 
  }
}

// ----------------- Setup -----------------
void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  // 1. Khởi động màn hình trước để hiện trạng thái
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("OLED error"));
    for(;;);
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0,10);
  display.println("Khoi dong WiFi...");
  display.display();

  // 2. Cấu hình WiFiManager
  WiFiManager wm;
  
  // Xóa cài đặt cũ để test (Bỏ comment dòng dưới nếu muốn reset WiFi mỗi lần nạp code)
  wm.resetSettings();

  // Đặt giao diện OLED khi vào chế độ Config
  wm.setAPCallback(configModeCallback);
  
  // Tự động kết nối. Nếu thất bại sẽ tạo AP tên "ESP32_Pulse_Config"
  // Sau 60s không ai cấu hình, nó sẽ tiếp tục chạy offline
  wm.setConfigPortalTimeout(30); 

  if(!wm.autoConnect("ESP32_Pulse_Config")) {
    Serial.println("Khong ket noi duoc WiFi - Chay che do Offline");
    display.clearDisplay();
    display.println("WiFi Failed!");
    display.println("Offline Mode");
    display.display();
    delay(2000);
  } else {
    Serial.println("Da ket noi WiFi!");
    Serial.println(WiFi.localIP());
    display.clearDisplay();
    display.println("WiFi OK!");
    display.println(WiFi.localIP());
    display.display();
    delay(2000);
  }

  // 3. Khởi động Web Server hiển thị dữ liệu (Chỉ chạy khi có mạng hoặc sau timeout)
  server.on("/", []() { server.send(200, "text/html", webpageHTML); });
  server.on("/data", []() {
    StaticJsonDocument<192> doc;
    doc["bpm"] = (int)round(shared_BPM);
    doc["spo2"] = (int)round(shared_SpO2);
    doc["ir"] = (int)(shared_IR_Filtered * 1.5); 
    String json;
    serializeJson(doc, json);
    server.send(200, "application/json", json);
  });
  server.begin();

  // 4. Bắt đầu Task Cảm biến (Core 0)
  xTaskCreatePinnedToCore(
    TaskSensorCode,  
    "TaskSensor",     
    10000,           
    NULL,             
    1,               
    &TaskSensor,      
    0);              
}

// ----------------- Loop (Core 1) -----------------
void loop() {
  // Nếu có mạng thì xử lý Web Server
  if(WiFi.status() == WL_CONNECTED) {
    server.handleClient();
  }
  
  // Xử lý hiển thị OLED
  static unsigned long lastDraw = 0;
  if (millis() - lastDraw > 33) { 
    lastDraw = millis();
    
    float currentSample = shared_IR_Filtered;
    
    // Tự động scale biên độ đồ thị OLED
    static float maxAmp = 50;
    maxAmp = 0.98 * maxAmp + 0.02 * abs(currentSample);
    if(maxAmp < 20) maxAmp = 20;

    int yVal = map((int)currentSample, -maxAmp, maxAmp, 0, 48);
    yVal = constrain(yVal, 0, 48);

    for (int i = 0; i < WAVE_WIDTH - 1; i++) waveIR[i] = waveIR[i + 1];
    waveIR[WAVE_WIDTH - 1] = yVal;

    display.clearDisplay();
    
    if(!handDetected) {
       display.setCursor(20, 25);
       display.setTextSize(1);
       display.println("Place Finger...");
       digitalWrite(LED_PIN, LOW);
       
       // Hiện IP góc dưới nếu chưa đặt tay
       if(WiFi.status() == WL_CONNECTED) {
         display.setCursor(0, 55);
         display.setTextSize(1);
         display.print("IP: ");
         display.print(WiFi.localIP());
       }

    } else {
       digitalWrite(LED_PIN, HIGH);
       display.setCursor(0, 0);
       display.setTextSize(1);
       display.printf("BPM:%d  SpO2:%d%%", (int)shared_BPM, (int)shared_SpO2);

       int graphBottom = 63;
       for (int i = 0; i < WAVE_WIDTH - 1; i++) {
         int y1 = graphBottom - waveIR[i];
         int y2 = graphBottom - waveIR[i + 1];
         display.drawLine(i, y1, i + 1, y2, SSD1306_WHITE);
       }
    }
    display.display();
  }
}

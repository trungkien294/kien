#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include "time.h"

const char* ssid     = "TB 101";
const char* password = "8888@8888";
const char* serverUrl = "http://192.168.11.103:5000/api/log";

const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 7 * 3600;
const int   daylightOffset_sec = 0;

#define SENSOR_GATE_IN   4
#define SENSOR_GATE_OUT  5
#define SENSOR_PARK1     14
#define SENSOR_PARK2     19
#define FLAME_SENSOR     13
#define RELAY_PIN        26
#define BUZZER_PIN       15
#define DHT_PIN          23
#define SERVO_IN_PIN     32
#define SERVO_OUT_PIN    33

#define DHTTYPE DHT11
DHT dht(DHT_PIN, DHTTYPE);

LiquidCrystal_I2C lcd(0x27, 16, 2);
Servo servoIn;
Servo servoOut;

unsigned long lastSensorUpdate = 0;
unsigned long lastTempUpdate = 0;
unsigned long flameStartTime = 0;
bool fireActive = false;

bool waitingForIn = false;
bool waitingForOut = false;

void sendData(String type, String timestamp, float temp = -1, float hum = -1);
void handleSerialCommand(String cmd);

void setup() {
  Serial.begin(115200);

  pinMode(SENSOR_GATE_IN, INPUT);
  pinMode(SENSOR_GATE_OUT, INPUT);
  pinMode(SENSOR_PARK1, INPUT);
  pinMode(SENSOR_PARK2, INPUT);
  pinMode(FLAME_SENSOR, INPUT);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Smart Parking");

  servoIn.attach(SERVO_IN_PIN);
  servoOut.attach(SERVO_OUT_PIN);
  servoIn.write(0);
  servoOut.write(0);

  dht.begin();
  digitalWrite(RELAY_PIN, HIGH);
  digitalWrite(BUZZER_PIN, HIGH);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");

  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
}
unsigned long flameClearStartTime = 0; 
void loop() {
  bool flameNow = digitalRead(FLAME_SENSOR) == LOW;

  if (flameNow) {
    flameClearStartTime = 0; // Reset thời gian không cháy
    if (!fireActive) {
      fireActive = true;
      fireStart();
    }
    fireLoop();
    return;
  } else if (fireActive) {
    // Nếu đang cháy nhưng cảm biến không còn LOW
    if (flameClearStartTime == 0) {
      flameClearStartTime = millis();  // Bắt đầu đếm thời gian không cháy
    } else if (millis() - flameClearStartTime > 3000) {
      fireEnd();  // Chỉ tắt báo cháy sau 3s không phát hiện lửa
    }
  }

  if (digitalRead(FLAME_SENSOR) == LOW) {
    if (!fireActive) {
      fireActive = true;
      fireStart();
    }
    fireLoop();
    return;
  } else if (fireActive) {
    fireEnd();
  }

  // Đọc lệnh từ Python
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    handleSerialCommand(cmd);
  }

  if (millis() - lastSensorUpdate >= 500) {
    lastSensorUpdate = millis();

    // Kiểm tra sensor vào
if (digitalRead(SENSOR_GATE_IN) == LOW) {
  if (!waitingForIn) {
    waitingForIn = true;
    Serial.println("REQ_IN");
  }
} else {
  waitingForIn = false;  // Reset khi cảm biến không còn LOW
}

// Kiểm tra sensor ra
if (digitalRead(SENSOR_GATE_OUT) == LOW) {
  if (!waitingForOut) {
    waitingForOut = true;
    Serial.println("REQ_OUT");
  }
} else {
  waitingForOut = false;  // Reset khi cảm biến không còn LOW
}
    updateParkingStatus();
  }

  if (millis() - lastTempUpdate >= 5000) {
    lastTempUpdate = millis();
    float t = dht.readTemperature();
    float h = dht.readHumidity();
    int f = digitalRead(FLAME_SENSOR) == LOW ? 1 : 0;
    if (!isnan(t) && !isnan(h)) {
      Serial.printf("DHT:%.2f,%.2f,%d\n", t, h, f);  // gửi qua Serial
    }
  }
}

void handleSerialCommand(String cmd) {
  String timeNow = getCurrentTime();
  if (cmd == "OPEN_IN" && waitingForIn) {
    beep();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Xe vao luc:");
    lcd.setCursor(0, 1);
    lcd.print(timeNow);
    Serial.println("[VÀO] Xe vào lúc: " + timeNow);
    sendData("IN", timeNow);
    servoIn.write(90);
    delay(3000);
    servoIn.write(0);
    waitingForIn = false;
  }
  else if (cmd == "OPEN_OUT" && waitingForOut) {
    beep();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Xe ra luc:");
    lcd.setCursor(0, 1);
    lcd.print(timeNow);
    Serial.println("[RA] Xe ra lúc: " + timeNow);
    sendData("OUT", timeNow);
    servoOut.write(90);
    delay(3000);
    servoOut.write(0);
    waitingForOut = false;
  }
}

void sendData(String type, String timestamp, float temp, float hum) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(serverUrl);
    http.addHeader("Content-Type", "application/json");

    String postData = "{";
    postData += "\"type\":\"" + type + "\","; 
    postData += "\"timestamp\":\"" + timestamp + "\","; 
    postData += "\"temperature\":" + String(temp) + ","; 
    postData += "\"humidity\":" + String(hum) + ","; 
    postData += "\"fire\":" + String(fireActive ? 1 : 0); 
    postData += "}";

    int httpResponseCode = http.POST(postData);
    Serial.println("POST -> " + String(httpResponseCode));
    http.end();
  }
}

void updateParkingStatus() {
  bool p1 = digitalRead(SENSOR_PARK1) == LOW;
  bool p2 = digitalRead(SENSOR_PARK2) == LOW;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("P1: ");
  lcd.print(p1 ? "Full" : "Free");
  lcd.setCursor(0, 1);
  lcd.print("P2: ");
  lcd.print(p2 ? "Full" : "Free");
}

void beep() {
  digitalWrite(BUZZER_PIN, LOW);
  delay(200);
  digitalWrite(BUZZER_PIN, HIGH);
}

void fireStart() {
  Serial.println("🔥 CẢNH BÁO CHÁY!");
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(RELAY_PIN, LOW);
  servoIn.write(90);
  servoOut.write(90);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("!!! FIRE ALERT !!!");
  flameStartTime = millis();
  sendData("FIRE", getCurrentTime());
}

void fireLoop() {
  if (millis() - flameStartTime > 1000) {
    digitalWrite(BUZZER_PIN, !digitalRead(BUZZER_PIN));
    flameStartTime = millis();
  }
}

void fireEnd() {
  Serial.println("✅ Cháy đã kết thúc.");
  digitalWrite(BUZZER_PIN, HIGH);
  digitalWrite(RELAY_PIN, HIGH);
  fireActive = false;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("System Restarted");
  delay(2000);
  updateParkingStatus();
  servoIn.write(0);
  servoOut.write(0);
}

String getCurrentTime() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    return "No time";
  }
  char buf[30];
  strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &timeinfo);
  return String(buf);
}

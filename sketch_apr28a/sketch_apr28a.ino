#include <WiFi.h> // Use WiFi.h for ESP32 or ESP8266WiFi.h for ESP8266
#include <HTTPClient.h>

const char* ssid = "SceneScribe-b24b49";
const char* password = "scenescribe.pi1234";

const int triggerPin = 4; // GPIO pin for interrupt
volatile bool stateChanged = false;
volatile int currentState = HIGH;
unsigned long lastInterruptTime = 0;

void IRAM_ATTR handleInterrupt() {
  unsigned long interruptTime = millis();
  // Debounce - ignore interrupts too close together
  if (interruptTime - lastInterruptTime > 200) {
    stateChanged = true;
    currentState = !digitalRead(triggerPin);
  }
  lastInterruptTime = interruptTime;
}


void setup() {
  Serial.begin(115200);
  connectWifi(ssid,password);
  
  
  // Set up interrupt pin
  pinMode(triggerPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(triggerPin), handleInterrupt, CHANGE);
}

void loop() {
  if (stateChanged) {
    stateChanged = false;
    Serial.println("state changed");
    callRaspiAPI(currentState);
  }
  delay(10); // Small delay to prevent watchdog trigger
}

void connectWifi(const char* ssid, const char* password) {
  WiFi.begin(ssid, password);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();
}

void callRaspiAPI(int state) {
  HTTPClient http;
  
  // Raspberry Pi should connect to ESP's SoftAP
  String serverUrl = "http://192.168.4.1:5001/api/state"; // Example URL
  String payload = "{\"state\":" + String(state) + "}";
  
  http.begin(serverUrl);
  http.addHeader("Content-Type", "application/json");
  
  int httpCode = http.POST(payload);
  
  if (httpCode > 0) {
    String response = http.getString();
    Serial.println(httpCode);
    Serial.println(response);
  } else {
    Serial.println("Error on HTTP request");
  }
  
  http.end();
}
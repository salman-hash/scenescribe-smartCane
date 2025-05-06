#include <ESP32Servo.h>
#include <WiFi.h>
#include <HTTPClient.h>

const char* ssid = "SceneScribe";
const char* password = "pi123456";

// Define button pin and it's variables
#define MIC_BUTTON_PIN 4    // Button for recording the Normal prompt 
#define GK_BUTTON_PIN 4     // Button for recording the General Knowledge prompt 
const int triggerPin = 15;
volatile bool stateChanged = false;
volatile int currentState = HIGH;
unsigned long lastInterruptTime = 0;

#define TRIG_PIN 5
#define ECHO_PIN 18
#define LED_PIN 2
static const int HF_SERVO_PIN = 13;

// Motor driver pins
const int motorPin1 = 32;
const int motorPin2 = 33;

// Encoder pins
const int encoderPinA = 27;
const int encoderPinB = 26;

volatile long encoderCount = 0;
int direction = 0; // 1 for forward, -1 for reverse


Servo servo_f;


// Interrupt Service Routine for encoder
void IRAM_ATTR encoderISR() {
  int stateB = digitalRead(encoderPinA);
  if (stateB == HIGH) {
    encoderCount += 1 * direction;
  } else {
    encoderCount -= 1 * direction;
  }
}
// Interrupt Service Routine for button
void IRAM_ATTR handleInterrupt() {
  unsigned long interruptTime = millis();
  // Debounce - ignore interrupts too close together
  if (interruptTime - lastInterruptTime > 500) {
    stateChanged = true;
  }
  lastInterruptTime = interruptTime;
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
  digitalWrite(LED_PIN, HIGH);
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

// Move motor right (forward)
void moveRight(long targetSteps = 550) {
  encoderCount = 0;
  direction = 1;

  digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin2, LOW);

  delay(5000);

  stopMotor();
}

// Move motor left (reverse)
void moveLeft(long targetSteps = 550) {
  encoderCount = 0;
  direction = -1;

  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, HIGH);

  delay(5000);

  stopMotor();
}

// Stop motor
void stopMotor() {
  digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin2, HIGH);
}

float get_distance(){
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(1000);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH);
  float distance = (duration * 0.0343) / 2;  // Convert to cm (Speed of sound = 343m/s)
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  return distance;
}

void buttonCallback(void *pvParameters){
  while(1){
    if (stateChanged) {
      stateChanged = false;
      Serial.println("state changed");
      currentState = !digitalRead(triggerPin);
      callRaspiAPI(currentState);
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

void directionControl(void *pvParameters){
  while(1){
    if(get_distance() < 100.0){
      if(get_distance() < 100.0){
        moveRight(200);
        delay(1000);    
        if(get_distance() > 100.0){
          servo_f.write(180);

          moveLeft(175);
          Serial.println(" directing right");
        }else{
          moveLeft(650);
          delay(1000);
          if(get_distance() < 100.0){
            servo_f.write(180);
            moveRight(200);          
            Serial.println(" directing left");
            delay(1000);
          }else{
            servo_f.write(0);
            moveRight(200);          
            Serial.println(" directing left");
            delay(1000);
          }
        }
        delay(2000);
        servo_f.write(90);
      }
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }

}
void setup() {
  
  Serial.begin(115200);
  connectWifi(ssid, password);
  servo_f.attach(HF_SERVO_PIN);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);

  // Motor pins as output
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);

  // Encoder pins as input
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);

  // Configure button pins as INPUT_PULLUP
  pinMode(MIC_BUTTON_PIN, INPUT_PULLUP);
  pinMode(GK_BUTTON_PIN, INPUT_PULLUP);
  
  // Set up button interrupt pin
  pinMode(triggerPin, INPUT_PULLUP);

  // Attach interrupt on encoder channel A
  attachInterrupt(digitalPinToInterrupt(encoderPinB), encoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(triggerPin), handleInterrupt, CHANGE);

  // Create Button Callback pinned to Core 0
  xTaskCreatePinnedToCore(
    buttonCallback,                // Task function
    "Button Task",             // Name of the task
    4096,                 // Stack size (in words, 1000 is usually enough)
    NULL,                 // Parameters for the task
    1,                    // Task priority
    NULL,                 // Task handle (not used)
    0                     // Core 0
  );
  
  // Create Direction Control pinned to Core 1
  xTaskCreatePinnedToCore(
    directionControl,                // Task function
    "Direction Task",             // Name of the task
    4096,                 // Stack size
    NULL,                 // Parameters
    1,                    // Task priority
    NULL,                 // Task handle (not used)
    1                     // Core 1
  );

}


void loop() {
}


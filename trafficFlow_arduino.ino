#include <Wire.h>
#include <ESP32Servo.h>
#include <RTClib.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <PubSubClient.h>
#include <FirebaseESP32.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <SPIFFS.h>
#include "esp_camera.h"

// Pin Assignments
#define RED_LIGHT_1 26
#define YELLOW_LIGHT_1 27
#define GREEN_LIGHT_1 14
#define RED_LIGHT_2 12

#define TRIG1 4
#define ECHO1 5
#define TRIG2 18
#define ECHO2 19

#define IR_SENSOR1 33  
#define IR_SENSOR2 32  

#define SERVO_PIN 13
Servo servo;

#define GSM_TX 17
#define GSM_RX 16
HardwareSerial gsmSerial(2);

RTC_DS3231 rtc;

// WiFi Configuration
const char* ssid = "Aniii";
const char* password = "987654321";

// ThingsBoard MQTT Configuration
const char* mqtt_broker = "demo.thingsboard.io";
const int mqtt_port = 1883;
const char* mqtt_token = "majez4vuz4j7xn0rhvg1";
const char* mqtt_topic = "v1/devices/me/telemetry";

// Firebase Configuration
FirebaseConfig firebaseConfig;
FirebaseAuth firebaseAuth;
FirebaseData firebaseData;
const char* FIREBASE_HOST = "traffic-new-90b5d-default-rtdb.firebaseio.com";
const char* FIREBASE_AUTH = "yRKiQPJpbkDOWWZdCsjr32KsWiNDdw6ofTvLpuac";

// Junction Identifier
const String junctionID = "junction_1";

// Federated Learning Configuration
const char* FL_SERVER = "http://172.18.160.1:5000/api";
const int MODEL_VERSION_CHECK_INTERVAL = 3600000;
unsigned long lastModelCheck = 0;
const int LOCAL_TRAIN_INTERVAL = 1800000;
unsigned long lastTrainingTime = 0;

// Model Parameters
float weights[8] = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
float bias = 1.0;

// Local Data Storage for Training
const int MAX_DATA_POINTS = 1000;
int dataCount = 0;
struct DataPoint {
  float features[8];
  float optimal_timing;
};
DataPoint trainingData[MAX_DATA_POINTS];

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// Vehicle Counters
int vehicleCount1 = 0;
int vehicleCount2 = 0;

// Previous IR Sensor States
int prevIR1 = HIGH;
int prevIR2 = HIGH;

// Timestamps for debouncing
unsigned long lastChangeTime1 = 0;
unsigned long lastChangeTime2 = 0;
const int debounceDelay = 50;

// Adaptive timing parameters
int greenTimeLane1 = 10000;
int greenTimeLane2 = 10000;
int yellowTime = 3000;

// Function prototypes
void setupFirebase();
void setupWiFi();
void setupMQTT();
void reconnectMQTT();
long getDistance(int trigPin, int echoPin);
void countVehicles();
int predictOptimalTiming(float features[]);
String getCurrentTimestamp();
void storeEffectivenessData(int greenTime, int vehiclesPassed1, int vehiclesPassed2, int traffic1, int traffic2);
void saveModelToSPIFFS();
void loadModelFromSPIFFS();
void sendModelToServer();
void checkForModelUpdates();
void trainLocalModel();
void controlTrafficLights();
void sendTelemetry();
void checkSensors();
void setupFederatedLearning();
void handleFederatedLearningTasks();

void setup() {
  Serial.begin(115200);
  Serial.println("Traffic Control System Initializing...");
  
  // Initialize pins
  pinMode(RED_LIGHT_1, OUTPUT);
  pinMode(YELLOW_LIGHT_1, OUTPUT);
  pinMode(GREEN_LIGHT_1, OUTPUT);
  pinMode(RED_LIGHT_2, OUTPUT);
  
  pinMode(TRIG1, OUTPUT);
  pinMode(ECHO1, INPUT);
  pinMode(TRIG2, OUTPUT);
  pinMode(ECHO2, INPUT);
  
  pinMode(IR_SENSOR1, INPUT);
  pinMode(IR_SENSOR2, INPUT);
  
  servo.attach(SERVO_PIN);
  gsmSerial.begin(9600, SERIAL_8N1, GSM_RX, GSM_TX);
  
  if (rtc.begin()) {
    Serial.println("✅ RTC initialized successfully");
  } else {
    Serial.println("❌ RTC initialization failed");
  }
  String ip = WiFi.localIP().toString();
  Firebase.setString(firebaseData, "/junction_info/camera_stream", "http://" + ip + ":81/stream");

  Serial.println("Camera Stream URL uploaded to Firebase!");
  setupWiFi();
  setupMQTT();
  setupFirebase();
  setupFederatedLearning();
  
  checkSensors();
  
  Serial.println("✅ Setup complete!");
}

void loop() {
  // Maintain MQTT connection
  if (!mqttClient.connected()) {
    reconnectMQTT();
  }
  mqttClient.loop();
  
  // Count vehicles continuously
  countVehicles();
  
  // Control traffic lights based on ML model
  controlTrafficLights();
  
  // Send telemetry data
  sendTelemetry();
  
  // Handle federated learning tasks
  handleFederatedLearningTasks();
  
  // Perform sensor check every 10 minutes
  static unsigned long lastSensorCheck = 0;
  if (millis() - lastSensorCheck > 600000) {
    checkSensors();
    lastSensorCheck = millis();
  }
   static unsigned long lastTrafficDataSend = 0;
  if (millis() - lastTrafficDataSend > 30000) {
    sendTrafficDataToServer();
    lastTrafficDataSend = millis();
  }
}


void setupFirebase() {
  firebaseConfig.host = FIREBASE_HOST;
  firebaseConfig.signer.tokens.legacy_token = FIREBASE_AUTH;
  Firebase.begin(&firebaseConfig, &firebaseAuth);
  Firebase.reconnectWiFi(true);
  Serial.println("✅ Firebase Initialized Successfully!");
}

void setupWiFi() {
  Serial.print("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  
  unsigned long startAttemptTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) {
    delay(500);
    Serial.print(".");
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✅ WiFi Connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\n❌ WiFi Connection Failed!");
  }
}

void setupMQTT() {
  mqttClient.setServer(mqtt_broker, mqtt_port);
  Serial.println("✅ MQTT client configured");
}

void reconnectMQTT() {
  int attempts = 0;
  while (!mqttClient.connected() && attempts < 3) {
    Serial.println("Connecting to MQTT Broker...");
    if (mqttClient.connect("ESP32Client", mqtt_token, NULL)) {
      Serial.println("✅ Connected to ThingsBoard!");
    } else {
      Serial.print("❌ MQTT Connection Failed! Error Code: ");
      Serial.println(mqttClient.state());
      delay(2000);
      attempts++;
    }
  }
}

long getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  long duration = pulseIn(echoPin, HIGH, 30000);
  if (duration == 0) return -1;
  return duration * 0.034 / 2;
}

void countVehicles() {
  int ir1 = digitalRead(IR_SENSOR1);
  int ir2 = digitalRead(IR_SENSOR2);
  unsigned long currentTime = millis();

  if (prevIR1 == HIGH && ir1 == LOW) {
    if (currentTime - lastChangeTime1 > debounceDelay) {
      vehicleCount1++;
      Serial.print("✅ Vehicle Detected by IR Sensor 1! Total: ");
      Serial.println(vehicleCount1);
      lastChangeTime1 = currentTime;
    }
  }

  if (prevIR2 == HIGH && ir2 == LOW) {
    if (currentTime - lastChangeTime2 > debounceDelay) {
      vehicleCount2++;
      Serial.print("✅ Vehicle Detected by IR Sensor 2! Total: ");
      Serial.println(vehicleCount2);
      lastChangeTime2 = currentTime;
    }
  }

  prevIR1 = ir1;
  prevIR2 = ir2;
}

int predictOptimalTiming(float features[]) {
  float prediction = bias;
  for (int i = 0; i < 8; i++) {
    prediction += weights[i] * features[i];
  }
  
  if (prediction < 5000) prediction = 5000;
  if (prediction > 30000) prediction = 30000;
  
  return (int)prediction;
}

String getCurrentTimestamp() {
  DateTime now = rtc.now();
  char timestamp[20];
  sprintf(timestamp, "%04d%02d%02d%02d%02d%02d", 
          now.year(), now.month(), now.day(), 
          now.hour(), now.minute(), now.second());
  return String(timestamp);
}

void storeEffectivenessData(int greenTime, int vehiclesPassed1, int vehiclesPassed2, int traffic1, int traffic2) {
  String path = "/model_feedback/" + junctionID + "/" + String(millis());
  
  FirebaseJson json;
  json.set("green_time", greenTime);
  json.set("vehicles_passed_1", vehiclesPassed1);
  json.set("vehicles_passed_2", vehiclesPassed2);
  json.set("traffic_density_1", traffic1);
  json.set("traffic_density_2", traffic2);
  json.set("timestamp", getCurrentTimestamp());
  
  if (Firebase.setJSON(firebaseData, path, json)) {
    Serial.println("✅ Effectiveness data stored for model improvement");
  } else {
    Serial.println("❌ Failed to store effectiveness data: " + firebaseData.errorReason());
  }
}

void saveModelToSPIFFS() {
  if (!SPIFFS.begin(true)) {
    Serial.println("❌ Failed to mount SPIFFS");
    return;
  }
  
  File file = SPIFFS.open("/model.json", "w");
  if (!file) {
    Serial.println("❌ Failed to open file for writing");
    return;
  }
  
  DynamicJsonDocument doc(1024);
  
  JsonArray weightsArray = doc.createNestedArray("weights");
  for (int i = 0; i < 8; i++) {
    weightsArray.add(weights[i]);
  }
  
  doc["bias"] = bias;
  doc["version"] = millis();
  doc["junction_id"] = junctionID;
  
  if (serializeJson(doc, file) == 0) {
    Serial.println("❌ Failed to write model to file");
  } else {
    Serial.println("✅ Model saved to SPIFFS");
  }
  
  file.close();
}

void loadModelFromSPIFFS() {
  if (!SPIFFS.begin(true)) {
    Serial.println("❌ Failed to mount SPIFFS");
    return;
  }
  
  if (!SPIFFS.exists("/model.json")) {
    Serial.println("❌ No model file found");
    return;
  }
  
  File file = SPIFFS.open("/model.json", "r");
  if (!file) {
    Serial.println("❌ Failed to open file for reading");
    return;
  }
  
  DynamicJsonDocument doc(1024);
  DeserializationError error = deserializeJson(doc, file);
  
  if (error) {
    Serial.print("❌ Failed to parse model file: ");
    Serial.println(error.c_str());
    file.close();
    return;
  }
  
  JsonArray weightsArray = doc["weights"];
  for (int i = 0; i < 8 && i < weightsArray.size(); i++) {
    weights[i] = weightsArray[i];
  }
  
  bias = doc["bias"];
  
  Serial.println("✅ Model loaded from SPIFFS");
  
  file.close();
}

void sendModelToServer() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("❌ WiFi not connected. Cannot send model to server.");
    return;
  }
  
  HTTPClient http;
  String url = String(FL_SERVER) + "/submit_model";
  
  http.begin(url);
  http.addHeader("Content-Type", "application/json");
  
  DynamicJsonDocument doc(1024);
  
  JsonArray weightsArray = doc.createNestedArray("weights");
  for (int i = 0; i < 8; i++) {
    weightsArray.add(weights[i]);
  }
  
  doc["bias"] = bias;
  doc["junction_id"] = junctionID;
  doc["data_points"] = dataCount;
  
  String requestBody;
  serializeJson(doc, requestBody);
  
  Serial.println("Sending local model to server");
  
  int httpResponseCode = http.POST(requestBody);
  
  if (httpResponseCode > 0) {
    String response = http.getString();
    Serial.print("✅ Server Response: ");
    Serial.println(response);
  } else {
    Serial.print("❌ Error on sending model: ");
    Serial.println(httpResponseCode);
  }
  
  http.end();
}

void checkForModelUpdates() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("❌ WiFi not connected. Cannot check for model updates.");
    return;
  }
  
  HTTPClient http;
  String url = String(FL_SERVER) + "/get_global_model?junction_id=" + junctionID;
  
  http.begin(url);
  
  int httpResponseCode = http.GET();
  
  if (httpResponseCode > 0) {
    String response = http.getString();
    
    DynamicJsonDocument doc(1024);
    DeserializationError error = deserializeJson(doc, response);
    
    if (!error) {
      if (doc.containsKey("weights") && doc["weights"].is<JsonArray>()) {
        JsonArray weightsArray = doc["weights"];
        
        if (weightsArray.size() >= 8) {
          Serial.println("✅ Updating model weights and bias from server");
          
          for (int i = 0; i < 8 && i < weightsArray.size(); i++) {
            weights[i] = weightsArray[i];
          }
          
          if (doc.containsKey("bias")) {
            bias = doc["bias"];
          }
          
          saveModelToSPIFFS();
          Serial.println("✅ Local model updated successfully");
        }
      }
    }
  }
  
  http.end();
}

void trainLocalModel() {
  if (dataCount < 10) {
    Serial.println("❌ Not enough data points for training");
    return;
  }
  
  Serial.println("Training local model...");
  
  float learningRate = 0.01;
  int epochs = 10;
  
  for (int epoch = 0; epoch < epochs; epoch++) {
    float totalLoss = 0.0;
    
    for (int i = 0; i < dataCount; i++) {
      float prediction = bias;
      for (int j = 0; j < 8; j++) {
        prediction += weights[j] * trainingData[i].features[j];
      }
      
      float error = prediction - trainingData[i].optimal_timing;
      totalLoss += error * error;
      
      bias -= learningRate * error;
      
      for (int j = 0; j < 8; j++) {
        weights[j] -= learningRate * error * trainingData[i].features[j];
      }
    }
    
    totalLoss /= dataCount;
    Serial.print("Epoch ");
    Serial.print(epoch + 1);
    Serial.print("/");
    Serial.print(epochs);
    Serial.print(" Loss: ");
    Serial.println(totalLoss);
  }
  
  Serial.println("✅ Local model training completed");
  saveModelToSPIFFS();
  sendModelToServer();
}

void controlTrafficLights() {
  long distance1 = getDistance(TRIG1, ECHO1);
  long distance2 = getDistance(TRIG2, ECHO2);
  
  int ir1 = digitalRead(IR_SENSOR1);
  int ir2 = digitalRead(IR_SENSOR2);

  DateTime now;
  int hourOfDay = 12;
  int dayOfWeek = 3;
  
  if (rtc.begin()) {
    now = rtc.now();
    hourOfDay = now.hour();
    dayOfWeek = now.dayOfTheWeek();
  }

  float features[8] = {
    (float)distance1, 
    (float)distance2,
    (float)(ir1 == LOW ? 1 : 0),
    (float)(ir2 == LOW ? 1 : 0),
    (float)vehicleCount1,
    (float)vehicleCount2,
    (float)hourOfDay,
    (float)dayOfWeek
  };

  int optimalGreenTime = predictOptimalTiming(features);
  
  Serial.println("Adjusting Traffic Lights using ML Model...");
  Serial.print("✅ Predicted Optimal Green Time: ");
  Serial.println(optimalGreenTime);

  int traffic1 = (distance1 < 20 ? 1 : 0) + (ir1 == LOW ? 1 : 0) + min(vehicleCount1 / 10, 1);
  int traffic2 = (distance2 < 20 ? 1 : 0) + (ir2 == LOW ? 1 : 0) + min(vehicleCount2 / 10, 1);
  
  if (dataCount < MAX_DATA_POINTS) {
    for (int i = 0; i < 8; i++) {
      trainingData[dataCount].features[i] = features[i];
    }
    trainingData[dataCount].optimal_timing = optimalGreenTime;
    dataCount++;
  }

  int initialVehicleCount1 = vehicleCount1;
  int initialVehicleCount2 = vehicleCount2;
  
  if (traffic1 >= traffic2) {
    digitalWrite(GREEN_LIGHT_1, HIGH);
    digitalWrite(YELLOW_LIGHT_1, LOW);
    digitalWrite(RED_LIGHT_1, LOW);
    digitalWrite(RED_LIGHT_2, HIGH);
    
    for (int i = 0; i < optimalGreenTime; i += 100) {
      countVehicles();
      delay(100);
    }
    
    digitalWrite(GREEN_LIGHT_1, LOW);
    digitalWrite(YELLOW_LIGHT_1, HIGH);
    
    for (int i = 0; i < yellowTime; i += 100) {
      countVehicles();
      delay(100);
    }
    
    digitalWrite(YELLOW_LIGHT_1, LOW);
    digitalWrite(RED_LIGHT_1, HIGH);
    digitalWrite(RED_LIGHT_2, LOW);
    
    for (int i = 0; i < optimalGreenTime; i += 100) {
      countVehicles();
      delay(100);
    }
    
    digitalWrite(YELLOW_LIGHT_1, HIGH);
    digitalWrite(RED_LIGHT_1, LOW);
    
    for (int i = 0; i < yellowTime; i += 100) {
      countVehicles();
      delay(100);
    }
  } else {
    digitalWrite(GREEN_LIGHT_1, LOW);
    digitalWrite(RED_LIGHT_1, HIGH);
    digitalWrite(RED_LIGHT_2, LOW);
    
    for (int i = 0; i < optimalGreenTime; i += 100) {
      countVehicles();
      delay(100);
    }
    
    digitalWrite(YELLOW_LIGHT_1, HIGH);
    digitalWrite(RED_LIGHT_1, LOW);
    
    for (int i = 0; i < yellowTime; i += 100) {
      countVehicles();
      delay(100);
    }
    
    digitalWrite(YELLOW_LIGHT_1, LOW);
    digitalWrite(GREEN_LIGHT_1, HIGH);
    digitalWrite(RED_LIGHT_2, HIGH);
    
    for (int i = 0; i < optimalGreenTime; i += 100) {
      countVehicles();
      delay(100);
    }
    
    digitalWrite(GREEN_LIGHT_1, LOW);
    digitalWrite(YELLOW_LIGHT_1, HIGH);
    
    for (int i = 0; i < yellowTime; i += 100) {
      countVehicles();
      delay(100);
    }
  }
  
  digitalWrite(YELLOW_LIGHT_1, LOW);
  
  int vehiclesPassed1 = vehicleCount1 - initialVehicleCount1;
  int vehiclesPassed2 = vehicleCount2 - initialVehicleCount2;
  
  storeEffectivenessData(optimalGreenTime, vehiclesPassed1, vehiclesPassed2, traffic1, traffic2);
}

void sendTelemetry() {
  if (!mqttClient.connected()) {
    reconnectMQTT();
  }

  long distance1 = getDistance(TRIG1, ECHO1);
  long distance2 = getDistance(TRIG2, ECHO2);
  int ir1 = digitalRead(IR_SENSOR1);
  int ir2 = digitalRead(IR_SENSOR2);

  int traffic_flow = (distance1 < 20 ? 1 : 0) + (distance2 < 20 ? 1 : 0) + 
                    (ir1 == LOW ? 1 : 0) + (ir2 == LOW ? 1 : 0);

  String payload = "{";
  payload += "\"ultrasonic1\":" + String(distance1) + ",";
  payload += "\"ultrasonic2\":" + String(distance2) + ",";
  payload += "\"ir1\":" + String(ir1 == LOW ? 1 : 0) + ",";
  payload += "\"ir2\":" + String(ir2 == LOW ? 1 : 0) + ",";
  payload += "\"traffic_flow\":" + String(traffic_flow) + ",";
  payload += "\"vehicle_count1\":" + String(vehicleCount1) + ",";
  payload += "\"vehicle_count2\":" + String(vehicleCount2) + ",";
  payload += "\"junction_id\":\"" + junctionID + "\",";
  payload += "\"model_bias\":" + String(bias) + ",";
  payload += "\"model_weights_sum\":" + String(weights[0] + weights[1] + weights[2] + weights[3]) + ",";
  payload += "\"training_data_count\":" + String(dataCount);
  payload += "}";

  if (mqttClient.connected()) {
    mqttClient.publish(mqtt_topic, payload.c_str());
    Serial.println("✅ Telemetry data published to MQTT");
  } else {
    Serial.println("❌ Failed to publish telemetry data - MQTT not connected");
  }

  String timestamp;
  if (rtc.begin()) {
    timestamp = getCurrentTimestamp();
  } else {
    timestamp = String(millis());
  }
  
  String path = "/traffic_data/" + junctionID + "/" + timestamp;

  FirebaseJson json;
  json.set("ultrasonic1", distance1);
  json.set("ultrasonic2", distance2);
  json.set("ir1", ir1 == LOW ? 1 : 0);
  json.set("ir2", ir2 == LOW ? 1 : 0);
  json.set("traffic_flow", traffic_flow);
  json.set("vehicle_count1", vehicleCount1);
  json.set("vehicle_count2", vehicleCount2);
  json.set("timestamp", timestamp);
  json.set("model_bias", bias);
  
  for (int i = 0; i < 8; i++) {
    json.set("model_weights/weight" + String(i+1), weights[i]);
  }
  
  json.set("training_data_count", dataCount);

  if (Firebase.setJSON(firebaseData, path, json)) {
    Serial.println("✅ Telemetry data saved to Firebase");
  } else {
    Serial.println("❌ Failed to save telemetry data to Firebase");
  }
}

void checkSensors() {
  long distance1 = getDistance(TRIG1, ECHO1);
  long distance2 = getDistance(TRIG2, ECHO2);
  
  Serial.println("Sensor Status Check:");
  
  if (distance1 >= 0) {
    Serial.println("✅ Ultrasonic Sensor 1 is connected. Distance: " + String(distance1) + " cm");
  } else {
    Serial.println("❌ Ultrasonic Sensor 1 connection issue detected!");
  }
  
  if (distance2 >= 0) {
    Serial.println("✅ Ultrasonic Sensor 2 is connected. Distance: " + String(distance2) + " cm");
  } else {
    Serial.println("❌ Ultrasonic Sensor 2 connection issue detected!");
  }
  
  int ir1 = digitalRead(IR_SENSOR1);
  int ir2 = digitalRead(IR_SENSOR2);
  
  Serial.println("✅ IR Sensor 1 status: " + String(ir1 == LOW ? "Object Detected" : "Clear"));
  Serial.println("✅ IR Sensor 2 status: " + String(ir2 == LOW ? "Object Detected" : "Clear"));
  
  if (rtc.begin()) {
    DateTime now = rtc.now();
    Serial.println("✅ RTC is connected. Current time: " + 
                  String(now.year()) + "/" + 
                  String(now.month()) + "/" + 
                  String(now.day()) + " " + 
                  String(now.hour()) + ":" + 
                  String(now.minute()) + ":" + 
                  String(now.second()));
  } else {
    Serial.println("❌ RTC connection issue detected!");
  }
  
  Serial.println("✅ Total vehicle count - Lane 1: " + String(vehicleCount1) + ", Lane 2: " + String(vehicleCount2));
}

void setupFederatedLearning() {
  if (SPIFFS.begin(true)) {
    Serial.println("✅ SPIFFS mounted successfully");
    loadModelFromSPIFFS();
    
    if (WiFi.status() == WL_CONNECTED) {
      checkForModelUpdates();
    }
  } else {
    Serial.println("❌ Failed to mount SPIFFS");
  }
}

void sendTrafficDataToServer() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("❌ WiFi not connected. Cannot send traffic data to server.");
    return;
  }
  
  HTTPClient http;
  String url = String(FL_SERVER) + "/submit_traffic_data";
  
  http.begin(url);
  http.addHeader("Content-Type", "application/json");
  
  long distance1 = getDistance(TRIG1, ECHO1);
  long distance2 = getDistance(TRIG2, ECHO2);
  int ir1 = digitalRead(IR_SENSOR1);
  int ir2 = digitalRead(IR_SENSOR2);

  int traffic_flow = (distance1 < 20 ? 1 : 0) + (distance2 < 20 ? 1 : 0) + 
                     (ir1 == LOW ? 1 : 0) + (ir2 == LOW ? 1 : 0);
  
  DynamicJsonDocument doc(512);
  doc["junction_id"] = junctionID;
  doc["ultrasonic1"] = distance1;
  doc["ultrasonic2"] = distance2;
  doc["ir1"] = ir1 == LOW ? 1 : 0;
  doc["ir2"] = ir2 == LOW ? 1 : 0;
  doc["traffic_flow"] = traffic_flow;
  doc["vehicle_count1"] = vehicleCount1;
  doc["vehicle_count2"] = vehicleCount2;
  
  String requestBody;
  serializeJson(doc, requestBody);
  
  int httpResponseCode = http.POST(requestBody);
  
  if (httpResponseCode > 0) {
    String response = http.getString();
    Serial.print("✅ Traffic Data Server Response: ");
    Serial.println(response);
  } else {
    Serial.print("❌ Error on sending traffic data: ");
    Serial.println(httpResponseCode);
  }
  
  http.end();
}

void handleFederatedLearningTasks() {
  unsigned long currentMillis = millis();
  
  if (currentMillis - lastModelCheck > MODEL_VERSION_CHECK_INTERVAL) {
    checkForModelUpdates();
    lastModelCheck = currentMillis;
  }
  
  if (currentMillis - lastTrainingTime > LOCAL_TRAIN_INTERVAL && dataCount >= 10) {
    trainLocalModel();
    lastTrainingTime = currentMillis;
  }
}
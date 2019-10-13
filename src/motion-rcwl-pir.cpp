/*********
 * Use rcwl-0516 and a AM312 as motion sensor for MQTT
 * The RCWL is very sensitive and far ranging, and 360 degree
 * the AM312 is less of those attributes. We use it to to start
 * a detection and the rcwl will extend the detection .
 
*********/
#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>

// Replace the next variables with your SSID/Password combination
const char* ssid = "CJCNET";
const char* password = "LostAgain2";

// Add your MQTT Broker IP address, device name, topic 
//const char* mqtt_server = "192.168.1.144";
const char* mqtt_server = "192.168.1.7";
#define MQTT_DEVICE "ESP32_MotionTwo"
#define MQTT_TOPIC "sensors/office/motion2"
#define MQTT_CMD   "sensors/office/motion2"

boolean turnedOn = true;  // controls whether device sends to MQTT

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

hw_timer_t * timer = NULL;
volatile SemaphoreHandle_t timerSemaphore;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

volatile uint32_t isrCounter = 0;
volatile uint32_t lastIsrAt = 0;

// LED Pin - built-in blue led on some boards
//const int led = 2;

// RCWL-0516 Microwave motion sensor
const int motionSensor = 17;
volatile boolean haveMotion = false;      // set by ISR

// AM312 pir motion sensor
#define  pirSensor 16
volatile boolean havePir = false;

#define DelayToOff 25
unsigned int delaySeconds = DelayToOff;   // we may want to adjust this variable dynamically
int motionCount = 0;

// Interrupt handler for motion on AM312 pir
void IRAM_ATTR intrPir() {
  Serial.println("PIR ISR");
  havePir = true;
}

//Interrupt handler for motion on RCWL-0516 pir
void IRAM_ATTR detectsMovement() {
  Serial.println("RCWL ISR");
  haveMotion = true;
}

void IRAM_ATTR onTimer(){
  // Increment the counter and set the time of ISR
  portENTER_CRITICAL_ISR(&timerMux);
  isrCounter++;
  lastIsrAt = millis();
  portEXIT_CRITICAL_ISR(&timerMux);
  // Give a semaphore that we can check in the loop
  xSemaphoreGiveFromISR(timerSemaphore, NULL);
}

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin((char *)ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  // convert byte* to String
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    messageTemp += (char)payload[i];
  }
  Serial.println();

  // If a message is received on the topic 
  // check if the message is either "enabled", "disabled" or
  // 'active', 'inactive' since we get the what we sent. Problem?
  // Set 'turnedOn' appropriately
  if (String(topic) == MQTT_CMD) {
    if(messageTemp == "enable") {
      turnedOn = true;
      Serial.println("mqtt sent an enable");
    } else if (messageTemp == "disable") {
      turnedOn = false;
      Serial.println("mqtt sent a disable");
    }
  }
}

void setup() {
  Serial.begin(115200);

  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  // Create semaphore to inform us when the timer has fired
  timerSemaphore = xSemaphoreCreateBinary();

  // Use 1st timer of 4 (counted from zero).
  // Set 80 divider for prescaler (see ESP32 Technical Reference Manual for more
  // info).
  timer = timerBegin(0, 80, true);

  // Attach onTimer function to our timer.
  timerAttachInterrupt(timer, &onTimer, true);

  // Set alarm to call onTimer function every second (value in microseconds).
  // Repeat the alarm (third parameter)
  timerAlarmWrite(timer, 1000000, true);

  // Start an alarm
  timerAlarmEnable(timer);

  //motionSemaphore = xSemaphoreCreateBinary();
  // PIR Motion Sensor mode INPUT_PULLUP
  pinMode(pirSensor, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pirSensor), intrPir, RISING);
  
  // Microwave Motion Sensor mode INPUT_PULLUP
  pinMode(motionSensor, INPUT_PULLUP);
  // Set motionSensor pin as interrupt, assign interrupt function and set RISING mode
  attachInterrupt(digitalPinToInterrupt(motionSensor), detectsMovement, RISING);

  //pinMode(led, OUTPUT);
  //digitalWrite(led, LOW);
}



void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(MQTT_DEVICE)) {
      Serial.println("connected");
      // Subscribe
      client.subscribe(MQTT_CMD);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // wait for the haveMotion var to be set by the ISR
  if (haveMotion && havePir) {
    //portENTER_CRITICAL(&motionMux);
    // publish to MQTT
    if (turnedOn) 
      client.publish(MQTT_TOPIC, "active");
    haveMotion = false;
    havePir = false;
    // begin countdown to responding
    motionCount = delaySeconds;     // while motion, keep delaying
    //portEXIT_CRITICAL(&motionMux);
    //digitalWrite(led, HIGH);
    Serial.println("Motion BEGIN");
  }
  // If one second Timer has fired
  if (turnedOn && xSemaphoreTake(timerSemaphore, 0) == pdTRUE) {
    uint32_t isrCount = 0;
    // Read the interrupt count and time
    portENTER_CRITICAL(&timerMux);
    isrCount = isrCounter;
    portEXIT_CRITICAL(&timerMux);
    Serial.print("Tick ");
    Serial.print(isrCount);
    if (motionCount > 0) {
      motionCount--;
      if (motionCount == 0) {
        // publish to MQTT
        if (turnedOn) 
          client.publish(MQTT_TOPIC, "inactive");
        Serial.println(" Motion END");
        //digitalWrite(led, LOW);
      } else
        Serial.print(" counting down from ");
        Serial.println(motionCount);
    } else 
      Serial.println(" no motion");
  } 
}

/*
 * A simple PIR ISR Test Routine
 *
 * ESP8266 Module used is a Wemos D1 Mini Pro (16MB) clone, from
 * https://www.tindie.com/products/mostlyrobots/wemos-d1-mini-pro-16mb-with-external-antenna/
 *
 * Although the code indicates a HC-SR501 sensor, the actual sensor used
 * is a PYE_IR sensor module from Keyestudio. Data available from
 * https://wiki.keyestudio.com/Ks0052_keyestudio_PIR_Motion_Sensor
 *
 * Whilst learning to code ESP-8266 devices and Webthings, the following URL were
 * used as reference.
 *
 * https://randomnerdtutorials.com/esp8266-pinout-reference-gpios/
 * https://randomnerdtutorials.com/interrupts-timers-esp8266-arduino-ide-nodemcu/
 * https://web-relays.com/en/blog/esp8266-multiple-ssids-using-one-esp8266-module/
 * https://tttapa.github.io/ESP8266/Chap04%20-%20Microcontroller.html
 * https://www.makerguides.com/hc-sr501-arduino-tutorial/
 * https://github.com/Diallomm/webthings_pushbutton_esp32
 * https://randomnerdtutorials.com/esp8266-nodemcu-static-fixed-ip-address-arduino/
 * https://microcontrollerslab.com/esp8266-interrupts-timers-arduino-ide-nodemcu/
 * http://docs.iot-bus.com/en/latest/mozilla-iot-examples/mozilla-iot-bus-hcsr501-pir-thing.html
 *
 * File last updated update 19 Dec 2022
 * 
 */

/*
 * Adding some WoT Stuff that will be required in the future.
 */

#define LARGE_JSON_BUFFERS 1
#define MAX_MOTION 1024

#include <Arduino.h>
#include <Thing.h>
#include <WebThingAdapter.h>

// TODO: Hardcode your wifi credentials here (and keep it private)
const char *ssid = "networkSSID";
const char *password = "networkPassword";
volatile bool blink = true;

WebThingAdapter *adapter;

// WoT Motion JSON definitions
const char *sensorTypes[] = {"MotionSensor", nullptr}; // define a type
ThingDevice hcsr501("HC-SR501", "PIR Motion Sensor", sensorTypes); // ID, name and type
ThingProperty sensorOn("on", "", BOOLEAN, "MotionProperty");
// Set motion events as level-property, an added extra
ThingProperty sensorEvents("events", "Motion Events", NUMBER, "LevelProperty");
ThingPropertyValue sensorValue;


// defines for the onboard LED
//
#if defined(LED_BUILTIN)
const int led_pin = LED_BUILTIN;
#else
const int led_pin = 13; // manually configure LED pin
#endif

// pin definitions
//
// Wemos D1 Mini Pro Definitions
const byte sensor_pin = 5;  // D1 on Wemos D1 Mini PCB, Interrupt Pin
const byte led_blip = 16;   // D0 on Wemos D1 Mini PCB, off board LED

// Wio Link Definitions
//const byte sensor_pin = 14;  // D1 on Wemos D1 Mini PCB, Interrupt Pin
//const byte led_blip = 12;   // D0 on Wemos D1 Mini PCB, off board LED

// some required defines
//
#define blipOFF() (digitalWrite(led_blip, HIGH))
#define blipON() (digitalWrite(led_blip, LOW))

#define ledOFF() (digitalWrite(led_pin, HIGH))
#define ledON() (digitalWrite(led_pin, LOW))

// setup required variables
//
volatile byte intCounter = 0;
int numberOfInterrupts = 0;
//the time we give the sensor to calibrate (10-60 secs according to the datasheet)
int calibrationTime = 300; 
const long interval = 1000;
//unsigned long current_time = millis();
//unsigned long last_trigger = 0;
//boolean timer_on = false;
volatile boolean irqPIR = false;
//volatile boolean motion_detected = false;

// the pin interrupt service routine
//
IRAM_ATTR void motionDetected() {
  irqPIR = true;
  intCounter++;
  blipON();
//  timer_on = true;
//  last_trigger = millis();
}

void configurePins (void) {
  pinMode(led_pin, OUTPUT);
  digitalWrite(led_pin, LOW);
  pinMode(led_blip, OUTPUT);
  digitalWrite(led_blip, LOW);
 
  pinMode(sensor_pin, INPUT);
  digitalWrite(sensor_pin, LOW);   // disable internal pullup on pin.
}

void sensorCalTime(void) {
  //give the sensor some time to calibrate
  Serial.print("calibrating sensor ");
    for(int i = 0; i < calibrationTime; i++){
      Serial.print(".");
      digitalWrite(led_blip, blink ? LOW : HIGH); // Use Blip LED to indicate startup
      blink = !blink;
      delay(100);
      }
}

void setup() {

  delay(3000);
  
  Serial.begin(115200);
  Serial.flush();

  configurePins();
  sensorCalTime();

  ledOFF();
  blipOFF();
  
  Serial.println();
  Serial.println("!done");
  Serial.println("SENSOR ACTIVE");

  // configure WiFi from here...
  //
  Serial.println("");
  Serial.print("Connecting to \"");
  Serial.print(ssid);
  Serial.println("\"");
#if defined(ESP8266) || defined(ESP32)
  WiFi.mode(WIFI_STA);
#endif
  WiFi.begin(ssid, password);
  Serial.println("");

  // Wait for connection
  //
  bool blink = true;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    digitalWrite(led_blip, blink ? LOW : HIGH); // active low led
    blink = !blink;
  }
  ledOFF();

  // show wifi status
  //
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  adapter = new WebThingAdapter("adapter", WiFi.localIP());
  
  delay(1000);

  // set unit for count of events
  //
  sensorEvents.title = "Motion Events";
  sensorEvents.readOnly = "true";

  sensorOn.title = "Motion Trigger";
  sensorOn.readOnly = "true";
  
  hcsr501.addProperty(&sensorOn);
  hcsr501.addProperty(&sensorEvents);
  adapter->addDevice(&hcsr501);
  adapter->begin();

  Serial.println("HTTP server started");
  Serial.print("http://");
  Serial.print(WiFi.localIP());
  Serial.print("/things/");
  Serial.println(hcsr501.id);
  
  attachInterrupt(digitalPinToInterrupt(sensor_pin), motionDetected, RISING);
  Serial.println("Interrupts enabled");
}

void loop() {
//
//  current_time = millis();

  int sensorState = digitalRead(sensor_pin);
  
  if(sensorState == HIGH) {  
  //if((intCounter > 0) && irqPIR) {
    if(irqPIR) {
      intCounter--;
      numberOfInterrupts++;
      irqPIR = false;
    }

    if(numberOfInterrupts > MAX_MOTION) {
      numberOfInterrupts = 1;
    }

    if(sensorOn.getValue().boolean != true) {
      sensorValue.boolean = true;
      Serial.println("Motion = 1");
      Serial.print("Events: ");
      Serial.println(numberOfInterrupts);     
    }   
  } else {
    if(sensorOn.getValue().boolean == true) {
      Serial.println("Motion = 0");
      sensorValue.boolean = false;
      blipOFF();
    }
  }

  sensorOn.setValue(sensorValue);
  sensorValue.number = numberOfInterrupts;
  sensorEvents.setValue(sensorValue);
  adapter->update();

  delay(interval);
//
//  do {
//  } while(current_time - last_trigger < interval);
}

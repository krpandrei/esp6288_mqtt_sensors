/*
 https://github.com/akkerman/espthp

#include <DHT.h>
#include <RunningAverage.h>
#include <Ticker.h>
#include "config.h"
 */
 
#define SERIAL_VERBOSE
#define WIFI
#define BME
#define MQTT
 
#include <math.h>              //Mathematical functions
#include <ESP8266WiFi.h>       //esp8266 Wifi support
#include <ESP8266mDNS.h>       //Multicast DNS library
#include <WiFiUdp.h>           //WiFi UDP
#include <PubSubClient.h>      //Client for MQTT
#include <Wire.h>              //I2C functions
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "user_interface.h"

/************ WIFI and MQTT INFORMATION (CHANGE THESE FOR YOUR SETUP) ******************/
#define HOSTNAME "KRP-"
String hostname(HOSTNAME);
IPAddress ip;

#define WIFI_SSID "KRP"
#define WIFI_PASS "m@st3rn3tw0rk"

#define MQTT_IP "192.168.100.10"
#define MQTT_PORT 1883

#define STATUS_DISCONNECTED "disconnected"
#define STATUS_ONLINE "online"
/************* MQTT TOPICS (change these topics as you wish)  **************************/
const String chipId = String(ESP.getChipId());
const String locationTopic = "test";
const String baseTopic = "sensors/" + chipId + "/";
const String tempTopic = baseTopic + "temperature";
const String humiTopic = baseTopic + "humidity";
const String presTopic = baseTopic + "pressure";
const String altiTopic = baseTopic + "altitude";
const String willTopic = baseTopic + "status";
const String vccTopic  = baseTopic + "vcc";
const String batTopic  = baseTopic + "battery";
const String batpTopic = baseTopic + "batteryp";
const String ipTopic   = baseTopic + "ip";
/******************************** PIN DEFINITIONS *************************************************/
#define SCLPIN 5 //D1
#define SDAPIN 4 //D2


/******************************** SENSOR DEFINITIONS **********************************************/

String s_temperature = "0.0";   //*C
String s_humidity = "0.0";      //%
String s_pressure = "0.0";      //hPa
String s_altitude = "0.0";      //m
String s_vcc = "0.0";           //V
String s_b = "0.0";             //V
String s_bp = "0";              //%

#define SEALEVELPRESSURE_HPA (1027.7)

#define VMIN 1900
#define VMAX 3300

/******************************** GLOBALS for fade/flash ******************************************/
#define LED_BUILTIN 2                   //D4
#define BRIGHT    350                   //max led intensity (1-500)
#define INHALE    1250                  //Inhalation time in milliseconds.
#define PULSE     INHALE*1000/BRIGHT
#define REST      1000                  //Rest Between Inhalations.
/******************************** GLOBALS *********************************************************/
#define MY_BAUD_RATE 115200
// Uncomment the next line for verbose output over UART.


#define SLEEP_USE  1
/*
 * we need to tie the RST pin to GPIO 16 (D0)
 */
int SLEEP_TIME = 600;
float DELAY_TIME = 60.0;
unsigned long previousMillis = 0;         //The last time of measurement




ADC_MODE(ADC_VCC);

WiFiClient WiFiClient;
PubSubClient client(WiFiClient);
Adafruit_BME280 bme; // I2C

//[SETUP]###########################################################################################
void setup() {
  Serial.println("\r\nsetup()");

  pinMode(LED_BUILTIN, OUTPUT); //GPIO16
  
  #ifdef SERIAL_VERBOSE
  Serial.begin(MY_BAUD_RATE);
  delay(10);
  Serial.setTimeout(2000);
  // Wait for serial to initialize.
  while (!Serial) { }

  // Try pushing frequency to 160MHz.
  //system_update_cpu_freq(SYS_CPU_160MHZ);
  
  Serial.println("\r\n");
  Serial.print("Chip ID: 0x");
  Serial.println(ESP.getChipId(), HEX);
  #endif

  #ifdef WIFI
  //WiFi
  setup_wifi();
  #endif

  #ifdef BME
  //Sensor
  Wire.begin(SDAPIN, SCLPIN);
  Wire.setClock(100000);
  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
  #endif

  #ifdef MQTT
  //MQTT
  client.setServer(MQTT_IP, MQTT_PORT);
  client.setCallback(callback);
  #endif
}

//[LOOP]############################################################################################

bool publishNow = false;

void loop() {
  
  yield();
  
  #ifdef MQTT
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  #endif
  
  #ifndef BME
  bmeRead();
  #endif
  
  vccRead();
  //batteryRead();

  unsigned long currentMillis = millis();
  if(currentMillis - previousMillis > DELAY_TIME*1000) {
    #ifdef MQTT
    publish(3);
	#endif
    previousMillis = millis();
  }
  
  // Add a 1 second delay. 
  delay(1000); //just here to slow down the output.
   
}

//[OTHER]###########################################################################################

float bmeRead() {
  float t = bme.readTemperature();
  float h = bme.readHumidity();
  float p = bme.readPressure()/100.0F;
  float a = bme.readAltitude(SEALEVELPRESSURE_HPA);

  s_temperature = String(t);
  s_humidity = String(h);
  s_pressure = String(p);
  s_altitude = String(a);
  
  Serial.print("Temperature: "); Serial.print(t); Serial.println(" *C");
  Serial.print("Humidity   : "); Serial.print(h); Serial.println(" %");
  Serial.print("Pressure   : "); Serial.print(p); Serial.println(" hPa");
  Serial.print("Altitude   : "); Serial.print(a); Serial.println(" m");

  return t+h+p;
}

float vccRead() {
  float v  = ESP.getVcc() / 1000.0;

  s_vcc = String(v);
  
  Serial.print("Vcc        : "); Serial.print(v); Serial.println(" V");
  
  return v;
}

/*--
long batteryRead() {
  long b = readVcc();
  uint8_t bp = constrain(map(b,VMIN,VMAX,0,100),0,255);    
  
  s_b = String(b);
  s_bp = String(bp);
  
  Serial.print("Vcc battery: "); Serial.print(b); Serial.println(" V");
  Serial.print("Vcc percent: "); Serial.print(bp); Serial.println(" %");
  
  return b;
}

// function for reading Vcc by reading 1.1V reference against AVcc. Based from http://provideyourown.com/2012/secret-arduino-voltmeter-measure-battery-voltage/
// To calibrate reading replace 1125300L with scale_constant = internal1.1Ref * 1023 * 1000, where internal1.1Ref = 1.1 * Vcc1 (per voltmeter) / Vcc2 (per readVcc() function) 
long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both

  long result = (high<<8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}
--*/

void publish(int cycles) {
  Serial.println();
  Serial.println("-----");

  for (int c = 1; c <= cycles; c++) {
    Serial.print("MQTT: publishing values... [");
    Serial.print(c);
    Serial.print("]");
    Serial.println();
	
	#ifndef BME
	bmeRead();
    client_publish(tempTopic, s_temperature, "*C");
    client_publish(humiTopic, s_humidity, "%");
    client_publish(presTopic, s_pressure, "hPa");
    client_publish(altiTopic, s_altitude, "m");
	#endif
	
    vccRead();
    client_publish(vccTopic, s_vcc, "V");
	
	//batteryRead();
	//client_publish(batTopic, s_b, "V");
	//client_publish(batpTopic, s_bp, "%");
	
	delay(500);
  }
  
  Serial.println("-----");
  gotoSleep(SLEEP_TIME);
}

void client_publish(String topic, String payload, String metric) {
  Serial.print("[");
  Serial.print(topic);
  Serial.print("] -> ");
  Serial.print(payload);
  Serial.print(" ");
  Serial.print(metric);
  Serial.println();
  client.publish(topic.c_str(), payload.c_str());
  
  delay(500);
}

//MQTT callback
// This functions is executed when some device publishes a message to a topic that your ESP8266 is subscribed to
// Change the function below to add logic to your program, so when a device publishes a message to a topic that 
// your ESP8266 is subscribed you can actually do something
void callback(char* topic, byte* payload, unsigned int length) {
  std::string s( reinterpret_cast<char const*>(payload), length );
  Serial.print("Message arrived on topic [");
  Serial.print(topic);
  Serial.print("] - ");
  Serial.print(s.c_str());
  Serial.println();
  if (s == "all" || s == chipId.c_str()) {
    publish(3);
  }
}

//MQTT reconnected
// This functions reconnects your ESP8266 to your MQTT broker
// Change the function below if you want to subscribe to more topics with your ESP8266 
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    // connect (clientID, willTopic, willQoS, willRetain, willMessage)
    if (client.connect(chipId.c_str(), willTopic.c_str(), 1, true, STATUS_DISCONNECTED)) {
      Serial.println("connected");
      Serial.println();
      client.publish(willTopic.c_str(), STATUS_ONLINE, true);
      client.publish(ipTopic.c_str(), ip.toString().c_str(), true);
      client.subscribe("config/publish");
      publishNow = true;
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      Serial.println();
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void breathingLED () {
  //ramp increasing intensity, Inhalation: 
  for (int i=1;i<BRIGHT;i++){
    digitalWrite(LED_BUILTIN, LOW);          // turn the LED on.
    delayMicroseconds(i*10);                 // wait
    digitalWrite(LED_BUILTIN, HIGH);         // turn the LED off.
    delayMicroseconds(PULSE-i*10);           // wait
    delay(0);                                //to prevent watchdog firing.
  }
  //ramp decreasing intensity, Exhalation (half time):
  for (int i=BRIGHT-1;i>0;i--){
    digitalWrite(LED_BUILTIN, LOW);          // turn the LED on.
    delayMicroseconds(i*10);                 // wait
    digitalWrite(LED_BUILTIN, HIGH);         // turn the LED off.
    delayMicroseconds(PULSE-i*10);           // wait
    i--;
    delay(0);                                //to prevent watchdog firing.
  }
  delay(REST);                               //take a rest... 
}

void blinkSuccess() {
  for (int i = 4; i < 50; i = (5 * i) >> 2) {
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED on
    delay(10 * i);                     // wait
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED off
    delay(10 * i);                     // wait
  }
}

void blinkError() {
  for (int i = 0; i < 28; i++) {
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED on
    delay(125);                        // wait
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED off
    delay(125);                        // wait
  }
}

void gotoSleep(int sec_duration) {
  if(SLEEP_USE == 1){
    Serial.println("Going to sleep...");
    client.disconnect();
    WiFi.disconnect();
    //delay(100);
    /*
    mode is one of WAKE_RF_DEFAULT, WAKE_RFCAL, WAKE_NO_RFCAL, WAKE_RF_DISABLED.
    WAKE_RF_DEFAULT = 0, // RF_CAL or not after deep-sleep wake up, depends on init data byte 108.
    WAKE_RFCAL = 1,      // RF_CAL after deep-sleep wake up, there will be large current.
    WAKE_NO_RFCAL = 2,   // no RF_CAL after deep-sleep wake up, there will only be small current.
    WAKE_RF_DISABLED = 4 // disable RF after deep-sleep wake up, just like modem sleep, there will be the smallest current.
    (GPIO16 needs to be tied to RST to wake from deepSleep.)
    */
    ESP.deepSleep(sec_duration * 1000 * 1000, WAKE_RF_DEFAULT);
    //delay(1000);   // wait for deep sleep to happen
    //delayMicroseconds(1000000);  //Simple 1 Sec Delay

    // Deep sleep mode until RESET pin is connected to a LOW signal (for example pushbutton or magnetic reed switch)
    //Serial.println("I'm awake, but I'm going into deep sleep mode until RESET pin is connected to a LOW signal");
    //ESP.deepSleep(0); 

    //Serial.println("Sleep failed.");
    //while (1) {
    //  blinkError();
    //}
  }
}

bool checkBound(float newValue, float prevValue, float maxDiff) {
  return !isnan(newValue) &&
         (newValue < prevValue - maxDiff || newValue > prevValue + maxDiff);
}

//WiFi connect
void setup_wifi() {
  hostname += String(ESP.getChipId(), HEX);
  WiFi.hostname(hostname);

  // Print hostname.
  Serial.println("Hostname: " + hostname);
  //Serial.println(WiFi.hostname());
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  int retries = 15;
  while (WiFi.status() != WL_CONNECTED && --retries > 0) {
    delay(500);
    Serial.print(".");
    breathingLED ();   
  }
  if (retries == 0) {
    blinkError();
    ESP.restart();
  /*
  ESP.restart() tells the SDK to reboot, so its a more clean reboot, use this one if possible.
  the boot mode:(1,7) problem is known and only happens at the first restart after serial flashing.
  if you do one manual reboot by power or RST pin all will work
  */
  }

  Serial.println("");
  Serial.println("WiFi connected");
  blinkSuccess();
  Serial.print("IP address: ");
  ip = WiFi.localIP();
  Serial.println(ip);
}



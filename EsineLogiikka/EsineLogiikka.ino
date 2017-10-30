/**************************************************************************************
   Sketch to interact with users of Tulevaisuuden Esine artwork.
   Copyright 2017 Aapo Rista / Vekotinverstas / Forum Virium Helsinki / MIT license

   The idea is to read one I2C sernsor and based on sensor values change colors
   in digital RGB LED strip. This sketch supports sensors listed below (one at a time):
   - gesture / rgb light sensor
   - IR thermometer
   - LUX meter (BH1750)
   - temperature / humidity sensor
   - a button
 **************************************************************************************/

#include "settings.h"
#include <Wire.h>
#include <BH1750.h>       // https://github.com/claws/BH1750
#include "SparkFun_Si7021_Breakout_Library.h"  // https://github.com/sparkfun/Si7021_Breakout
#include <SparkFun_APDS9960.h>
#include <FastLED.h>
#include <PubSubClient.h>
#include <ESP8266WiFi.h>
#include <ArduinoJson.h>

void callback(char* topic, byte* payload, unsigned int length) {
  // handle message arrived
  // Serial.println(payload);
}

WiFiClient wifiClient;
PubSubClient client(MQTT_SERVER, 1883, callback, wifiClient);
bool wifiReconnect = false;

uint32_t lastMsgTime = 0;

// I2C settings
#define SDA     D2
#define SCL     D1

// FastLED settings
#define LED_PIN     D7  // 13    // D7
#define CLK_PIN     D5  // 14    // D5
#define NUM_LEDS    60
#define BRIGHTNESS  255
//#define LED_TYPE    WS2811
#define LED_TYPE    LPD8806
#define COLOR_ORDER GRB
#define UPDATES_PER_SECOND 20
CRGB leds[NUM_LEDS];

// Modes
#define S_BUTTON 0
#define S_IR_TEMP 1
#define S_TEMP_HUMIDITY 2
#define S_LUX_METER 3
#define S_GESTURE_RGB 4
uint8_t currentMode = S_BUTTON;

// Button settings
const byte interruptPin = D4; // 2; // D4
volatile byte interruptCounter = 0;
int numberOfInterrupts = 0;
long lastInterruptTime = 0;

const byte interruptPin2 = D2;
volatile byte interruptCounter2 = 0;
int numberOfInterrupts2 = 0;
long lastInterruptTime2 = 0;


CRGBPalette16 currentPalette;
TBlendType    currentBlending;

extern CRGBPalette16 myRedWhiteBluePalette;
extern const TProgmemPalette16 myRedWhiteBluePalette_p PROGMEM;
long cnt = 0;

// IR settings
float ir_ambient_temp = -273.15;
float ir_object_temp = -273.15;

// LUX meter
BH1750 lightMeter(0x23);
uint16_t lux = -1;

// RGB meter, Global Variables
#define APDS9960_INT D5 // Needs to be an interrupt pin
SparkFun_APDS9960 apds = SparkFun_APDS9960();
int isr_flag = 0;
uint16_t ambient_light = 0;
uint16_t red_light = 0;
uint16_t green_light = 0;
uint16_t blue_light = 0;

Weather ht_sensor;

DEFINE_GRADIENT_PALETTE( infrared_gp ) {
  0,   255,  0,  255,  // -28 purple
  56,    0,  0,  255,  //   0 blue
  80,   0, 255,   0,  //  22 green
  //101, 255,   0,   0,  //  37 red
  108, 255, 255,   0,  //  30 yellow
  //112, 255, 128,   0,  //  30 yellow
  112, 255,   0,   0,  //  37 red
  156,   0, 255, 255,  //  50 cyan
  255, 255, 255, 255   //full white
}; 

DEFINE_GRADIENT_PALETTE( lux_gp ) {
    0,   0,  0,  255,  //   0 blue
  100,   0, 255,   0,  //  22 green
  120, 255, 128,   0,  //  30 yellow
  140, 255,   0,   0,  //  37 red
  255, 255, 255, 255   //  full white
}; 

DEFINE_GRADIENT_PALETTE( humi_gp ) {
    0,    0,  0,  255, //   0 blue
   20, 255, 255,   0,  //  20 yellow
   89,   0, 255,   0,  //  30 green
  127,   0, 255,   0,  //  30 green
  255, 255,   0,   0   // 100 red
}; 



String macToStr(const uint8_t* mac)
{
  String result;
  for (int i = 0; i < 6; ++i) {
    result += String(mac[i], 16);
    if (i < 5)
      result += ':';
  }
  return result;
}

void WifiSetup() {
  uint32_t wifi_start = millis();
  FillLEDsFromStaticColor(0,0,250);   
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  // TODO: quit connecting after e.g. 20 seconds
  delay ( 500 );
  while (WiFi.status() != WL_CONNECTED) {
    // TODO: blink leds here
    delay ( 250 );
    FillLEDsFromStaticColor(100,100,0); 
    Serial.print ( "." );
    delay ( 250 );
    FillLEDsFromStaticColor(0,0,0); 
    if ((millis() - wifi_start) > 20*1000) {
      Serial.println("");
      Serial.println("WiFi connect failed");
      // TODO: show error message in leds
      for (int i = 0; i < 10; i++) {
        delay ( 250 );
        FillLEDsFromStaticColor(200,0,0); 
        Serial.print ( "." );
        delay ( 250 );
        FillLEDsFromStaticColor(200,200,0); 
      }
      return;
    }
  }
  wifiReconnect = true;  // we got once connected
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void MqttSetup() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("No WiFi so no MQTT. Continuing.");
    return;
  }
  // Generate client name based on MAC address and last 8 bits of microsecond counter
  String clientName;
  clientName += "esp8266-";
  uint8_t mac[6];
  WiFi.macAddress(mac);
  clientName += macToStr(mac);
  clientName += "-";
  clientName += String(micros() & 0xff, 16);

  Serial.print("Connecting to ");
  Serial.print(MQTT_SERVER);
  Serial.print(" as ");
  Serial.println(clientName);

//  if (client.connect((char*) clientName.c_str())) {
  if (client.connect((char*) clientName.c_str(), MQTT_USER, MQTT_PASSWORD)) {
    
    Serial.println("Connected to MQTT broker");
    Serial.print("Topic is: ");
    Serial.println(MQTT_TOPIC);

    /*
        if (client.publish(MQTT_TOPIC, "hello from ESP8266")) {
          Serial.println("Publish ok");
        }
        else {
          Serial.println("Publish failed");
        }
    */
  }
  else {
    Serial.println("MQTT connect failed");
    Serial.println("Will reset and try again...");
    // TODO: quit connecting after e.g. 20 seconds to enable standalone usage
    // abort();
  }
}

void setup() {
  Wire.begin(SDA, SCL);
  Serial.begin(115200);
  Serial.println();
  Serial.println();
  Serial.println("Waiting 1 sec...");
  delay( 1000 ); // power-up safety delay
  Serial.println("Init FastLED");
  //FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip );
  FastLED.addLeds<LED_TYPE, LED_PIN, CLK_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip );

  FastLED.setBrightness( BRIGHTNESS );

  WifiSetup();
  MqttSetup();

  //currentPalette = RainbowColors_p;
  currentPalette = infrared_gp;
  currentBlending = LINEARBLEND;
  //currentBlending = NOBLEND;

  // Check which sensor is connected
  Serial.println("Setup: Polling IR thermometer");
  if (readObjectTempC(0x5A) < 500) {
    currentMode = S_IR_TEMP;
    currentPalette = infrared_gp;
    Serial.print("Setup: Found IR thermometer: ");
    Serial.println(readObjectTempC(0x5A));
  }

  Serial.println("Setup: Polling BH1750 lightmeter");
  lightMeter.begin(BH1750_CONTINUOUS_HIGH_RES_MODE);
  uint16_t lux = lightMeter.readLightLevel();
  if (lux >= 0 && lux < 54612) {
    currentMode = S_LUX_METER;
    currentPalette = lux_gp;
    Serial.print("Setup: Found LUX meter: ");
    Serial.println(lux);
  }

  Serial.println("Setup: Polling Si xxxxx");
  ht_sensor.begin();
  float humidity = ht_sensor.getRH();
  Serial.println(humidity);
  if (humidity > 0.0 && humidity <= 150.0) {
    currentMode = S_TEMP_HUMIDITY;
    currentPalette = humi_gp;
    Serial.print("Setup: Found HUMI meter: ");
    Serial.println(humidity);
  }
  //uint16_t lux = lightMeter.readLightLevel();

  Serial.println("Setup: Polling RGB");
  // Initialize APDS-9960 (configure I2C and initial values)
  if ( apds.init() ) {
    Serial.println(F("APDS-9960 initialization complete"));
    // Start running the APDS-9960 light sensor (no interrupts)
    if ( apds.enableLightSensor(false) ) {
      Serial.println(F("Light sensor is now running"));
      currentMode = S_GESTURE_RGB;
      // Wait for initialization and calibration to finish
      delay(500);
      /*
      // Start running the APDS-9960 gesture sensor engine
      if ( apds.enableGestureSensor(true) ) {
        Serial.println(F("Gesture sensor is now running"));
      } else {
        Serial.println(F("Something went wrong during gesture sensor init!"));
      } 
      */ 
    } else {
      Serial.println(F("Something went wrong during light sensor init!"));
    }

  } else {
    Serial.println(F("Something went wrong during APDS-9960 init!"));
  }
  Serial.print("Current mode: ");
  Serial.println(currentMode);
  
}


void loop()
{
  if (!client.loop()) {
    Serial.print("Client disconnected...");
    // TODO: increase reconnect from every loop() to every 60 sec or so
    MqttSetup();
  }
  ShowCurrentEffect();
  FastLED.show();
  FastLED.delay(1000 / UPDATES_PER_SECOND);
}


void FillLEDsFromPaletteStaticColor( uint8_t colorIndex)
{
  uint8_t brightness = 255;
  for ( int i = 0; i < NUM_LEDS; i++) {
    leds[i] = ColorFromPalette( currentPalette, colorIndex, brightness, currentBlending);
  }
}

void FillLEDsFromStaticColor( uint8_t r, uint8_t g, uint8_t b)
{
  uint8_t brightness = 255;
  CRGB c = CRGB(r, g, b);
  for ( int i = 0; i < NUM_LEDS; i++) {
    leds[i] = c;
  }
  FastLED.show();
}

void SendDataToMQTT(char sensor[], char type1[], float val, char type2[], float val2, char type3[], float val3) {
  StaticJsonBuffer<200> jsonBuffer;
  char jsonChar[200];
  JsonObject& root = jsonBuffer.createObject();
  root["chipid"] = ESP.getChipId();
  root["sensor"] = sensor;
  root["millis"] = millis();
  JsonArray& data = root.createNestedArray("data");
  data.add(type1);
  data.add(val);
  data.add(type2);
  data.add(val2);
  data.add(type3);
  data.add(val3);
  root.printTo(jsonChar);
  client.publish(MQTT_TOPIC, jsonChar);
  Serial.println(jsonChar);
}

void ShowCurrentEffect() {
  char* sensor;
  char* type;
  float val;
  char* type2;
  float val2;
  char* type3;
  float val3;

  uint8_t brightness = 255;

  if (currentMode == S_IR_TEMP)  {
    sensor = "irtemp";
    type = "temp";
    val = readObjectTempC(0x5A);
    type2 = "_";
    val2 = 0;
    type3 = "_";
    val3 = 0;
    float colorIndex = map(val * 100, -28 * 100, 100 * 100, 0, 255);
    if (colorIndex < 0) {colorIndex = 0;}
    if (colorIndex > 255) {colorIndex = 255;}
    Serial.print(val);
    Serial.print("\t");
    Serial.println(colorIndex);
    uint8_t brightness = 255;
    for ( int i = 0; i < NUM_LEDS; i++) {
      leds[i] = ColorFromPalette( currentPalette, colorIndex, brightness, currentBlending);
    }
  } else

  if (currentMode == S_TEMP_HUMIDITY)  {
    uint8_t brightness = 100;
    sensor = "humitemp";
    type = "humi";
    type2 = "temp";
    val = ht_sensor.getRH();
    val2 = ht_sensor.getTemp();
    type3 = "_";
    val3 = 0;

    float colorIndex = map(val * 100, 0 * 100, 100 * 100, 0, 255);
    Serial.print(val);
    Serial.print(" %, Humi map ");
    Serial.println(colorIndex);
    for ( int i = 0; i < NUM_LEDS; i++) {
      leds[i] = ColorFromPalette( currentPalette, (int)colorIndex, brightness, currentBlending);
    }
  } else

  if (currentMode == S_LUX_METER)  {
    sensor = "lux";
    type = "luxi";
    uint8_t brightness = 100;
    val = lightMeter.readLightLevel();
    type2 = "_";
    val2 = 0;
    type3 = "_";
    val3 = 0;
    uint32_t _val = val;
    if (_val <= 0) {
      _val = 1;
    }
    _val = log(_val) * 20;  // log(65000) < 12, so 12*20 = max 240
    // float colorIndex = map(_val, 0, 12, 0, 255);
    float colorIndex = _val;
    Serial.print(val);
    Serial.print(" lx, lux map ");
    Serial.println(colorIndex);
    for ( int i = 0; i < NUM_LEDS; i++) {
      leds[i] = ColorFromPalette( currentPalette, (int)colorIndex, brightness, currentBlending);
    }
  } else

  if (currentMode == S_GESTURE_RGB)  {
    //create some variables to store the color data in
    uint16_t r, g, b, c;
    sensor = "rgbgest";
    type = "r";
    type2 = "g";
    type3 = "b";
    
    if (  !apds.readAmbientLight(ambient_light) ||
          !apds.readRedLight(red_light) ||
          !apds.readGreenLight(green_light) ||
          !apds.readBlueLight(blue_light) ) {
      Serial.println("Error reading light values");
    } else {
      Serial.print("Ambient: ");
      Serial.print(ambient_light);
      Serial.print(" Red: ");
      Serial.print(red_light);
      Serial.print(" Green: ");
      Serial.print(green_light);
      Serial.print(" Blue: ");
      Serial.println(blue_light);
      val = red_light;
      val2 = green_light;
      val3 = blue_light;
      /*
      uint16_t maxval = max(val, val2);
      maxval = max(maxval, val3);
      float divider = maxval / 255.0;
      uint8_t _r = r / divider;
      uint8_t _g = g / divider;
      uint8_t _b = b / divider;
      Serial.println(_r);
      Serial.println(_g);
      Serial.println(_r);
      */
      if ((blue_light > red_light) && (blue_light > green_light)) {
        green_light = green_light / 2;
        red_light = red_light / 2;
      }
      else
      if ((green_light > red_light) && (green_light > blue_light)) {
        blue_light = blue_light / 2;
        red_light = red_light / 2;
      }
      else   
      if ((red_light > blue_light) && (red_light > green_light)) {
        green_light = green_light / 2;
        blue_light = blue_light / 2;
      }
      if (red_light > 255) {red_light = 255;}
      if (green_light > 255) {green_light = 255;}
      if (blue_light > 255) {blue_light = 255;}
      Serial.print("Ambient: ");
      Serial.print(ambient_light);
      Serial.print(" Red: ");
      Serial.print(red_light);
      Serial.print(" Green: ");
      Serial.print(green_light);
      Serial.print(" Blue: ");
      Serial.println(blue_light);
      FillLEDsFromStaticColor(red_light, green_light, blue_light); 
    }
    
  }

  if (millis() > (lastMsgTime + 1000)) {
    SendDataToMQTT(sensor, type, val, type2, val2, type3, val3);
    lastMsgTime = millis();
  }


}


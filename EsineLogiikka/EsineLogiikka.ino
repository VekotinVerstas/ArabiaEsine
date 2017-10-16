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
#define NUM_LEDS    30
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
  0,   255,  0, 255,  //purple
  80,     0,  0, 255,  //blue
  120,     0, 255,  0,  //green
  150,   255,  05,  0,  //red
  255,   255, 255, 255
}; //full white


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
  delay ( 2500 );
  while (WiFi.status() != WL_CONNECTED) {
    // TODO: blink leds here
    delay ( 250 );
    FillLEDsFromStaticColor(100,100,0); 
    Serial.print ( "." );
    delay ( 250 );
    FillLEDsFromStaticColor(0,0,0); 
    if ((millis() - wifi_start) > 10*1000) {
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
  //pinMode(interruptPin, INPUT_PULLUP);
  //pinMode(interruptPin2, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(interruptPin), handleInterrupt, FALLING);
  //attachInterrupt(digitalPinToInterrupt(interruptPin2), handleInterrupt2, RISING);
  // Initialize interrupt service routine, RGB APDS-9960 gesture function
  pinMode(APDS9960_INT, INPUT_PULLUP);
  attachInterrupt(APDS9960_INT, interruptRoutine, FALLING);

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

  // Check which sensor is connected
  Serial.println("Setup: Polling IR thermometer");
  if (readObjectTempC(0x5A) < 500) {
    currentMode = S_IR_TEMP;
    Serial.print("Setup: Found IR thermometer: ");
    Serial.println(readObjectTempC(0x5A));
  }

  Serial.println("Setup: Polling BH1750 lightmeter");
  lightMeter.begin(BH1750_CONTINUOUS_HIGH_RES_MODE);
  uint16_t lux = lightMeter.readLightLevel();
  if (lux >= 0 && lux < 54612) {
    currentMode = S_LUX_METER;
    Serial.print("Setup: Found LUX meter: ");
    Serial.println(lux);
  }

  Serial.println("Setup: Polling Si xxxxx");
  ht_sensor.begin();
  float humidity = ht_sensor.getRH();
  Serial.println(humidity);
  if (humidity > 0.0 && humidity <= 150.0) {
    currentMode = S_TEMP_HUMIDITY;
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
    MqttSetup();
  }
  if (cnt % 10 == 0) {
    /*
      Serial.print("looping ");
      Serial.println(cnt);
      Serial.print("Ambient = "); Serial.print(readAmbientTempC(0x5A));
      Serial.print("*C\tObject = "); Serial.print(readObjectTempC(0x5A)); Serial.println("*C");
      Serial.println();
    */
    //Serial.println(readObjectTempC(0x5A));

  }
  cnt++;

  if (interruptCounter > 0) {
    interruptCounter--;
    numberOfInterrupts++;
    Serial.print("An interrupt has occurred. Total: ");
    Serial.println(numberOfInterrupts);
  }

  if (interruptCounter2 > 0) {
    interruptCounter2--;
    numberOfInterrupts2++;
    //Serial.print("A PIR interrupt has occurred. Total: ");
    //Serial.println(numberOfInterrupts2);
  }

  // 
  /*
  if( isr_flag == 1 ) {
    detachInterrupt(APDS9960_INT);
    handleGesture();
    isr_flag = 0;
    attachInterrupt(APDS9960_INT, interruptRoutine, FALLING);
  }
  */
  // ChangePalettePeriodically();

  // static uint8_t startIndex = 0;
  // startIndex = startIndex + 1; /* motion speed */

  // FillLEDsFromPaletteColors(startIndex);
  // FillLEDsFromPaletteStaticColor
  ShowCurrentEffect();

  FastLED.show();
  FastLED.delay(1000 / UPDATES_PER_SECOND);
}


void debug(const String& msg) {
#ifdef DEBUG
  Serial.println(msg);
#endif
}


void handleInterrupt() {
  // Serial.println((millis() - lastInterruptTime));
  if ((millis() - lastInterruptTime) < 1000) {
    return;
  }
  lastInterruptTime = millis();
  interruptCounter++;
}


void handleInterrupt2() {
  // Serial.println((millis() - lastInterruptTime));
  if ((millis() - lastInterruptTime2) < 1000) {
    return;
  }
  lastInterruptTime2 = millis();
  interruptCounter2++;
}

void interruptRoutine() {
  isr_flag = 1;
}

void handleGesture() {
    if ( apds.isGestureAvailable() ) {
    switch ( apds.readGesture() ) {
      case DIR_UP:
        Serial.println("UP");
        break;
      case DIR_DOWN:
        Serial.println("DOWN");
        break;
      case DIR_LEFT:
        Serial.println("LEFT");
        break;
      case DIR_RIGHT:
        Serial.println("RIGHT");
        break;
      case DIR_NEAR:
        Serial.println("NEAR");
        break;
      case DIR_FAR:
        Serial.println("FAR");
        break;
      default:
        Serial.println("NONE");
    }
  }
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


void FillLEDsFromPaletteColors( uint8_t colorIndex)
{
  uint8_t brightness = 255;

  for ( int i = 0; i < NUM_LEDS; i++) {
    leds[i] = ColorFromPalette( currentPalette, colorIndex, brightness, currentBlending);
    colorIndex += 3;
  }
}


//void SendDataToMQTT(char sensor[], float val) {
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
    float colorIndex = map(val * 100, 10 * 100, 40 * 100, 0, 255);
    Serial.print(val);
    Serial.print("\t");
    Serial.println(colorIndex);
    uint8_t brightness = 255;
    CRGB green  = CHSV( HUE_GREEN, 255, 255);
    for ( int i = 0; i < NUM_LEDS; i++) {
      leds[i] = ColorFromPalette( currentPalette, colorIndex, brightness, currentBlending);
      //leds[i] = green;
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

    float colorIndex = map(val * 100, 40 * 100, 100 * 100, 0, 255);
    Serial.print(val);
    Serial.print(" %, Humi map ");
    Serial.println(colorIndex);
    CRGB green  = CHSV( HUE_GREEN, 255, 255);

    for ( int i = 0; i < NUM_LEDS; i++) {
      leds[i] = ColorFromPalette( currentPalette, (int)colorIndex, brightness, currentBlending);
      //leds[i] = green;
    }
  } else

  if (currentMode == S_LUX_METER)  {
    sensor = "lux";
    type = "luxi";
    uint8_t brightness = 100;
    val = lightMeter.readLightLevel();
    Serial.print(val);
    type2 = "_";
    val2 = 0;
    type3 = "_";
    val3 = 0;

    if (val > 500) {
      val = 500;
    }
    float colorIndex = map(val, 0, 500, 0, 255);
    //    Serial.println(val);
    Serial.print(" lx, lux map ");
    Serial.println(colorIndex);
    CRGB green  = CHSV( HUE_GREEN, 255, 255);

    for ( int i = 0; i < NUM_LEDS; i++) {
      leds[i] = ColorFromPalette( currentPalette, (int)colorIndex, brightness, currentBlending);
      //leds[i] = green;
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
    }
    
  }

  if (millis() > (lastMsgTime + 1000)) {
    SendDataToMQTT(sensor, type, val, type2, val2, type3, val3);
    lastMsgTime = millis();
  }


}

// There are several different palettes of colors demonstrated here.
//
// FastLED provides several 'preset' palettes: RainbowColors_p, RainbowStripeColors_p,
// OceanColors_p, CloudColors_p, LavaColors_p, ForestColors_p, and PartyColors_p.
//
// Additionally, you can manually define your own color palettes, or you can write
// code that creates color palettes on the fly.  All are shown here.

void ChangePalettePeriodically()
{
  // uint8_t currentMode = (millis() / 1000) % 60;
  // uint8_t currentMode = numberOfInterrupts % 8;
  static uint8_t lastMode = 99;
  if (currentMode != lastMode) {
    lastMode = currentMode;

    if ( currentMode ==  0)  {
      debug("Rainbow");
      //SetupRobotEyePalette();
      currentPalette = RainbowColors_p;
      currentBlending = LINEARBLEND;
    }

    //if( currentMode == 10)  { currentPalette = RainbowStripeColors_p;   currentBlending = NOBLEND;  }
    if ( currentMode == 1)  {
      float temp = readObjectTempC(0x5A);
      debug("Rainbow stripe");
      currentPalette = RainbowStripeColors_p;
      currentBlending = LINEARBLEND;
    }

    if ( currentMode == 2)  {
      debug("purple green");
      SetupPurpleAndGreenPalette();
      currentBlending = LINEARBLEND;
    }

    if ( currentMode == 3)  {
      debug("random");
      SetupTotallyRandomPalette();
      currentBlending = LINEARBLEND;
    }
    //if( currentMode == 30)  { SetupBlackAndWhiteStripedPalette();       currentBlending = NOBLEND; }

    if ( currentMode == 4)  {
      debug("B & W");
      SetupBlackAndWhiteStripedPalette();
      currentBlending = LINEARBLEND;
    }

    if ( currentMode == 5)  {
      debug("cloud");
      currentPalette = CloudColors_p;
      currentBlending = LINEARBLEND;
    }

    if ( currentMode == 6)  {
      debug("party");
      currentPalette = PartyColors_p;
      currentBlending = LINEARBLEND;
    }

    //if( currentMode == 50)  { currentPalette = myRedWhiteBluePalette_p; currentBlending = NOBLEND;  }
    if ( currentMode == 7)  {
      debug("red white blue");
      currentPalette = myRedWhiteBluePalette_p;
      currentBlending = LINEARBLEND;
    }
  }
}




// This function fills the palette with totally random colors.
void SetupTotallyRandomPalette()
{
  for ( int i = 0; i < 16; i++) {
    currentPalette[i] = CHSV( random8(), 255, random8());
  }
}

// This function sets up a palette of black and white stripes,
// using code.  Since the palette is effectively an array of
// sixteen CRGB colors, the various fill_* functions can be used
// to set them up.
void SetupBlackAndWhiteStripedPalette()
{
  // 'black out' all 16 palette entries...
  fill_solid( currentPalette, 16, CRGB::Black);
  // and set every fourth one to white.
  currentPalette[0] = CRGB::White;
  currentPalette[4] = CRGB::White;
  currentPalette[8] = CRGB::White;
  currentPalette[12] = CRGB::White;

}

// This function sets up a palette of purple and green stripes.
void SetupPurpleAndGreenPalette()
{
  CRGB purple = CHSV( HUE_PURPLE, 255, 255);
  CRGB green  = CHSV( HUE_GREEN, 255, 255);
  CRGB black  = CRGB::Black;

  currentPalette = CRGBPalette16(
                     green,  green,  black,  black,
                     purple, purple, black,  black,
                     green,  green,  black,  black,
                     purple, purple, black,  black );
}

// This function sets up a palette of purple and green stripes.
void SetupRobotEyePalette()
{
  CRGB red = CRGB::Red;
  CRGB black  = CRGB::Black;

  currentPalette = CRGBPalette16(
                     red, red, black,  black,
                     red, red, black,  black,
                     red, red, black,  black,
                     red, red, black,  black
                     /*
                                                        red, red, black,  black,
                                                        red, red, red, red,
                                                        red, red, red, red,
                                                        black,  black, black,  black,
                                                        black,  black, black,  black
                     */
                   );
}




// This example shows how to set up a static color palette
// which is stored in PROGMEM (flash), which is almost always more
// plentiful than RAM.  A static PROGMEM palette like this
// takes up 64 bytes of flash.
const TProgmemPalette16 myRedWhiteBluePalette_p PROGMEM =
{
  CRGB::Red,
  CRGB::Gray, // 'white' is too bright compared to red and blue
  CRGB::Blue,
  CRGB::Black,

  CRGB::Red,
  CRGB::Gray,
  CRGB::Blue,
  CRGB::Black,

  CRGB::Red,
  CRGB::Red,
  CRGB::Gray,
  CRGB::Gray,
  CRGB::Blue,
  CRGB::Blue,
  CRGB::Black,
  CRGB::Black
};



// Additionl notes on FastLED compact palettes:
//
// Normally, in computer graphics, the palette (or "color lookup table")
// has 256 entries, each containing a specific 24-bit RGB color.  You can then
// index into the color palette using a simple 8-bit (one byte) value.
// A 256-entry color palette takes up 768 bytes of RAM, which on Arduino
// is quite possibly "too many" bytes.
//
// FastLED does offer traditional 256-element palettes, for setups that
// can afford the 768-byte cost in RAM.
//
// However, FastLED also offers a compact alternative.  FastLED offers
// palettes that store 16 distinct entries, but can be accessed AS IF
// they actually have 256 entries; this is accomplished by interpolating
// between the 16 explicit entries to create fifteen intermediate palette
// entries between each pair.
//
// So for example, if you set the first two explicit entries of a compact
// palette to Green (0,255,0) and Blue (0,0,255), and then retrieved
// the first sixteen entries from the virtual palette (of 256), you'd get
// Green, followed by a smooth gradient from green-to-blue, and then Blue.

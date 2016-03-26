/// RC-ESC RADIO
/////////////////////////////////////////////////////////////////////////////////////
///           Electronic Stability Control for RC Car v1.0 2016/03/26             ///
/////////////////////////////////////////////////////////////////////////////////////
/// IMPORTANT NOTICE: THIS PROJECT IS FAR FROM READY, SO USE IT AT YOUR OWN RISK!
/// Description coming soon...

#include <nRF24L01.h>
#include <printf.h>
#include <RF24.h>
#include <RF24_config.h>
#include <SPI.h>

#include <I2Cdev.h>
#include <EasyTransferI2C.h>
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include <Wire.h>
#endif

#include <EnableInterrupt.h>

/////////////////////////////////////////
///   PIN ASSIGNMENTS !! NEED SETTING !!
////////////////////////////////////////
///   ANALOG
#define BATTERY_PIN A5
#define ENGINE_TEMP_PIN 0
#define SAFETY_RELAY_PIN 4
///   DIGITAL
#define ESC_RESET_PIN 6
#define GLOW_RELAY_PIN A1
#define GLOW_LED_PIN A2
#define SAFETY_RELAY_LED_PIN 3
#define MANUAL_GLOW_PIN 2
/// endof PIN ASSIGNMENTS

#define MODE_BOOT 0
#define MODE_STARTUP 1
#define MODE_RUN 2

#define ESC_MODE_FORCEPROGRAM 0
#define ESC_MODE_RUN 1
#define ESC_MODE_QUICK_PROGRAM 2
#define ESC_MODE_FULL_PROGRAM 3
#define ESC_MODE_ERROR 4

#define BATTERY_OK 0
#define BATTERY_LOW 1
#define BATTERY_CRITICAL 2

//// RADIO SETTINGS
bool radioNumber = 0;
RF24 radio(7,8);
byte addresses[][6] = {"1Node","2Node"};
bool role = 0;
/// endof radio settings

float batteryV;
float engineTemp;
bool reset;
volatile uint32_t lastWireInMillis = 0;
uint32_t lastWireOutMillis = 0;
uint32_t startTime = 0;
uint16_t engineRPM = 0;
bool glowActive = false;
bool ESCfail = false;
byte gMode = ESC_MODE_RUN;

byte progMode = MODE_BOOT;
byte batteryState = BATTERY_OK;

//////////////////////////////////////////////
/// DUMMY ensitivity settings for testing
//////////////////////////////////////////////
/// Input-related sensitivity:
// the amount of jitter allowed while assuming centered controller
uint8_t steeringDeadzone = 15;
// the sideways acceleration allowed when attempting to go straight
float straightSensitivity = 0.1;
// the avg wheel angle under which we assume straight motion
// this is to avoid passing a zero to the tangent function ;)
uint8_t ESC_deadzone = 0.0174532925; /* 2-3 degrees */
int brake_deadzone = 150;
/// Decision related sensitivity:
float oversteer_yaw_difference = 1;
float understeerXdiff = 0.2;
float understeerYdiff = 0.2;
/// Intervention strength (multi = multiplying; corr = adding/subtracting):
// 1 = wheels face the pre-skid direction, 
float oversteer_steering_multi = 1; 
int oversteer_throttle_corr = -150;
int understeer_throttle_corr = -25;
int straightline_corr = 30;
/// Geometry related settings:
uint8_t wheelCircumference = 200; 

EasyTransferI2C easyTransferIn; /// for receiving telemetry data from other arduino
EasyTransferI2C easyTransferOut; /// for setting values of ESC

struct SEND_SETTINGS_STRUCTURE{ /// for setting values of ESC
  // variables have the same meaning as in ESC
  uint8_t steeringDeadzone;
  uint8_t straightSensitivity;
  uint8_t ESC_deadzone; 
  int brake_deadzone;
  float oversteer_yaw_difference;
  float understeerXdiff;
  float understeerYdiff;
  float oversteer_steering_multi; 
  int oversteer_throttle_corr;
  int understeer_throttle_corr;
  int straightline_corr;
  uint8_t wheelCircumference;
  uint8_t gMode;
  uint8_t engine_throttle_corr;
};

struct RECEIVE_DATA_STRUCTURE{ /// for receiving telemetry data from other arduino
  uint8_t state;
  uint8_t gMode;
  float dTrainSpeed;
};

struct RADIO_DATA {
  uint8_t state;
  uint8_t gMode;
  float dTrainSpeed;
};

RECEIVE_DATA_STRUCTURE rxdata;
SEND_SETTINGS_STRUCTURE txdata;

RADIO_DATA radioData;

void setup() 
{  
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  Wire.begin(0x9);
  TWBR = 24;

  radio.begin();
  radio.setPALevel(RF24_PA_LOW);
  // Open a writing and reading pipe on each radio, with opposite addresses
  if(radioNumber){
    radio.openWritingPipe(addresses[1]);
    radio.openReadingPipe(1,addresses[0]);
  }else{
    radio.openWritingPipe(addresses[0]);
    radio.openReadingPipe(1,addresses[1]);
  }
  
  // Start the radio listening for data
  radio.startListening();
  
  pinMode(SAFETY_RELAY_PIN,OUTPUT);
  pinMode(GLOW_LED_PIN,OUTPUT);
  pinMode(GLOW_RELAY_PIN,OUTPUT);
  pinMode(ESC_RESET_PIN,OUTPUT);
  pinMode(SAFETY_RELAY_LED_PIN,OUTPUT);
  pinMode(MANUAL_GLOW_PIN,INPUT);
  
  digitalWrite(SAFETY_RELAY_PIN, LOW);
  digitalWrite(ESC_RESET_PIN, HIGH);
  digitalWrite(GLOW_LED_PIN, LOW);
  digitalWrite(SAFETY_RELAY_LED_PIN, LOW);
  
  easyTransferIn.begin(details(rxdata), &Wire);
  easyTransferOut.begin(details(txdata), &Wire);  

  txdata.steeringDeadzone = steeringDeadzone;
  txdata.straightSensitivity = straightSensitivity;
  txdata.ESC_deadzone = ESC_deadzone; 
  txdata.brake_deadzone = brake_deadzone;
  txdata.oversteer_yaw_difference = oversteer_yaw_difference;
  txdata.understeerXdiff = understeerXdiff;
  txdata.understeerYdiff = understeerYdiff;
  txdata.oversteer_steering_multi = oversteer_steering_multi; 
  txdata.oversteer_throttle_corr = oversteer_throttle_corr;
  txdata.understeer_throttle_corr = understeer_throttle_corr;
  txdata.straightline_corr = straightline_corr;
  txdata.wheelCircumference = wheelCircumference;
  txdata.gMode = gMode;
  txdata.engine_throttle_corr = 0;

  progMode = MODE_RUN;
}

void loop() 
{  
  if (progMode == MODE_BOOT)
  {
    startTime = millis();
    while (!easyTransferIn.receiveData())
    {
      if (millis() > (startTime + 5000)) 
      { 
        break; 
      }
    }
  }
  else if (progMode == MODE_STARTUP)
  {    
    if (engineRPM > 0) /// only glow when engine is rotating
    {
      digitalWrite(GLOW_RELAY_PIN,HIGH);
      digitalWrite(GLOW_LED_PIN,HIGH);
      glowActive = true;
    }
    else
    {
      digitalWrite(GLOW_RELAY_PIN,LOW);
      digitalWrite(GLOW_LED_PIN,LOW);
      glowActive = false;
    } 
    /*
    if (engineRPM > startPadRPM) /// refine conditions for a few seconds run
    {
      progMode = MODE_RUN; 
    }
    */
  }
  else if (progMode == MODE_RUN)
  {
    uint32_t commonMillis = millis();
    
    engineTemp = analogRead(ENGINE_TEMP_PIN);
    engineTemp = (engineTemp / 1023) * 500 - 273; /// LM335 gives temp in Kelvin  
        
    if (easyTransferIn.receiveData())
    {
      /// TODO: store and send Data on RF
      digitalWrite(SAFETY_RELAY_PIN, HIGH);
      digitalWrite(SAFETY_RELAY_LED_PIN,HIGH);
      lastWireInMillis = commonMillis;
      ESCfail = false;
      reset = false;

      radioData.state = rxdata.state;
      radioData.dTrainSpeed = rxdata.dTrainSpeed;
      radio.stopListening();    
      radio.write( &radioData, sizeof(radioData));          
      radio.startListening();

      gMode = rxdata.gMode;
    }
    else if (!ESCfail && (commonMillis > lastWireInMillis + 500)) /// ESC not working
    {
      digitalWrite(SAFETY_RELAY_PIN, LOW);
      digitalWrite(SAFETY_RELAY_LED_PIN,LOW);
      ESCfail = true;
    }    
    
    if (ESCfail && (commonMillis > lastWireInMillis + 1000))
    { /// pull reset pin high after 300ms so ESC can reset
      digitalWrite(ESC_RESET_PIN, HIGH);  
      ESCfail = false;  
      if (!reset)
      {
        digitalWrite(ESC_RESET_PIN, LOW);
        delay(150);
        reset = true;
      }
      else
      {
        digitalWrite(ESC_RESET_PIN, HIGH);
      }
    }
    txdata.engine_throttle_corr = 0;

    if (digitalRead(MANUAL_GLOW_PIN) == HIGH)
    {
      digitalWrite(GLOW_RELAY_PIN,HIGH);
      digitalWrite(GLOW_LED_PIN,HIGH);
      glowActive = true;
    }
    else
    {
      digitalWrite(GLOW_RELAY_PIN,LOW);
      digitalWrite(GLOW_LED_PIN,LOW);
      glowActive = false;
    } 
    /*
    if (commonMillis > (lastWireOutMillis + 200))
    {
      if (false)
      {
        Wire.beginTransmission(0x8);
        easyTransferOut.sendData(0x8);
        Wire.endTransmission(true);
        lastWireOutMillis = commonMillis;
        digitalWrite(2, HIGH);
      }           
      else
      {
        digitalWrite(2, LOW);
      }
    }
    */
   
    
    
  /*
    
    if ((int) engineTemp > 120)
    {
      /// decreaseRPM
      /// if throttle <= idle give little pulses of throttle, to roll the car
    }
    else if ((int) engineTemp > 90)
    {
      /// glowOff & noChangeRPM
      digitalWrite(GLOW_RELAY_PIN,LOW);
      digitalWrite(GLOW_LED_PIN,LOW);
      glowActive = false;
    }
    else if ((int) engineTemp < 80) /// or RPM is falling?
    {
      /// glowOn & increaseRPM
      digitalWrite(GLOW_RELAY_PIN,HIGH);
      digitalWrite(GLOW_LED_PIN,HIGH);
      glowActive = true;
    } */
   
  }
  /* Battery measurement unrelated to modes, but should not happen when glowing,
     because the current draw of the glow plug might drop voltage */
/*  if (!glowActive)
  {
    batteryV = analogRead(BATTERY_PIN);
    batteryV = batteryV / 1023 * 10;
    if (batteryV < 5.5) /// arduinos switch off at this point... :(
    {
      batteryState = BATTERY_CRITICAL;
      /// limit throttle and alert user
    }
    else if (batteryV < 5.8) 
    {
      batteryState = BATTERY_LOW;
      // digitalWrite(GLOW_LED_PIN, HIGH);
    } 
    else
    {
      batteryState = BATTERY_OK; 
      // digitalWrite(GLOW_LED_PIN, LOW);
    }
  }*/
}

void receiveEvent(int howMany)
{
  
}

void requestEvent()
{
  
}


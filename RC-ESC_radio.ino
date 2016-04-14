/// RC-ESC RADIO
/////////////////////////////////////////////////////////////////////////////////////
///           Electronic Stability Control for RC Car v1.0 2016/04/14             ///
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
#define BATTERY_PIN 5
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
#define MODE_PROGRAM 3

#define ESC_MODE_FORCEPROGRAM 0
#define ESC_MODE_RUN 1
#define ESC_MODE_QUICK_PROGRAM 2
#define ESC_MODE_FULL_PROGRAM 3
#define ESC_MODE_ERROR 4
#define ESC_MODE_REMOTE_PROGRAM 5

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
uint32_t lastRadioOut = 0;
uint32_t lastBatteryMillis = 0;
byte progMode = MODE_BOOT;
byte batteryState = BATTERY_OK;

bool ESCswitch = true;

bool programming = false;
bool programSent = false;
bool programRequested = false;
bool statusRequested = false;

//////////////////////////////////////////////
/// Sensitivity settings
//////////////////////////////////////////////
/// Input-related sensitivity:
// the amount of jitter allowed while assuming centered controller
uint8_t steeringDeadzone = 15;
// the sideways acceleration allowed when attempting to go straight
float straightSensitivity = 0.1;
// the avg wheel angle under which we assume straight motion
// this is to avoid passing a zero to the tangent function ;)
float ESC_deadzone = 0.0174532925; /* 2-3 degrees */
int brake_deadzone = 100;
/// Decision related sensitivity:
float oversteer_yaw_difference = 2;
float oversteer_yaw_release = 2;
float understeerXdiff = 0.4;
float understeerYdiff = 0.4;
/// Intervention strength (multi = multiplying; corr = adding/subtracting):
// 1 = wheels face the pre-skid direction, 
float oversteer_steering_multi = 1; 
int oversteer_throttle_corr = -150;
int understeer_throttle_corr = -25;
int straightline_corr = 30;
/// Geometry related settings:
uint8_t wheelCircumference = 200; // in mm, if wheel diameter > 80 change to int
/// Sensitivity settings' limit will be implemented by the other arduino, if any
/// End of sensitivity settings ////

EasyTransferI2C easyTransferIn; /// for receiving telemetry data from other arduino
EasyTransferI2C easyTransferOut; /// for setting values of ESC

struct SEND_SETTINGS_STRUCTURE{ /// for setting values of ESC
  // variables have the same meaning as in ESC
  uint8_t steeringDeadzone;
  float straightSensitivity;
  float ESC_deadzone; 
  int brake_deadzone;
  float oversteer_yaw_difference;
  float oversteer_yaw_release;
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

struct RADIO_DATA_OUT {
  uint8_t state;
  uint8_t gMode;
  float dTrainSpeed;
  float batteryV;
  float engineTemp;
  bool glowActive;
};

struct RADIO_DATA_IN {
  bool ESCswitch;
};

RECEIVE_DATA_STRUCTURE rxdata;
SEND_SETTINGS_STRUCTURE txdata;

RADIO_DATA_OUT radioDataOut;
RADIO_DATA_IN radioDataIn;

void setup() 
{    
  Serial.begin(9600);
  Serial.println("Start");
  
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestStatus);
  Wire.begin(0xA3);
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
  txdata.oversteer_yaw_release = oversteer_yaw_release;
  txdata.understeerXdiff = understeerXdiff;
  txdata.understeerYdiff = understeerYdiff;
  txdata.oversteer_steering_multi = oversteer_steering_multi; 
  txdata.oversteer_throttle_corr = oversteer_throttle_corr;
  txdata.understeer_throttle_corr = understeer_throttle_corr;
  txdata.straightline_corr = straightline_corr;
  txdata.wheelCircumference = wheelCircumference;
  txdata.gMode = gMode;
  txdata.engine_throttle_corr = 100;

  ESCswitch = true;

  progMode = MODE_RUN;

}

void loop() 
{  
  uint32_t commonMillis;
  if (progMode == MODE_BOOT)
  {
    startTime = commonMillis;
    while (!easyTransferIn.receiveData() && !ESCfail)
    {
      if (commonMillis > (startTime + 5000)) 
      { 
        ESCfail = true;
        break; 
      }
    }
    if (ESCfail)
    {
      digitalWrite(ESC_RESET_PIN,LOW);
      delay(200);
      digitalWrite(ESC_RESET_PIN,HIGH);
      ESCfail = false;
    }
    else
    {
      progMode == MODE_STARTUP;
      programming = 1;
    }
  }
  else if (progMode == MODE_STARTUP)
  {    
    progMode == MODE_RUN; /// TODO: remove this & next line when startup mode is ready
    return;
    
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
    commonMillis = millis();

    analogRead(ENGINE_TEMP_PIN);
    engineTemp = analogRead(ENGINE_TEMP_PIN);
    engineTemp = (engineTemp / 1023) * 500 - 273; /// LM335 gives temp in Kelvin  
    radioDataOut.engineTemp = engineTemp;
        
    if (easyTransferIn.receiveData())
    {
      /// TODO: store and send Data on RF
      if (ESCswitch)
      {        
        digitalWrite(SAFETY_RELAY_PIN, HIGH);
        digitalWrite(SAFETY_RELAY_LED_PIN,HIGH);
      }
      else
      {
        digitalWrite(SAFETY_RELAY_PIN, LOW);
        digitalWrite(SAFETY_RELAY_LED_PIN,LOW);
      }
      lastWireInMillis = commonMillis;
      ESCfail = false;
      reset = false;

      radioDataOut.state = rxdata.state;
      radioDataOut.dTrainSpeed = rxdata.dTrainSpeed;      

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

    /// TODO: delete or comment out, if manual glow is not needed anymore
/*    if (digitalRead(MANUAL_GLOW_PIN) == HIGH)
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
    } */
    
    radioDataOut.glowActive = glowActive;
    /// Do these every 50 ms
    if (commonMillis - lastRadioOut > 50)
    {
      radio.stopListening();    
      radio.write( &radioDataOut, sizeof(radioDataOut));          
      radio.startListening();
      lastRadioOut = commonMillis;
    }

    if(radio.available())
    {                                                             
      while (radio.available()) 
      {                                  
        radio.read( &radioDataIn, sizeof(radioDataIn));
      }  
      ESCswitch = radioDataIn.ESCswitch;
    }
    
    if (commonMillis > (lastWireOutMillis + 15000))
    {
      txdata.steeringDeadzone = steeringDeadzone;
      txdata.straightSensitivity = straightSensitivity;
      txdata.ESC_deadzone = ESC_deadzone; 
      txdata.brake_deadzone = brake_deadzone;
      txdata.oversteer_yaw_difference = oversteer_yaw_difference;
      txdata.oversteer_yaw_release = oversteer_yaw_release;
      txdata.understeerXdiff = understeerXdiff;
      txdata.understeerYdiff = understeerYdiff;
      txdata.oversteer_steering_multi = oversteer_steering_multi; 
      txdata.oversteer_throttle_corr = oversteer_throttle_corr;
      txdata.understeer_throttle_corr = understeer_throttle_corr;
      txdata.straightline_corr = straightline_corr;
      txdata.wheelCircumference = wheelCircumference;
      txdata.gMode = gMode;
      txdata.engine_throttle_corr = 200;
      programming = true;
      lastWireOutMillis = commonMillis;
    }
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
  else if (progMode == MODE_PROGRAM)
  {  
    if (programSent && statusRequested)
    {   
      statusRequested = false;
      Serial.println("Prog sent, listening"); 
      if (easyTransferIn.receiveData())
      {
        gMode = rxdata.gMode;
        Serial.print("Mode is: ");
        Serial.println(gMode);
        lastWireInMillis = commonMillis;
        if (gMode == ESC_MODE_RUN)
        {
          Serial.println("Ready");
          progMode = MODE_RUN;
          programSent = false;
          programming = false;
        }
      }      
    }    
  }
  /* Battery measurement unrelated to modes, but should not happen when glowing,
     because the current draw of the glow plug might drop voltage */
  if (!glowActive && commonMillis - lastBatteryMillis > 500)
  {
    analogRead(BATTERY_PIN);
    batteryV = analogRead(BATTERY_PIN);
    batteryV = batteryV / 1023 * 10;    
    radioDataOut.batteryV = batteryV;
    
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
    lastBatteryMillis = commonMillis;
  }
  if (programming)
  {
    progMode = MODE_PROGRAM;
  }
}

void receiveEvent(int howMany)
{
  
}

void requestStatus()
{  
  if (programming)
  {    
    digitalWrite(SAFETY_RELAY_PIN, LOW);
    digitalWrite(SAFETY_RELAY_LED_PIN,LOW);
    Wire.write(1);  
    Wire.onRequest(requestProgram); 
  }
  else
  {
    Wire.write(0);
  }
  statusRequested = true;
}
void requestProgram()
{  
  programRequested = true; 
  easyTransferOut.sendDataPerRequest(0xA2);
  programSent = true;
  Wire.onRequest(requestStatus); 
}

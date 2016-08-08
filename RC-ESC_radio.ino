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
#include <EasyTransfer.h>
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
#define BATTERY_PIN 6
#define ENGINE_TEMP_PIN 0
#define SAFETY_RELAY_PIN 4
///   DIGITAL
#define ESC_RESET_PIN 6
#define GLOW_RELAY_PIN A1
#define GLOW_LED_PIN A2
#define SAFETY_RELAY_LED_PIN 3
#define MANUAL_GLOW_PIN 2
#define NEON_PIN 10
#define LIGHT_PIN 9
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
uint32_t glowStart = 0;
uint32_t glowWaitStart = 0;
uint16_t engineRPM = 0;
bool glowActive = false;
bool manualGlow = false;
bool glowWait = false;
bool ESCfail = false;
byte gMode = ESC_MODE_RUN;
uint32_t lastRadioOut = 0;
uint32_t lastBatteryMillis = 0;
byte progMode = MODE_BOOT;
byte batteryState = BATTERY_OK;

bool ESCswitch = true;
bool remoteProgram = false;

bool programming = false;
bool programSent = false;
bool programRequested = false;
bool statusRequested = false;

/* Bools in switches0
 *  0 ESCswitch
 *  1 programming
 *  2 glowActive
 *  3 safetyRelayState
 *  4 headlight
 *  5 neon
 *  6 brakelight?
 *  7 empty  */
byte switches0 = 0;
/* Bools in switches1
 *  0 Oversteer detection
 *  1 Oversteer correction
 *  2 Understeer detection
 *  3 Understeer correction
 *  4 Veering detection
 *  5 Veering correction
 *  6 ABS
 *  7 LED  */
byte switches1 = B11111111;

//////////////////////////////////////////////
/// Sensitivity settings
//////////////////////////////////////////////
/// Input-related sensitivity:
// the amount of jitter allowed while assuming centered controller
int steeringDeadzone = 15;
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
int wheelCircumference = 200; // in mm, if wheel diameter > 80 change to int
/// Sensitivity settings' limit will be implemented by the other arduino, if any
/// End of sensitivity settings ////

EasyTransferI2C easyTransferIn; /// for receiving telemetry data from other arduino
EasyTransferI2C easyTransferOut; /// for setting values of ESC

struct SEND_SETTINGS_STRUCTURE{ /// for setting values of ESC
  // variables have the same meaning as in ESC
  byte switches0;
  byte switches1;
  int steeringDeadzone;
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
  int wheelCircumference;
  uint8_t gMode;
  uint8_t engine_throttle_corr;
};

struct RECEIVE_DATA_STRUCTURE{ /// for receiving telemetry data from other arduino
  uint8_t state;
  uint8_t gMode;
  bool glow;
  float dTrainSpeed;
};

struct RADIO_DATA_OUT {
  uint8_t state;
  uint8_t gMode;
  float dTrainSpeed;
  float batteryV;
  float engineTemp;
  bool glowActive;
  bool safetyRelayState;
  uint8_t progMode;
  uint8_t radioStatus;
};

struct RADIO_DATA_IN_0 {
  byte switches0;
  byte switches1;
  /// settings:
  int steeringDeadzone;
  float straightSensitivity;
  float ESC_deadzone; 
  int brake_deadzone;
  float oversteer_yaw_difference;
  float oversteer_yaw_release;
  uint8_t code;
};

struct RADIO_DATA_IN_1 {
  float understeerXdiff;
  float understeerYdiff;
  float oversteer_steering_multi; 
  int oversteer_throttle_corr;
  int understeer_throttle_corr;
  int straightline_corr;
  int wheelCircumference;
  uint8_t gMode;
  uint8_t engine_throttle_corr;
  uint8_t code;
};

uint8_t engine_throttle_corr = 0;

RECEIVE_DATA_STRUCTURE rxdata;
SEND_SETTINGS_STRUCTURE txdata;

RADIO_DATA_OUT radioDataOut;
RADIO_DATA_IN_0 radioDataIn0;
RADIO_DATA_IN_1 radioDataIn1;

uint8_t radioStatus;

#define R_STAT_READY 0
#define R_STAT_ACK_0 1
#define R_STAT_ACK_1 2

#define R_CODE_0 122
#define R_CODE_1 195

#define GLOW_TIME 3000
#define GLOW_WAIT 2000

void setup() 
{    
  // Serial.begin(115200);
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
  pinMode(NEON_PIN,OUTPUT);
  pinMode(LIGHT_PIN,OUTPUT);
  
  digitalWrite(SAFETY_RELAY_PIN, LOW);
  digitalWrite(ESC_RESET_PIN, HIGH);
  digitalWrite(GLOW_LED_PIN, LOW);
  digitalWrite(SAFETY_RELAY_LED_PIN, LOW);
 
  easyTransferIn.begin(details(rxdata), &Wire);
  easyTransferOut.begin(details(txdata), &Wire);  
  
  txdata.switches0 = switches0;
  txdata.switches1 = switches1;
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
  txdata.engine_throttle_corr = engine_throttle_corr;

  ESCswitch = true;
  radioStatus = R_STAT_READY;

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
      if (rxdata.glow) 
      {                 
        if (!manualGlow) { rxdata.glow = false; }
        manualGlow = true; 
        glowWait = true; 
        glowWaitStart = commonMillis;
      }

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

    
    if ((commonMillis - glowStart) > GLOW_TIME)
    {
      digitalWrite(GLOW_RELAY_PIN,LOW);
      digitalWrite(GLOW_LED_PIN,LOW);
      glowActive = false;
    }
    if (manualGlow)
    {
      if (glowActive || rxdata.glow)
      {
        digitalWrite(GLOW_RELAY_PIN,LOW);
        digitalWrite(GLOW_LED_PIN,LOW);
        glowActive = false;
        manualGlow = false;
        glowWait = false;
      }      
    }
    if (manualGlow)
    {      
      if (glowWait)
      {
        if ((commonMillis - glowWaitStart) > GLOW_WAIT)
        {
          glowWait = false;
        }
        else
        {
          // A good way to async flash LEDs :)
          bool state = (commonMillis - glowWaitStart) / 100 % 2;
          digitalWrite(GLOW_LED_PIN,state);
        }
      }
      else
      {
        digitalWrite(GLOW_RELAY_PIN,HIGH);
        digitalWrite(GLOW_LED_PIN,HIGH);
        glowStart = commonMillis;
        glowActive = true;      
        manualGlow = false;
      }
      
    }
    /// TODO: delete or comment out, if manual glow is not needed anymore
    /*if (digitalRead(MANUAL_GLOW_PIN) == LOW)
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
    }*/
    
    
    
    
    if (remoteProgram && radioStatus == R_STAT_ACK_1)
    {
      txdata.switches0 = switches0;
      txdata.switches1 = switches1;
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
      txdata.engine_throttle_corr = engine_throttle_corr;
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
  /*
  if (digitalRead(NEON_PIN) != !readBool(&switches0, 4)) // lightstate != switchstate
  {
    switchLight(NEON_PIN, readBool(&switches0, 5));
  }
  if (digitalRead(LIGHT_PIN) != !readBool(&switches0, 4)) // lightstate != switchstate
  {
    switchLight(LIGHT_PIN, readBool(&switches0, 4));
  }
  */
  
  /* Battery measurement unrelated to modes, but should not happen when glowing,
     because the current draw of the glow plug might drop voltage */
  if (!glowActive && commonMillis - lastBatteryMillis > 500)
  {    
    analogRead(BATTERY_PIN);
    batteryV = analogRead(BATTERY_PIN);
    float vcc = (float) readVcc() / 1000;
    batteryV = batteryV / 1023 * vcc * 2;
    
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
  radioDataOut.progMode = progMode;
  /// Do these every 50 ms
  if (commonMillis - lastRadioOut > 50)
  {
    /// Set variables:
    radioDataOut.glowActive = glowActive;
    radioDataOut.safetyRelayState = digitalRead(SAFETY_RELAY_PIN);
    radioDataOut.radioStatus = radioStatus;
    radio.stopListening();    
    radio.write( &radioDataOut, sizeof(radioDataOut));          
    radio.startListening();
    if (radioStatus == R_STAT_ACK_1)
    {
      radioStatus = R_STAT_READY;
    }
    lastRadioOut = commonMillis;
  }
  
  if(radio.available())
  {        
    if (radioStatus == R_STAT_READY)
    {
      Serial.println("PACKET0?"); 
      while (radio.available()) 
      {                                  
        radio.read( &radioDataIn0, sizeof(radioDataIn0));
      }            
      Serial.println(radioDataIn0.code);                                 
      if (radioDataIn0.code == R_CODE_0)
      {
        Serial.println("PACKET0!"); 
        switches0 = radioDataIn0.switches0;
        switches1 = radioDataIn0.switches1; 
        ESCswitch = readBool(&switches0, 0);      
        remoteProgram = readBool(&switches0, 1);
        /// Set lights, make sophisticated later :D
        digitalWrite(LIGHT_PIN,!readBool(&switches0, 4));
        digitalWrite(NEON_PIN,!readBool(&switches0, 5));
        if (remoteProgram)
        {
          steeringDeadzone = radioDataIn0.steeringDeadzone; 
          straightSensitivity = radioDataIn0.straightSensitivity;
          ESC_deadzone = radioDataIn0.ESC_deadzone;  
          brake_deadzone = radioDataIn0.brake_deadzone; 
          oversteer_yaw_difference = radioDataIn0.oversteer_yaw_difference;
          oversteer_yaw_release = radioDataIn0.oversteer_yaw_release;
          radioStatus = R_STAT_ACK_0;
          Serial.println("PACKET0 ACK");          
        }        
      }
    }
    else if (radioStatus == R_STAT_ACK_0)
    {
      Serial.println("PACKET1?"); 
      while (radio.available()) 
      {                                  
        radio.read( &radioDataIn1, sizeof(radioDataIn1));
      }        
      Serial.println(radioDataIn1.code);                                          
      if (radioDataIn1.code == R_CODE_1)
      {
        Serial.println("PACKET1!"); 
        if (remoteProgram)
        {
          understeerXdiff = radioDataIn1.understeerXdiff;
          understeerYdiff = radioDataIn1.understeerYdiff;
          oversteer_steering_multi = radioDataIn1.oversteer_steering_multi; 
          oversteer_throttle_corr = radioDataIn1.oversteer_throttle_corr;
          understeer_throttle_corr = radioDataIn1.understeer_throttle_corr;
          straightline_corr = radioDataIn1.straightline_corr;
          wheelCircumference = radioDataIn1.wheelCircumference;
          // gMode = radioDataIn1.gMode;
          engine_throttle_corr = radioDataIn1.engine_throttle_corr;
          radioStatus = R_STAT_ACK_1;
          Serial.println("PACKET1 ACK");
          if (false)
          {
            Serial.println(steeringDeadzone); 
            Serial.println(straightSensitivity); 
            Serial.println(ESC_deadzone); 
            Serial.println(brake_deadzone);
            Serial.println(oversteer_yaw_difference); 
            Serial.println(oversteer_yaw_release); 
            Serial.println(understeerXdiff); 
            Serial.println(understeerYdiff); 
            Serial.println(oversteer_steering_multi); 
            Serial.println(oversteer_throttle_corr); 
            Serial.println(understeer_throttle_corr);
            Serial.println(straightline_corr);
            Serial.println(wheelCircumference);
            Serial.println(engine_throttle_corr);
          }
        }
      }
    }    
  }
}

void switchLight(int pin, bool state)
{
  digitalWrite(pin, !state);
  byte i = 255;
  if (state)
  {
    i = 255;
    while (i >= 0)
    {
      analogWrite(pin,i);
      delay(4);
      i--;
    }    
  }
  else
  {
    i = 0;
    while (i < 255)
    {
      analogWrite(pin,i);
      delay(4);
      i++;
    }  
  }
  digitalWrite(pin, state);
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

void flashLights(int pin, byte flashes, int time1, int time2, bool startState){
  bool state = digitalRead(pin);
  for (byte i = 0; i < flashes; i++)
  {
    digitalWrite(pin, startState);
    delay(time1);
    digitalWrite(pin, !startState);
    delay(time2);
  }
  digitalWrite(pin, state);
}

void flashLights(int pin, byte flashes, int time1, int time2){
  flashLights(pin, flashes, time1, time2, digitalRead(pin));
}

bool readBool(byte * data, byte index) 
{
  // Bits from left to right numbered from 0 to 7
  byte mask = B00000001 << (7 - index);
  return mask & *data;
}

byte writeBool(byte * data, byte index, bool input) 
{
  byte mask = B00000001 << (7 - index);
  if (input)
  {    
    *data = mask | *data;
  }
  else
  { 
    // This makes a mask where the bit to set is 0, rest is 1
    mask = B11111111 ^ mask;
    *data = mask & *data;
  }
  return *data;
}

byte flipBool(byte * data, byte index)
{
   byte mask = B00000001 << (7 - index);
   *data = mask ^ *data;
   return *data;
}

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

  result = 1084380L / result; // Calculate Vcc (in mV); Vref = 1,06V calibrated
  // 1084380L = 1.06*1023*1000
  return result; // Vcc in millivolts
}


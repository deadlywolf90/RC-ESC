/// RC-ESC REMOTE
/////////////////////////////////////////////////////////////////////////////////////
///           Electronic Stability Control for RC Car v1.0 2016/03/26             ///
/////////////////////////////////////////////////////////////////////////////////////
/// IMPORTANT NOTICE: THIS PROJECT IS FAR FROM READY, SO USE IT AT YOUR OWN RISK!
/// Description coming soon...

#include <nRF24L01.h>
#include <printf.h>
#include <RF24.h>
#include <RF24_config.h>

#include <Print.h>

#include <stdlib.h>

#include <SPI.h>
#include <RF24.h>

#include <Fat16.h>
#include <Fat16util.h>

#include <EnableInterrupt.h>
// #include <LiquidCrystal.h>

#include <LiquidCrystal_I2C.h>

#define MODE_HOMESCREEN 0
#define MODE_MENU 1
#define MODE_NUMBERCHANGE 2
#define MODE_FLOATCHANGE 3

#define P_MODE_BOOT 0
#define P_MODE_STARTUP 1
#define P_MODE_RUN 2
#define P_MODE_PROGRAM 3

#define ESC_MODE_FORCEPROGRAM 0
#define ESC_MODE_RUN 1
#define ESC_MODE_QUICK_PROGRAM 2
#define ESC_MODE_FULL_PROGRAM 3
#define ESC_MODE_ERROR 4
#define ESC_MODE_REMOTE_PROGRAM 5

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

const int ST_BALANCED = 0;
const int ST_UNDERSTEER = 1;
const int ST_OVERSTEER = 2;
const int ST_SLIP = 3;
const int ST_SPINNING = 4;
const int ST_ROLLED = 5;

int pulses = 0;
int A_SIG=0, B_SIG=1;
uint32_t lastLCDMillis;

/// LiquidCrystal lcd(2, 3, 4, 5, 6, 7);
LiquidCrystal_I2C lcd(0x27,20,4);

//// RADIO SETTINGS
bool radioNumber = 1;

RF24 radio(8,9);
byte addresses[][6] = {"1Node","2Node"};
bool receiving = false;

uint8_t state;
uint8_t gMode;
float dTrainSpeed;
float batteryV;
float engineTemp;
bool glowActive;
bool safetyRelayState;

bool ESCswitch = true;
bool programming = false;

uint8_t radioStatus;

#define R_STAT_READY 0
#define R_STAT_ACK_0 1
#define R_STAT_ACK_1 2

#define R_CODE_0 122
#define R_CODE_1 195

struct RADIO_DATA_IN {
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

struct RADIO_DATA_OUT_0 {
  bool ESCswitch;
  bool programming;
  /// settings:
  uint8_t steeringDeadzone;
  float straightSensitivity;
  float ESC_deadzone; 
  int brake_deadzone;
  float oversteer_yaw_difference;
  float oversteer_yaw_release;
  uint8_t code;
};

struct RADIO_DATA_OUT_1 {
  float understeerXdiff;
  float understeerYdiff;
  float oversteer_steering_multi; 
  int oversteer_throttle_corr;
  int understeer_throttle_corr;
  int straightline_corr;
  uint8_t wheelCircumference;
  uint8_t gMode;
  uint8_t engine_throttle_corr;
  uint8_t code;
};

RADIO_DATA_IN radioDataIn;
RADIO_DATA_OUT_0 radioDataOut0;
RADIO_DATA_OUT_1 radioDataOut1;

/// endof radio settings

// uparrow
byte newChar2[8] = {
  B00010,
  B00110,
  B01110,
  B11110,
  B01110,
  B00110,
  B00010,
  B00000
};
// degree symbol
byte newChar1[8] = {
  B01100,
  B10010,
  B10010,
  B01100,
  B00000,
  B00000,
  B00000,
  B00000
};
// triangle
byte newChar3[8] = {
  B00100,
  B00100,
  B01010,
  B01010,
  B10001,
  B10001,
  B11111,
  B00000
};
// fulltriangle
byte newChar4[8] = {
  B00100,
  B00100,
  B01110,
  B01110,
  B11111,
  B11111,
  B11111,
  B00000
};

volatile bool knobTurned;
volatile int8_t knobTurn = 0;

char line0[20] = { ' ' };
char line1[20] = { ' ' };
char line2[20] = { ' ' };
char line3[20] = { ' ' };

byte menuLevel = 0;
byte subMenuOpen = 0;
int8_t subMenuIndex = 0;
int8_t mainMenuIndex = 0;
byte mainMenuLength;
byte subMenuLength;

uint8_t mode = 0;

#define LONG_CLICK_DELAY 500
bool encoderBtnPressed = false;
bool encoderBtnClicked = false;
bool encoderBtnLongClick = false;
uint32_t lastEncoderClick = 0;

#define DIGITS_LENGTH 12
int8_t digits[DIGITS_LENGTH] = { 0 };
int8_t floatPower;

int8_t numChangeSelector = 0;
byte numChangeSelectorMax = 2;
bool numChange = false;
bool setUpNumChange = true;
bool addDigit = false;
byte valuePointer = 0;
byte arrowPointer = 0;
byte editIndex = 0;

int32_t numberToEdit;
int32_t inputNumber = -33201;
int * inputIntAdress;
float floatToEdit;
float inputFloat = 1228.625;
float * inputFloatAdress;
byte floatPrecision = 0;

byte numChangeHeaderSize = 0;
String numChangeHeader;

uint32_t lastRadioInMillis = 0;
uint32_t lastRadioOutMillis = 0;
#define RADIO_TIMEOUT 1000

SdCard card;
Fat16 file;

// store error strings in flash to save RAM
#define error(s) error_P(PSTR(s))

#define ENCODER_BTN_PIN A2
int16_t brightness = 255;

void setup() 
{  
  Serial.begin(9600);
  
  enableInterrupt(A0, A_RISE, RISING);
  enableInterrupt(A1, B_RISE, RISING);
  
  enableInterrupt(ENCODER_BTN_PIN, BTN_PRESS, CHANGE);

  pinMode(10, OUTPUT);
  pinMode(6, OUTPUT);

  SPI.setClockDivider(SPI_CLOCK_DIV4); 

  digitalWrite(10,HIGH);
  delay(10);
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

  digitalWrite(10,LOW);
  delay(10);
  
  // initialize the SD card
  if (!card.begin(10, SPI_CLOCK_DIV4)) error("card.begin");
  
  // initialize a FAT16 volume
  if (!Fat16::init(&card)) error("Fat16::init");
  
  
  // set up the LCD's number of columns and rows:
  lcd.init();
  lcd.backlight();

  for (byte i = 0; i < brightness; i++)
  {
    analogWrite(6,i);
    delay(4);
  }
  analogWrite(6,brightness);
  
  lcd.createChar(1, newChar1);
  lcd.createChar(2, newChar2);
  lcd.createChar(3, newChar3);
  lcd.createChar(4, newChar4);
  lcd.home();

  // open a file
  if (file.open("mainMenu.TXT", O_READ)) 
  {
    lcd.clear();
    lcd.print("Loaded SD card! Win!");
    mainMenuLength = file.read() - 48;
    Serial.print("mainMenuLength: ");
    Serial.println(mainMenuLength);
    file.close();
    delay(500);
    lcd.clear();
  } 
  else
  {    
    lcd.clear();
    lcd.print("Shit, where is that ");
    lcd.setCursor(0,1);
    lcd.print("fuckin SD card, bro?");
    error("file.open");
  }
  
  ESCswitch = true;
  programming = false;

  radioDataOut0.ESCswitch = ESCswitch;
  radioDataOut0.programming = programming;
  // settings:
  radioDataOut0.steeringDeadzone = steeringDeadzone;
  radioDataOut0.straightSensitivity = straightSensitivity;
  radioDataOut0.ESC_deadzone = ESC_deadzone; 
  radioDataOut0.brake_deadzone = brake_deadzone;
  radioDataOut0.oversteer_yaw_difference = oversteer_yaw_difference;
  radioDataOut0.oversteer_yaw_release = oversteer_yaw_release;
  radioDataOut0.code = R_CODE_0;
  
  radioDataOut1.understeerXdiff = understeerXdiff;
  radioDataOut1.understeerYdiff = understeerYdiff;
  radioDataOut1.oversteer_steering_multi = oversteer_steering_multi; 
  radioDataOut1.oversteer_throttle_corr = oversteer_throttle_corr;
  radioDataOut1.understeer_throttle_corr = understeer_throttle_corr;
  radioDataOut1.straightline_corr = straightline_corr;
  radioDataOut1.wheelCircumference = wheelCircumference;
  radioDataOut1.gMode = gMode;
  radioDataOut1.engine_throttle_corr = 0;
  radioDataOut1.code = R_CODE_1;
  
  radioDataIn.state = 255;
  mode = MODE_HOMESCREEN;
}

void loop() 
{    
  uint32_t commonMillis = millis();
  static bool radioInput;
  if (encoderBtnPressed)
  {
    if ((lastEncoderClick + LONG_CLICK_DELAY) < commonMillis)
    {
      encoderBtnLongClick = true;      
      encoderBtnPressed = false;
    }
  }
  
  if (mode == MODE_HOMESCREEN)
  {
    if (knobTurned)
    {
      int8_t turns = getKnobTurn();
      Serial.println(turns);
      /// until use free memory
      /// int8_t increment = turns / abs(turns);
      int8_t increment = turns / abs(turns);
      while (abs(turns) > 0)
      {
        brightness += increment * 5;
        brightness = constrain(brightness, 0, 255);
        turns -= increment; // this moves the value one step closer to 0 regardless of its sign
      }
      analogWrite(6,brightness);
    }
    if (commonMillis > (lastLCDMillis + 100))
    {
      lastLCDMillis = commonMillis;
      String stateString;
      switch(state)
      {
        case ST_BALANCED:
          stateString = F("Balanced  ");
          break;
        case ST_UNDERSTEER:
          stateString = F("Understeer");
          break;
        case ST_OVERSTEER:
          stateString = F("Oversteer ");
          break;
        case ST_ROLLED:
          stateString = F("Rolled    ");
          break;  
        default:
          stateString = F("N/A       ");
          break;        
      }  
     
      byte i = 0;
      for (; i < 20; i++)
      {
        line0[i] = ' ';
        line1[i] = ' ';
      }
      float speedometer = dTrainSpeed * 3.6;
      if (speedometer >= 10) { lcd.setCursor(1,0); i = 1; }
      else { lcd.setCursor(2,0); i = 2; }
   /* char number[4] = { ' ' };
      /// TODO: revise this: :D
      // sprintf (number, "%f", 2,speedometer);
      // dtostrf(float, char[] length, decimals, char[]);
      dtostrf(speedometer, 4, 0, number);
      lcd.setCursor(16,3);
      lcd.write(number[0]);
      lcd.write(number[1]);
      lcd.write(number[2]);
      lcd.write(number[3]);*/
      lcd.setCursor(2,0);
      lcd.print(speedometer,0);
      lcd.print(" km/h"); 
      if (dTrainSpeed >= 10) { lcd.setCursor(1,1); }
      else { lcd.setCursor(2,1); }
      lcd.print(dTrainSpeed,1);
      lcd.print(" m/s"); 
      lcd.setCursor(11,0);
      lcd.print(stateString);
      if (engineTemp >= 10) { lcd.setCursor(1,2); }
      else { lcd.setCursor(2,2); }
      lcd.print(engineTemp,1);
      lcd.write(1);
      lcd.print("C");
      lcd.setCursor(2,3);
      lcd.print(batteryV,2);
      lcd.print(" V");          
      if (radioInput)
      {
        lcd.setCursor(11,1);
        if (safetyRelayState) { lcd.print(F("ESC ON ")); }
        else { lcd.print(F("ESC OFF")); }
        lcd.setCursor(11,2);      
        if (glowActive) { lcd.print(F("GLOW ON ")); }
        else { lcd.print(F("GLOW OFF")); }
      }   
      else
      {
        lcd.setCursor(11,1);
        lcd.print(F("WAITING "));
        lcd.setCursor(11,2);      
        lcd.print(F(" FOR CAR"));
      }
    }
    if (encoderBtnClicked)
    {    
      encoderBtnClicked = false;
      lcd.clear();
      setKnobTurn(0);
      mainMenuIndex = 0;
      mode = MODE_MENU;   
    }
    if (encoderBtnLongClick)
    {
      encoderBtnLongClick = false;
      ESCswitch = !ESCswitch;
      radioDataOut0.ESCswitch = ESCswitch;
      programming = true;
      lcd.setCursor(11,1);
      lcd.print(F("WAITING"));
    }
  }
  else if (mode == MODE_MENU)
  {
    if (knobTurned)
    {    
      int8_t turns = getKnobTurn();
      Serial.println(turns);
      int8_t increment = turns / abs(turns);
      if (menuLevel == 0)
      {
        while (abs(turns) > 0)
        {
          mainMenuIndex += increment;
          if (mainMenuIndex >= mainMenuLength)
          {
            mainMenuIndex = 0;
          }
          else if (mainMenuIndex < 0)
          {
            mainMenuIndex = mainMenuLength - 1;
          }
          turns -= increment; // this moves the value one step closer to 0 regardless of its sign
        }
        file.open("mainMenu.TXT", O_READ);
        // file.seekSet(0);
        uint8_t index = 255;
        uint8_t currentByte;
        while ((currentByte = file.read()) > 0)
        {      
          if (currentByte == 35) // 35 is #
          {
            index = (uint8_t) file.read();
            index -= 48; // 48 is zero
          }
          if (index == mainMenuIndex)
          {
            for (byte i = 0; i < 20; i++)
            {
              line0[i] = file.read();
            }
            break;
            /// if (file.read() != $) { read another line }
          }
        }
        file.close();
      }
      else if (menuLevel == 1)
      {
        switch (subMenuOpen)
        {
          case 0:
            file.open("subMenu0.TXT", O_READ);
            break;
          case 1:
            file.open("subMenu1.TXT", O_READ);
            break;
          case 2:
            file.open("subMenu2.TXT", O_READ);
            break;
          case 3:
            file.open("subMenu3.TXT", O_READ);
            break;
          case 4: 
            file.open("subMenu4.TXT", O_READ);
            break;
          case 5:
            file.open("subMenu5.TXT", O_READ);
            break;
          default:
            file.open("mainMenu.TXT", O_READ);
            break;
        }      
        subMenuLength = file.read() - 48;
        Serial.println(subMenuLength);
        while (abs(turns) > 0)
        {
          subMenuIndex += increment;
          if (subMenuIndex >= subMenuLength)
          {
            subMenuIndex = 0;
          }
          else if (subMenuIndex < 0)
          {
            subMenuIndex = subMenuLength - 1;
          }
          turns -= increment; // this moves the value one step closer to 0 regardless of its sign
        }
        
        // file.seekSet(0);
        uint8_t index = 255;
        uint8_t currentByte;
        while ((currentByte = file.read()) > 0)
        {      
          if (currentByte == 35) // 35 is #
          {
            index = (uint8_t) file.read();
            index -= 48; // 48 is zero
          }
          if (index == subMenuIndex)
          {
            for (byte i = 0; i < 20; i++)
            {
              line1[i] = file.read();
            }
            break;
            /// if (file.read() != $) { read another line }
          }
        }
        file.close();
      }          
    }
    if (commonMillis > (lastLCDMillis + 100))
    {
      lastLCDMillis = commonMillis;
      
      if (menuLevel == 0)
      {        
        lcd.setCursor(0, 0);
        for (byte i = 0; i < 20; i++) lcd.write(line0[i]);
        /*lcd.print(line0);/*
        if (mainMenuIndex == (mainMenuLength - 3))
        {
          lcd.setCursor(6, 1);
          /// for unknown reasons, this doesn't work as expected...
          if (safetyRelayState) { lcd.print("ESC is ON "); }
          else { lcd.print("ESC is OFF"); }
        }
        else
        {
          lcd.setCursor(0, 1);
          lcd.print("                    ");
        }*/

        
      }
      if (menuLevel == 1)
      {
        lcd.setCursor(0, 1);
        for (byte i = 0; i < 20; i++) lcd.write(line1[i]);
      }
    }
    if (encoderBtnClicked)
    {    
      encoderBtnClicked = false;
      if (menuLevel == 0 )
      {
        if (mainMenuIndex == (mainMenuLength - 3))
        {
          ESCswitch = !ESCswitch;
          radioDataOut0.ESCswitch = ESCswitch;
          programming = true;
        }
        else if (mainMenuIndex == (mainMenuLength - 2))
        {          
          programming = true;
          radioDataOut0.programming = programming;
        }
        else if (mainMenuIndex == (mainMenuLength - 1))
        {
          lcd.clear();
          mode = MODE_HOMESCREEN;
        }
        else if (mainMenuIndex >= 0 && mainMenuIndex <= 5)
        {
          subMenuOpen = mainMenuIndex;
          menuLevel = 1;
          subMenuIndex = 0;
          setKnobTurn(0);
        }
        else
        {
          
        } 
      }
      else if (menuLevel == 1)
      {
        if (mainMenuIndex == 0) // Oversteer settings
        {  
          if (subMenuIndex == 0) // On-trigger yaw
          {          
            mode = MODE_FLOATCHANGE;
            setUpNumChange = true;
            numChangeSelector = 0;     
            inputFloatAdress = &oversteer_yaw_difference;     
            floatToEdit = *inputFloatAdress;
            numChangeHeader = "D/s^2: ";
            numChangeHeader.replace('D',(char) B11011111);
            numChangeHeaderSize = 7;
            floatPrecision = 3;
          }
          if (subMenuIndex == 1) // Off-trigger yaw
          {          
            mode = MODE_FLOATCHANGE;
            setUpNumChange = true;
            numChangeSelector = 0;     
            inputFloatAdress = &oversteer_yaw_release;     
            floatToEdit = *inputFloatAdress;
            numChangeHeader = "D/s^2: ";
            numChangeHeader.replace('D',(char) B11011111);
            numChangeHeaderSize = 7;
            floatPrecision = 3;
          }
          if (subMenuIndex == 2) // Steering correction
          {          
            mode = MODE_FLOATCHANGE;
            setUpNumChange = true;
            numChangeSelector = 0;     
            inputFloatAdress = &oversteer_steering_multi;     
            floatToEdit = *inputFloatAdress;
            numChangeHeader = "multi: ";
            numChangeHeaderSize = 7;
            floatPrecision = 3;
          }
          if (subMenuIndex == 3) // Throttle correction
          {          
            mode = MODE_NUMBERCHANGE;
            setUpNumChange = true;
            numChangeSelector = 0;    
            inputIntAdress = &oversteer_throttle_corr;      
            numberToEdit = *inputIntAdress;
            numChangeHeader = "corr: ";
            numChangeHeaderSize = 6;
          }
          // 4Detection on/off    
          // 5Correction on/off   
          // 6Back to main menu 
        }
        else if (mainMenuIndex == 1) // Understeer settings
        {
          //  0X-axis trigger acc. 
          //  1Y-axis trigger acc. 
          //  2Detection on/off    
          //  3Correction on/off   
          //  4Back to main menu   
        } 
        else if (mainMenuIndex == 2) // Veering settings
        {
          
        }
        else if (mainMenuIndex == 3) // Other ESC settings
        {
          
        }
        /* prototypes:
        if (subMenuIndex == 0)
        {          
          mode = MODE_FLOATCHANGE;
          setUpNumChange = true;
          numChangeSelector = 0;          
          floatToEdit = inputFloat;
          numChangeHeader = "VALUE: ";
          numChangeHeaderSize = 7;
          floatPrecision = 3;
        }
        if (subMenuIndex == 1)
        {          
          mode = MODE_NUMBERCHANGE;
          setUpNumChange = true;
          numChangeSelector = 0;          
          numberToEdit = inputNumber;
          numChangeHeader = "VALUE: ";
          numChangeHeaderSize = 7;
        }*/
        /// Back to main menu
        if (subMenuIndex == (subMenuLength - 1))
        {
          menuLevel = 0;
          lcd.clear();
        }        
      }     
    }  
  }
  else if (mode == MODE_NUMBERCHANGE)
  {
    if (setUpNumChange)
    {
      lcd.clear();
      for (byte i = 0; i < 20; i++) lcd.write(line1[i]);
      setUpNumChange = false;           
      byte i = numberToDigits(numberToEdit);  
      if (addDigit) 
      { 
        addDigit = false; 
        digits[0]++; 
        i--; 
        digits[i] = 1; 
        numChangeSelector = 1;
        numberToEdit = digitsToNumber();
      }
      valuePointer = (20 - (digits[0] + numChangeHeaderSize)) / 2;      
      lcd.setCursor(valuePointer,1);
      lcd.print(numChangeHeader);
      if (digits[1] == 1) 
      { 
        lcd.setCursor((valuePointer + numChangeHeaderSize - 1),1); 
        lcd.print("-"); 
      } 
      while (i < DIGITS_LENGTH) { lcd.print(digits[i]); i++; }
      numChangeSelectorMax = digits[0] + 2;
      lcd.setCursor(0,3);
      lcd.print("Cancel");
      lcd.setCursor(14,3);
      lcd.print("Accept");
    }
    if (knobTurned)
    {
      int8_t turns = getKnobTurn();
      Serial.println(turns);
      int8_t increment = turns / abs(turns);
      if (!numChange)
      {
        while (abs(turns) > 0)
        {
          numChangeSelector += increment;
          if (numChangeSelector > numChangeSelectorMax)
          {
            // numChangeSelector = 0;
            numChangeSelector = numChangeSelectorMax;
          }
          else if (numChangeSelector < 0)
          {
            // numChangeSelector = numChangeSelectorMax;
            numChangeSelector = 0;
          }
          turns -= increment; // this moves the value one step closer to 0 regardless of its sign
        } 
        lcd.setCursor(valuePointer + numChangeHeaderSize + digits[0],1);
        lcd.print(" ");
      }
      else
      {
        if (numChangeSelector == numChangeSelectorMax - 1)
        {
          while (abs(turns) > 0)
          {
            numberToEdit += increment;
            turns -= increment; // this moves the value one step closer to 0 regardless of its sign
          } 
          byte i = numberToDigits(numberToEdit);      
          valuePointer = (20 - (digits[0] + numChangeHeaderSize)) / 2;          
          lcd.setCursor(valuePointer + numChangeHeaderSize - 1,1);
          if (numberToEdit < 0) 
          { 
            lcd.print("-");
          }
          else
          {
            lcd.print(" ");
          }
          while (i < DIGITS_LENGTH) { lcd.print(digits[i]); i++; }
        }
        else
        {
          int8_t digit = digits[editIndex];
          while (abs(turns) > 0)
          {
            digit += increment;
            if (digit < 0)
            {
              digit = 9;
            }
            else if (digit > 9)
            {
              digit = 0;
            }
            turns -= increment; // this moves the value one step closer to 0 regardless of its sign
          }
          digits[editIndex] = digit;
          lcd.setCursor(arrowPointer,1);
          lcd.print(digits[editIndex]);
        }        
      }
      /// clear some stuff:
      lcd.setCursor(0,2);
      lcd.print("                    ");
      lcd.setCursor(6,3);
      lcd.print(" ");
      lcd.setCursor(13,3);
      lcd.print(" ");
    }
    if (commonMillis > (lastLCDMillis + 100))
    {
      lastLCDMillis = commonMillis;
      
      if (numChangeSelector == 0)
      {
        lcd.setCursor(6,3);
        lcd.print("<");
      }
      if (numChangeSelector == numChangeSelectorMax - 1)
      {
        lcd.setCursor(valuePointer + numChangeHeaderSize + digits[0],1);
        if (numChange)
        {
          lcd.write(2);
        }
        else
        {
          lcd.write(60);
        }  
      }
      if (numChangeSelector == numChangeSelectorMax)
      {
        lcd.setCursor(13,3);
        lcd.print(">");
      }
      if (numChangeSelector > 0 && numChangeSelector < numChangeSelectorMax - 1)
      {
        arrowPointer = valuePointer + numChangeHeaderSize + (numChangeSelector - 1);
        lcd.setCursor(arrowPointer, 2);
        if (numChange)
        {
          lcd.write(4);
        }
        else
        {
          lcd.write(3);
        }        
      }
    }
    if (encoderBtnClicked)
    {    
      encoderBtnClicked = false;
      if (numChangeSelector == 0)
      {
        lcd.setCursor(2,2);
        lcd.print("Cancelled...");
        delay(500);
      }
      else if (numChangeSelector == numChangeSelectorMax)
      {
        *inputIntAdress = numberToEdit;
        lcd.setCursor(2,2);
        lcd.print("Accepted...");
        delay(500);
      }
      if (numChangeSelector == 0 || numChangeSelector == numChangeSelectorMax)
      {
        lcd.clear();
        for (byte i = 0; i < 20; i++) lcd.write(line0[i]);
        lcd.setCursor(0,1);
        for (byte i = 0; i < 20; i++) lcd.write(line1[i]);
        
        mode = MODE_MENU;
      }
      else if (numChangeSelector > 0 && numChangeSelector < numChangeSelectorMax - 1)
      {
        if (numChange)
        {
          numChange = false;
          numberToEdit = digitsToNumber();
          setUpNumChange = true;
        }
        else 
        {
          numChange = true;
          byte firstDigitIndex = numberToDigits(numberToEdit);
          valuePointer = (20 - (digits[0] + numChangeHeaderSize)) / 2;
          arrowPointer = valuePointer + numChangeHeaderSize + (numChangeSelector - 1);
          editIndex = firstDigitIndex + arrowPointer - (valuePointer + numChangeHeaderSize); 
        }               
      }
      else if (numChangeSelector == numChangeSelectorMax - 1)
      {
        if (numChange)
        {
          numChange = false;
          numberToEdit = digitsToNumber();
          setUpNumChange = true;
        }
        else 
        {
          numChange = true;
        }
      }
    }
    if (encoderBtnLongClick)
    {
      encoderBtnLongClick = false;
      if (numChangeSelector == numChangeSelectorMax - 1)
      {
        /// Change polarity
        if (digits[1] == 1) { digits[1] = 0; }
        else { digits[1] = 1; }
        numberToEdit = digitsToNumber();
        setUpNumChange = true;
      }
      if (numChangeSelector == 1)
      {
        /// Add a digit                
        if ((digits[0] + 1) <= (DIGITS_LENGTH - 2)) { addDigit = true; }        
        setUpNumChange = true;
      }
    }
  }
  else if (mode == MODE_FLOATCHANGE)
  {
    if (setUpNumChange)
    {
      lcd.clear();
      for (byte i = 0; i < 20; i++) lcd.write(line1[i]);
      setUpNumChange = false;           
      byte i = floatToDigits(floatToEdit);  
      if (addDigit) 
      { 
        addDigit = false; 
        digits[0]++;          
        floatPower++;       
        i--; 
        digits[i] = 1; 
        numChangeSelector = 1;
        floatToEdit = digitsToFloat();
      }
      valuePointer = (20 - (digits[0] + numChangeHeaderSize + 1)) / 2;      
      lcd.setCursor(valuePointer,1);
      lcd.print(numChangeHeader);
      if (digits[1] == 1) 
      { 
        lcd.setCursor((valuePointer + numChangeHeaderSize - 1),1); 
        lcd.print("-"); 
      }       
      byte dot = i + floatPower;
      while (i < DIGITS_LENGTH) 
      { 
        lcd.print(digits[i]); 
        if (i == dot) lcd.print(".");
        i++; 
      }
      numChangeSelectorMax = digits[0] + 3;
      lcd.setCursor(0,3);
      lcd.print("Cancel");
      lcd.setCursor(14,3);
      lcd.print("Accept");
    }
    if (knobTurned)
    {
      int8_t turns = getKnobTurn();
      Serial.println(turns);
      int8_t increment = turns / abs(turns);
      if (!numChange)
      {
        while (abs(turns) > 0)
        {
          numChangeSelector += increment;
          if (numChangeSelector > numChangeSelectorMax)
          {
            // numChangeSelector = 0;
            numChangeSelector = numChangeSelectorMax;
          }
          else if (numChangeSelector < 0)
          {
            // numChangeSelector = numChangeSelectorMax;
            numChangeSelector = 0;
          }
          if (numChangeSelector == floatPower + 2) numChangeSelector += increment;
          turns -= increment; // this moves the value one step closer to 0 regardless of its sign
        } 
        lcd.setCursor(valuePointer + numChangeHeaderSize + digits[0] + 1,1);
        lcd.print(" ");
      }
      else
      {
        if (numChangeSelector == numChangeSelectorMax - 1)
        {
          float numberToIncrement = floatToEdit;
          byte i = floatPrecision;
          while (i > 0)
          {
            numberToIncrement *= 10; 
            i--;
          }
          numberToIncrement = (int32_t) numberToIncrement;
          while (abs(turns) > 0)
          {
            numberToIncrement += increment;
            turns -= increment; // this moves the value one step closer to 0 regardless of its sign
          } 
          i = floatPrecision;
          while (i > 0)
          {
            numberToIncrement /= 10; 
            i--;
          }
          floatToEdit = numberToIncrement;
          i = floatToDigits(floatToEdit);      
          valuePointer = (20 - (digits[0] + numChangeHeaderSize + 1)) / 2;          
          lcd.setCursor(valuePointer + numChangeHeaderSize - 1,1);
          if (floatToEdit < 0) 
          { 
            lcd.print("-");
          }
          else
          {
            lcd.print(" ");
          }
          byte dot = i + floatPower;
          while (i < DIGITS_LENGTH) 
          { 
            lcd.print(digits[i]); 
            if (i == dot) lcd.print(".");
            i++; 
          }
        }
        else
        {
          int8_t digit = digits[editIndex];
          while (abs(turns) > 0)
          {
            digit += increment;
            if (digit < 0)
            {
              digit = 9;
            }
            else if (digit > 9)
            {
              digit = 0;
            }
            turns -= increment; // this moves the value one step closer to 0 regardless of its sign
          }
          digits[editIndex] = digit;
          lcd.setCursor(arrowPointer,1);
          lcd.print(digits[editIndex]);
        }        
      }
      /// clear some stuff:
      lcd.setCursor(0,2);
      lcd.print("                    ");
      lcd.setCursor(6,3);
      lcd.print(" ");
      lcd.setCursor(13,3);
      lcd.print(" ");
    }
    if (commonMillis > (lastLCDMillis + 100))
    {
      lastLCDMillis = commonMillis;
      
      if (numChangeSelector == 0)
      {
        lcd.setCursor(6,3);
        lcd.print("<");
      }
      if (numChangeSelector == numChangeSelectorMax - 1)
      {
        lcd.setCursor(valuePointer + numChangeHeaderSize + digits[0] + 1,1);
        if (numChange)
        {
          lcd.write(2);
        }
        else
        {
          lcd.write(60);
        }  
      }
      if (numChangeSelector == numChangeSelectorMax)
      {
        lcd.setCursor(13,3);
        lcd.print(">");
      }
      if (numChangeSelector > 0 && numChangeSelector < numChangeSelectorMax - 1)
      {
        arrowPointer = valuePointer + numChangeHeaderSize + (numChangeSelector - 1);
        lcd.setCursor(arrowPointer, 2);
        if (numChange)
        {
          lcd.write(4);
        }
        else
        {
          lcd.write(3);
        }        
      }
    }
    if (encoderBtnClicked)
    {    
      encoderBtnClicked = false;
      if (numChangeSelector == 0)
      {
        lcd.setCursor(2,2);
        lcd.print("Cancelled...");
        delay(500);
      }
      else if (numChangeSelector == numChangeSelectorMax)
      {
        *inputFloatAdress = floatToEdit;
        lcd.setCursor(2,2);
        lcd.print("Accepted...");
        delay(500);
      }
      if (numChangeSelector == 0 || numChangeSelector == numChangeSelectorMax)
      {
        lcd.clear();
        for (byte i = 0; i < 20; i++) lcd.write(line0[i]);
        lcd.setCursor(0,1);
        for (byte i = 0; i < 20; i++) lcd.write(line1[i]);
        
        mode = MODE_MENU;
      }
      else if (numChangeSelector > 0 && numChangeSelector < numChangeSelectorMax - 1)
      {
        if (numChange)
        {
          numChange = false;
          floatToEdit = digitsToFloat();
          setUpNumChange = true;
        }
        else 
        {
          numChange = true;
          byte firstDigitIndex = floatToDigits(floatToEdit);
          valuePointer = (20 - (digits[0] + numChangeHeaderSize + 1)) / 2;
          arrowPointer = valuePointer + numChangeHeaderSize + (numChangeSelector - 1);
          // recalculate this for float
          editIndex = firstDigitIndex + arrowPointer - (valuePointer + numChangeHeaderSize); 
          if (arrowPointer - (valuePointer + numChangeHeaderSize) > floatPower) editIndex--;
        }               
      }
      else if (numChangeSelector == numChangeSelectorMax - 1)
      {
        if (numChange)
        {
          numChange = false;
          floatToEdit = digitsToFloat();
          setUpNumChange = true;
        }
        else 
        {
          numChange = true;
        }
      }
    }
    if (encoderBtnLongClick)
    {
      encoderBtnLongClick = false;
      if (numChangeSelector == numChangeSelectorMax - 1)
      {
        /// Change polarity
        if (digits[1] == 1) { digits[1] = 0; }
        else { digits[1] = 1; }
        floatToEdit = digitsToFloat();
        setUpNumChange = true;
      }
      if (numChangeSelector == 1)
      {
        /// Add a digit                
        if ((digits[0] + 1) <= (DIGITS_LENGTH - 2)) { addDigit = true; }
        setUpNumChange = true;
      }
    }
  }

  // bool radioStatusChanged = false;
  if(radio.available())
  {                                                             
    while (radio.available()) 
    {                                  
      radio.read( &radioDataIn, sizeof(radioDataIn));
    }  
    state = radioDataIn.state;
    gMode = radioDataIn.gMode;
    dTrainSpeed = radioDataIn.dTrainSpeed;
    batteryV = radioDataIn.batteryV;
    engineTemp = radioDataIn.engineTemp;
    glowActive = radioDataIn.glowActive;
    safetyRelayState = radioDataIn.safetyRelayState;
    /// radioStatusChanged = !(radioStatus == radioDataIn.radioStatus);
    radioStatus = radioDataIn.radioStatus;
    radioInput = true;
    lastRadioInMillis = commonMillis;
  }
  else
  {    
    if (lastRadioInMillis + RADIO_TIMEOUT < commonMillis)
    {
      radioInput = false;
    }    
  }  
  if (programming && (lastRadioOutMillis + 100 < commonMillis))
  {
    if (radioStatus == R_STAT_READY)
    {
      // settings:
      radioDataOut0.steeringDeadzone = steeringDeadzone;
      radioDataOut0.straightSensitivity = straightSensitivity;
      radioDataOut0.ESC_deadzone = ESC_deadzone; 
      radioDataOut0.brake_deadzone = brake_deadzone;
      radioDataOut0.oversteer_yaw_difference = oversteer_yaw_difference;
      radioDataOut0.oversteer_yaw_release = oversteer_yaw_release;
      radio.stopListening();    
      radio.write( &radioDataOut0, sizeof(radioDataOut0));          
      radio.startListening();
    }
    else if (radioStatus == R_STAT_ACK_0)
    {
      radioDataOut1.understeerXdiff = understeerXdiff;
      radioDataOut1.understeerYdiff = understeerYdiff;
      radioDataOut1.oversteer_steering_multi = oversteer_steering_multi; 
      radioDataOut1.oversteer_throttle_corr = oversteer_throttle_corr;
      radioDataOut1.understeer_throttle_corr = understeer_throttle_corr;
      radioDataOut1.straightline_corr = straightline_corr;
      radioDataOut1.wheelCircumference = wheelCircumference;
      radioDataOut1.gMode = gMode;
      radioDataOut1.engine_throttle_corr = 0;
      radio.stopListening();    
      radio.write( &radioDataOut1, sizeof(radioDataOut1));          
      radio.startListening();
    }
    else if (radioStatus == R_STAT_ACK_1)
    {
      programming = false;
    }    
    lastRadioOutMillis = commonMillis;
  }  
}

void A_RISE(){
 disableInterrupt(A0);
 A_SIG=1;
 
 if(B_SIG==0)
 pulses++;//moving forward
 if(B_SIG==1)
 pulses--;//moving reverse
 calcKnobTurn();
 enableInterrupt(A0, A_FALL, FALLING);
}

void A_FALL(){
 disableInterrupt(A0);
 A_SIG=0;
 
 if(B_SIG==1)
 pulses++;//moving forward
 if(B_SIG==0)
 pulses--;//moving reverse
 calcKnobTurn();
 enableInterrupt(A0, A_RISE, RISING);  
}

void B_RISE(){
 disableInterrupt(A1);
 B_SIG=1;
 
 if(A_SIG==1)
 pulses++;//moving forward
 if(A_SIG==0)
 pulses--;//moving reverse
 calcKnobTurn();
 enableInterrupt(A1, B_FALL, FALLING);
}

void B_FALL(){
 disableInterrupt(A1);
 B_SIG=0;
 
 if(A_SIG==0)
 pulses++;//moving forward
 if(A_SIG==1)
 pulses--;//moving reverse
 calcKnobTurn();
 enableInterrupt(A1, B_RISE, RISING);
}

void calcKnobTurn()
{
  if (pulses >= 4)
  {
    setKnobTurn(-1);
    pulses = 0;
  }
  else if (pulses <= -4)
  {
    setKnobTurn(1);
    pulses = 0;
  }   
}

int8_t getKnobTurn()
{
  int8_t localTurn = 0;
  localTurn = knobTurn;    
  knobTurn = 0;      
  knobTurned = false;
  return localTurn;
}

void setKnobTurn(int8_t turn)
{
  knobTurn += turn;
  knobTurned = true;
}

void setCurrentMenuIndex()
{
  
}

void BTN_PRESS()
{
  if (digitalRead(ENCODER_BTN_PIN) == false)
  {
    encoderBtnPressed = true;
  }
  else
  {
    encoderBtnPressed = false;
    if ((lastEncoderClick + LONG_CLICK_DELAY) > millis())
    {
      encoderBtnClicked = true;
    }
  }
  lastEncoderClick = millis();
}

void error_P(const char* str) {
  PgmPrint("error: ");
  SerialPrintln_P(str);
  if (card.errorCode) {
    PgmPrint("SD error: ");
    Serial.println(card.errorCode, HEX);
  }
  while(1);
}

byte numberToDigits(int32_t input)
{
  int32_t number = input;
  byte i = 0;
  floatPower = 125;
  for (; i < DIGITS_LENGTH; i++) digits[i] = 0;
  if (number < 0)
  {
    digits[1] = 1;
    number *= -1;
  }
  i = DIGITS_LENGTH - 1;
  while (number > 0 && i > 1)
  {
    digits[i] = number % 10;
    number /= 10;
    i--;
  }
  i = 2;
  while (i < DIGITS_LENGTH)
  {
    if (digits[i] > 0) break;
    i++;
  }
  digits[0] = DIGITS_LENGTH - i;
  if (i == DIGITS_LENGTH) { digits[0] = 1; i--; }
  /// return index to start reading from
  return i;
}

byte floatToDigits(float input)
{
  float numberF = input;
  byte i = 0;  
  for (; i < DIGITS_LENGTH; i++) digits[i] = 0;
  if (numberF < 0)
  {
    digits[1] = 1;
    numberF *= -1;
  }
  float power = log(numberF) / log(10);
  if (power > 0 && power < 1) power = 1;
  floatPower = (byte) constrain(power, 0, 128);
  Serial.println(numberF,6);
  i = floatPrecision;
  if ((i + power) > DIGITS_LENGTH - 2)
  { 
    i = DIGITS_LENGTH - 2 - power;
    floatPrecision = i;
  }
  Serial.println(i);
  while (i > 0)
  {
    numberF *= 10;
    i--;
  }
  uint32_t number = numberF;
  Serial.println(number);
  i = DIGITS_LENGTH - 1;
  if (power < 0) i++;
  while (number > 0 && i > 1)
  {
    digits[i] = number % 10;
    number /= 10;
    i--;
  }
  i = 2;
  while (i < DIGITS_LENGTH)
  {
    if (digits[i] > 0) break;
    i++;
  }
  if (power < 0) { digits[0]++; i--; }
  digits[0] = DIGITS_LENGTH - i;
  if (i == DIGITS_LENGTH) { digits[0] = 2; i--; i--; }  
  Serial.println(power);
  Serial.println(digits[0]);
  /// return index to start reading from
  Serial.println(i);
  return i;
}

int32_t digitsToNumber()
{  
  int32_t number = 0;
  int32_t multiplier = 1;
  byte i = DIGITS_LENGTH - 1;
  while (i > 1) /// refine to previusly calculated length
  {    
    number += digits[i] * multiplier;
    multiplier *= 10;
    i--;
  }  
  if (digits[1] == 1) number *= -1;
  return number;
}

float digitsToFloat()
{  
  float number = 0;
  float multiplier = 1;
  byte i = DIGITS_LENGTH - 1;
  while (i > 1) /// refine to previusly calculated length
  {    
    number += digits[i] * multiplier;
    multiplier *= 10;
    i--;
  }
  multiplier = 1;
  i = floatPrecision;
  while (i > 0)
  {
    multiplier *= 10;
    i--;
  }
  number /= multiplier;
  if (digits[1] == 1) number *= -1;
  return number;
}

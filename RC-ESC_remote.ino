/// RC-ESC REMOTE
/////////////////////////////////////////////////////////////////////////////////////
///           Electronic Stability Control for RC Car v1.0 2016/03/26             ///
/////////////////////////////////////////////////////////////////////////////////////
/// IMPORTANT NOTICE: THIS PROJECT IS FAR FROM READY, SO USE IT AT YOUR OWN RISK!
/// Description coming soon...

#include <nRF24L01.h>
#include <RF24.h>
#include <RF24_config.h>

#include <SPI.h>
#include <RF24.h>

#include <Fat16.h>
#include <Fat16util.h>

#include <EnableInterrupt.h>
// #include <LiquidCrystal.h>

#include <LiquidCrystal_I2C.h>
#include <LCD_Buffer.h>

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
LCD_Buffer lcd_buf;

//// RADIO SETTINGS
bool radioNumber = 1;

RF24 radio(8,9);
byte addresses[][6] = {"1Node","2Node"};

uint8_t state;
uint8_t gMode;
float dTrainSpeed;
float batteryV;
float engineTemp;
bool glowActive;
bool safetyRelayState;

uint8_t radioStatus;

/* Bools in switches0
 *  0 ESCswitch
 *  1 programming
 *  2 glowActive
 *  3 safetyRelayState
 *  4 headlight
 *  5 neon
 *  6 manualGlow (0,0 noGlow; 1,0 manualGlow; 0,1 autoGlow; 1,1 startUpMode)
 *  7 enableGlow  */
byte switches0 = 0;
/* Bools in switches1
 *  0 Oversteer detection
 *  1 Oversteer correction
 *  2 Understeer detection
 *  3 Understeer correction
 *  4 Veering detection
 *  5 Veering correction
 *  6 ABS
 *  7 empty  */
byte switches1 = 0;

bool btn1 = false;
bool btn2 = false;

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

struct RADIO_DATA_OUT_1 {
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
int32_t inputNumber;
int * inputIntAdress;
float floatToEdit;
float inputFloat;
float * inputFloatAdress;
byte floatPrecision;

byte numChangeHeaderSize;
String numChangeHeader;

uint32_t lastRadioInMillis = 0;
uint32_t lastRadioOutMillis = 0;
#define RADIO_TIMEOUT 1000

SdCard card;
Fat16 file;

// store error strings in flash to save RAM
#define error(s) error_P(PSTR(s))

#define ENCODER_BTN_PIN A2
#define HEADLIGHT_BTN A3 
#define NEON_BTN 3
int16_t brightness = 255;

bool sendDataToCar = false;

void setup() 
{
  Serial.begin(9600);
  
  enableInterrupt(A0, A_RISE, RISING); 
  enableInterrupt(A1, B_RISE, RISING);
  
  enableInterrupt(ENCODER_BTN_PIN, BTN_PRESS, CHANGE);
  enableInterrupt(HEADLIGHT_BTN, headlightSwitch, FALLING);
  enableInterrupt(NEON_BTN, neonSwitch, FALLING);

  /*pinMode(10, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(NEON_BTN, INPUT);
  pinMode(HEADLIGHT_BTN, INPUT);*/

  SPI.setClockDivider(SPI_CLOCK_DIV4); 
 

  // set up the LCD
  lcd.init();  
  lcd.print("Loading...");

  for (byte i = 0; i < brightness; i++)
  {
    analogWrite(6,i);
    delay(4);
  }
  analogWrite(6,brightness);
  lcd.backlight();

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
  if (!card.begin(10, SPI_CLOCK_DIV4))
  {
    error("");
  }
  
  // initialize a FAT16 volume
  if (!Fat16::init(&card))
  {
    error("");  
  }
  
  lcd.createChar(1, newChar1);
  lcd.createChar(2, newChar2);
  lcd.createChar(3, newChar3);
  lcd.createChar(4, newChar4);
  lcd.home();

  // open a file
  if (file.open("mm.TXT", O_READ)) 
  {
    lcd.clear();
    lcd.print(F("Loaded SD card! Win!"));
    mainMenuLength = file.read() - 48;
    file.close();
    delay(500);
    lcd.clear();
  } 
  else
  {    
    lcd.clear();
    lcd.print(F("Shit, where is that "));
    lcd.setCursor(0,1);
    lcd.print(F("fuckin SD card, bro?"));
    error("file.open");
  }  

  writeBool(&switches0, 0, true); // ESCswitch = true;
  writeBool(&switches0, 1, false); // programming = false;

  radioDataOut0.switches0 = switches0;
  radioDataOut0.switches1 = switches1;
  // settings:

  writeSDSettings();
  readSDSettings();

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
      if (brightness == 255)
      {
        lcd.backlight();
      }
      else
      {
        lcd.noBacklight();
        analogWrite(6,brightness);
      }      
    }
    if (radioInput)
    {
      // Later I can revise this block, so it only refreshes the zones that change,
      // So even in the buffer constants can stay untouched.
      
      // lcd_buf.clear();
      
      String stateString;
      switch(state)
      {
        case ST_BALANCED:
          stateString = F("Balanced");
          break;
        case ST_UNDERSTEER:
          stateString = F("Underst.");
          break;
        case ST_OVERSTEER:
          stateString = F("Overst.");
          break;
        case ST_ROLLED:
          stateString = F("Rolled");
          break;  
        default:
          stateString = F("N/A");
          break;        
      }  
      
      // Show speed in km/h:
      float speedometer = dTrainSpeed * 3.6;
      if (speedometer >= 10) { lcd_buf.setCursor(1,0); }
      else { lcd_buf.setCursor(2,0); }
      lcd_buf.print(speedometer,0);
      lcd_buf.print(F(" km/h"));
            
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
      
      // Show speed in m/s (informative for tests): 
      if (dTrainSpeed >= 10) { lcd_buf.setCursor(1,1); }
      else { lcd_buf.setCursor(2,1); }
      lcd_buf.print(dTrainSpeed,1);
      lcd_buf.print(F(" m/s")); 
      
      // Show state of the car:
      lcd_buf.setCursor(11,0);
      lcd_buf.print(stateString);

      // Show engine temp:
      if (engineTemp >= 10) { lcd_buf.setCursor(1,2); }
      else { lcd_buf.setCursor(2,2); }
      lcd_buf.print(engineTemp,1);
      lcd_buf.write(1);
      lcd_buf.print("C");

      // Show battery voltage:
      lcd_buf.setCursor(2,3);
      lcd_buf.print(batteryV,2);
      lcd_buf.print(" V");   

      // Show ESC relay state
      lcd_buf.setCursor(11,1);
      if (safetyRelayState) { lcd_buf.print(F("ESC ON ")); }
      else { lcd_buf.print(F("ESC OFF")); }

      // Show glow relay state
      lcd_buf.setCursor(11,2);      
      if (glowActive) { lcd_buf.print(F("GLOW ON ")); }
      else { lcd_buf.print(F("GLOW OFF")); }
        
      lcd_buf.setCursor(11, 3);
      lcd_buf.print(readBool(&switches0, 4), BIN);
      lcd_buf.print(" ");
      lcd_buf.print(readBool(&switches0, 5), BIN);
    }   
    else
    {
      lcd_buf.setCursor(11,1);
      lcd_buf.print(F("WAITING "));
      lcd_buf.setCursor(11,2);      
      lcd_buf.print(F(" FOR CAR"));
    }
    if (true)
    {
      
    }
    if (encoderBtnClicked)
    {    
      encoderBtnClicked = false;
      lcd.clear();
      lcd_buf.clear();
      setKnobTurn(0);
      mainMenuIndex = 0;
      mode = MODE_MENU;   
    }
    if (encoderBtnLongClick)
    {
      encoderBtnLongClick = false;
      flipBool(&switches0, 0);      
      sendDataToCar = true;
      radioDataOut0.switches0 = switches0;
      lcd_buf.setCursor(11,1);
      lcd_buf.print(F("WAITING"));
    }
    if (btn1)
    {
      btn1 = false;
      flipBool(&switches0, 4);
      sendDataToCar = true;
      radioDataOut0.switches0 = switches0;
    }
    if (btn2)
    {
      btn2 = false;
      flipBool(&switches0, 5);
      sendDataToCar = true;
      radioDataOut0.switches0 = switches0;
    }
  }
  else if (mode == MODE_MENU)
  {
    if (knobTurned)
    {   
      /* 
      int8_t turns = getKnobTurn();
      Serial.println(turns);
      int8_t increment = turns / abs(turns);*/
      if (menuLevel == 0)
      {
        useKnobTurn(&mainMenuIndex, getKnobTurn(), 0, mainMenuLength - 1);/*
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
        }*/
        readSDmenu("mm.TXT", mainMenuIndex, 0);/*
        file.open("mm.TXT", O_READ);
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
            lcd_buf.setCursor(0,0);
            for (byte i = 0; i < 20; i++)
            {
              lcd_buf.write(file.read());
            }
            break;
            /// if (file.read() != $) { read another line }
          }
        }
        file.close();*/
      }
      else if (menuLevel == 1)
      { 
        char* fileName;
        switch (subMenuOpen)
        {
          case 0:
            fileName = "sm0.TXT";
            break;
          case 1:
            fileName = "sm1.TXT";
            break;
          case 2:
            fileName = "sm2.TXT";
            break;
          case 3:
            fileName = "sm3.TXT";
            break;
          case 4: 
            fileName = "sm4.TXT";
            break;
          case 5:
            fileName = "sm5.TXT";
            break;
          default:
            fileName = "mm.TXT";
            break;
        }              
        file.open(fileName, O_READ);
        subMenuLength = file.read() - 48;
        file.close();
        
        useKnobTurn(&subMenuIndex, getKnobTurn(), 0, subMenuLength - 1);
        readSDmenu(fileName, subMenuIndex, 1);
        readSDmenu("mm.TXT", mainMenuIndex, 0);/*
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
            lcd_buf.setCursor(0,1);
            for (byte i = 0; i < 20; i++)
            {
              lcd_buf.write(file.read());
            }
            break;
            /// if (file.read() != $) { read another line }
          }
        }
        file.close();
        
        file.open("mm.TXT", O_READ);
        // file.seekSet(0);
        index = 255;
        while ((currentByte = file.read()) > 0)
        {      
          if (currentByte == 35) // 35 is #
          {
            index = (uint8_t) file.read();
            index -= 48; // 48 is zero
          }
          if (index == mainMenuIndex)
          {
            lcd_buf.setCursor(0,0);
            for (byte i = 0; i < 20; i++)
            {
              lcd_buf.write(file.read());
            }
            break;
            /// if (file.read() != $) { read another line }
          }
        }
        file.close();*/
      }          
    }
    if (true)
    {
      if (menuLevel == 0)
      { 
        if (mainMenuIndex == (mainMenuLength - 3))
        {
          /// ESC switch on/off
          lcd_buf.setCursor(6, 1);
          lcd_buf.print(F("ESC is "));
          if (safetyRelayState) { lcd_buf.print(F("ON ")); }
          else { lcd_buf.print(F("OFF")); }
        }
        else if (mainMenuIndex == (mainMenuLength - 2))
        {    
          /// Send program NOW!      
          lcd_buf.setCursor(0, 1);
          if (readBool(&switches0, 1)) { lcd_buf.print(F("Sending program...")); }
          else { lcd_buf.clearLine(1); }
        }      
        else
        {
          lcd_buf.clearLine(1);  
        }
      }
      if (menuLevel == 1)
      {
        /// TODO: Display current values for submenu items!
        /// Idea: modify the click action tree, to write values to lcd_buf
      }
    }
    if (encoderBtnClicked)
    {    
      encoderBtnClicked = false;
      if (menuLevel == 0 )
      {
        if (mainMenuIndex == (mainMenuLength - 3))
        {
          /// ESC switch on/off
          flipBool(&switches0, 0);      
          sendDataToCar = true;
          radioDataOut0.switches0 = switches0;
        }
        else if (mainMenuIndex == (mainMenuLength - 2))
        {    
          /// Send program NOW!      
          writeBool(&switches0, 1, true);
          sendDataToCar = true;
          radioDataOut0.switches0 = switches0;
        }
        else if (mainMenuIndex == (mainMenuLength - 1))
        {
          /// Back to homescreen
          lcd.clear();
          lcd_buf.clear();
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
            inputFloatAdress = &radioDataOut0.oversteer_yaw_difference;     
            floatToEdit = *inputFloatAdress;
            numChangeHeader = F("D/s^2: ");
            numChangeHeader.replace('D',(char) B11011111);
            numChangeHeaderSize = 7;
            floatPrecision = 3;
          }
          if (subMenuIndex == 1) // Off-trigger yaw
          {          
            mode = MODE_FLOATCHANGE;
            setUpNumChange = true;
            numChangeSelector = 0;     
            inputFloatAdress = &radioDataOut0.oversteer_yaw_release;     
            floatToEdit = *inputFloatAdress;
            numChangeHeader = F("D/s^2: ");
            numChangeHeader.replace('D',(char) B11011111);
            numChangeHeaderSize = 7;
            floatPrecision = 3;
          }
          if (subMenuIndex == 2) // Steering correction
          {          
            mode = MODE_FLOATCHANGE;
            setUpNumChange = true;
            numChangeSelector = 0;     
            inputFloatAdress = &radioDataOut1.oversteer_steering_multi;     
            floatToEdit = *inputFloatAdress;
            numChangeHeader = F("multi: ");
            numChangeHeaderSize = 7;
            floatPrecision = 3;
          }
          if (subMenuIndex == 3) // Throttle correction
          {          
            mode = MODE_NUMBERCHANGE;
            setUpNumChange = true;
            numChangeSelector = 0;    
            inputIntAdress = &radioDataOut1.oversteer_throttle_corr;      
            numberToEdit = *inputIntAdress;
            numChangeHeader = F("corr: ");
            numChangeHeaderSize = 6;
          }
          if (subMenuIndex == 4) // Detection on/off
          {
            flipBool(&switches1, 0);
          }
          if (subMenuIndex == 5) // Correction on/off
          {
            flipBool(&switches1, 1);
          }              
        }
        else if (mainMenuIndex == 1) // Understeer settings
        {          
          if (subMenuIndex == 0) // X-axis trigger acc. 
          {          
            mode = MODE_FLOATCHANGE;
            setUpNumChange = true;
            numChangeSelector = 0;          
            inputFloatAdress = &radioDataOut1.understeerXdiff;     
            floatToEdit = *inputFloatAdress;
            numChangeHeader = F("m/s^2: ");
            numChangeHeaderSize = 7;
            floatPrecision = 4;
          }          
          if (subMenuIndex == 1) // Y-axis trigger acc. 
          {          
            mode = MODE_FLOATCHANGE;
            setUpNumChange = true;
            numChangeSelector = 0;          
            inputFloatAdress = &radioDataOut1.understeerYdiff;     
            floatToEdit = *inputFloatAdress;
            numChangeHeader = F("m/s^2: ");
            numChangeHeaderSize = 7;
            floatPrecision = 4;
          }
          // TODO: REWRITE SD CARD MENU!!!!!!!!          
          if (subMenuIndex == 2) // Understeer throttle corr.
          {          
            mode = MODE_NUMBERCHANGE;
            setUpNumChange = true;
            numChangeSelector = 0;          
            inputIntAdress = &radioDataOut1.understeer_throttle_corr;      
            numberToEdit = *inputIntAdress;
            numChangeHeader = F("corr: ");
            numChangeHeaderSize = 6;
          }
          if (subMenuIndex == 3) // Detection on/off
          {
            flipBool(&switches1, 2);
          }
          if (subMenuIndex == 4) // Correction on/off
          {
            flipBool(&switches1, 3);
          }    
        } 
        else if (mainMenuIndex == 2) // Veering settings
        {
          if (subMenuIndex == 0) // Trigger acceleration
          {          
            mode = MODE_FLOATCHANGE;
            setUpNumChange = true;
            numChangeSelector = 0;          
            inputFloatAdress = &radioDataOut0.straightSensitivity;     
            floatToEdit = *inputFloatAdress;
            numChangeHeader = F("m/s^2: ");
            numChangeHeaderSize = 7;
            floatPrecision = 4;
          }          
          if (subMenuIndex == 1) // Correction factor 
          {          
            mode = MODE_NUMBERCHANGE;
            setUpNumChange = true;
            numChangeSelector = 0;          
            inputIntAdress = &radioDataOut1.straightline_corr;      
            numberToEdit = *inputIntAdress;
            numChangeHeader = F("corr: ");
            numChangeHeaderSize = 6;
          }
          if (subMenuIndex == 2) // Detection on/off
          {
            flipBool(&switches1, 4);
          }
          if (subMenuIndex == 3) // Correction on/off
          {
            flipBool(&switches1, 5);
          }
        }
        else if (mainMenuIndex == 3) // Other ESC settings
        {          
          if (subMenuIndex == 0) // ESC deadzone  
          {          
            // FLOAT specified in degrees. calculate to rad!
            mode = MODE_FLOATCHANGE;
            setUpNumChange = true;
            numChangeSelector = 0;          
            inputFloatAdress = &radioDataOut0.ESC_deadzone;     
            floatToEdit = *inputFloatAdress;
            numChangeHeader = F("deg: ");
            numChangeHeaderSize = 5;
            floatPrecision = 2;
          }          
          if (subMenuIndex == 1) // Steering deadzone
          {          
            mode = MODE_NUMBERCHANGE;
            setUpNumChange = true;
            numChangeSelector = 0;          
            inputIntAdress = &radioDataOut0.steeringDeadzone;      
            numberToEdit = *inputIntAdress;
            numChangeHeader = F("corr: ");
            numChangeHeaderSize = 6;
          }          
          if (subMenuIndex == 2) // Brake deadzone  INT
          {          
            mode = MODE_NUMBERCHANGE;
            setUpNumChange = true;
            numChangeSelector = 0;          
            inputIntAdress = &radioDataOut0.brake_deadzone;      
            numberToEdit = *inputIntAdress;
            numChangeHeader = F("corr: ");
            numChangeHeaderSize = 6;
          }           
          if (subMenuIndex == 3) // ABS on/off 
          {
            flipBool(&switches1, 6);
          }       
          // 4Wheel size          
          // 5Top speed limiter  INT
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
          lcd_buf.clear();
          setKnobTurn(0);
        }        
      } 
      if (mode == MODE_NUMBERCHANGE || mode == MODE_FLOATCHANGE)
      {
        lcd.clear();
        lcd_buf.home();
        lcd_buf.print(lcd_buf.line(1)); 
        lcd_buf.clearLine(1);
      }
    }  
  }
  else if (mode == MODE_NUMBERCHANGE)
  {
    if (setUpNumChange)
    {      
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
      lcd_buf.setCursor(valuePointer,1);
      lcd_buf.print(numChangeHeader);
      if (digits[1] == 1) 
      { 
        lcd_buf.setCursor((valuePointer + numChangeHeaderSize - 1),1); 
        lcd_buf.print("-"); 
      } 
      while (i < DIGITS_LENGTH) { lcd_buf.print(digits[i]); i++; }
      numChangeSelectorMax = digits[0] + 2;
      lcd_buf.setCursor(0,3);
      lcd_buf.print(F("Cancel"));
      lcd_buf.setCursor(14,3);
      lcd_buf.print(F("Accept"));
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
        lcd_buf.setCursor(valuePointer + numChangeHeaderSize + digits[0],1);
        lcd_buf.print(" ");
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
          lcd_buf.setCursor(valuePointer + numChangeHeaderSize - 1,1);
          if (numberToEdit < 0) 
          { 
            lcd_buf.print("-");
          }
          else
          {
            lcd_buf.print(" ");
          }
          while (i < DIGITS_LENGTH) { lcd_buf.print(digits[i]); i++; }
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
          lcd_buf.setCursor(arrowPointer,1);
          lcd_buf.print(digits[editIndex]);
        }        
      }
      /// clear some stuff:
      lcd_buf.setCursor(0,2);
      lcd_buf.print("                    ");
      lcd_buf.setCursor(6,3);
      lcd_buf.print(" ");
      lcd_buf.setCursor(13,3);
      lcd_buf.print(" ");
    }
    if (true)
    {           
      if (numChangeSelector == 0)
      {
        lcd_buf.setCursor(6,3);
        lcd_buf.print("<");
      }
      if (numChangeSelector == numChangeSelectorMax - 1)
      {
        lcd_buf.setCursor(valuePointer + numChangeHeaderSize + digits[0],1);
        if (numChange)
        {
          lcd_buf.write(2);
        }
        else
        {
          lcd_buf.write(60);
        }  
      }
      if (numChangeSelector == numChangeSelectorMax)
      {
        lcd_buf.setCursor(13,3);
        lcd_buf.print(">");
      }
      if (numChangeSelector > 0 && numChangeSelector < numChangeSelectorMax - 1)
      {
        arrowPointer = valuePointer + numChangeHeaderSize + (numChangeSelector - 1);
        lcd_buf.setCursor(arrowPointer, 2);
        if (numChange)
        {
          lcd_buf.write(4);
        }
        else
        {
          lcd_buf.write(3);
        }        
      }
    }
    if (encoderBtnClicked)
    {    
      encoderBtnClicked = false;
      if (numChangeSelector == 0)
      {
        lcd_buf.setCursor(2,2);
        lcd_buf.print(F("Cancelled..."));
        lcd.setCursor(0,2);
        lcd.print(lcd_buf.line(2));
        delay(500);
      }
      else if (numChangeSelector == numChangeSelectorMax)
      {
        *inputIntAdress = numberToEdit;
        lcd_buf.setCursor(2,2);
        lcd_buf.print(F("Accepted..."));
        lcd.setCursor(0,2);
        lcd.print(lcd_buf.line(2));
        delay(500);
      }
      if (numChangeSelector == 0 || numChangeSelector == numChangeSelectorMax)
      {
        lcd.clear();
        lcd_buf.clear();
        setKnobTurn(0);
        
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
      lcd_buf.home();
      lcd_buf.print(lcd_buf.line(1));
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
      lcd_buf.setCursor(valuePointer,1);
      lcd_buf.print(numChangeHeader);
      if (digits[1] == 1) 
      { 
        lcd_buf.setCursor((valuePointer + numChangeHeaderSize - 1),1); 
        lcd_buf.print("-"); 
      }       
      byte dot = i + floatPower;
      while (i < DIGITS_LENGTH) 
      { 
        lcd_buf.print(digits[i]); 
        if (i == dot) lcd_buf.print(".");
        i++; 
      }
      numChangeSelectorMax = digits[0] + 3;
      lcd_buf.setCursor(0,3);
      lcd_buf.print(F("Cancel"));
      lcd_buf.setCursor(14,3);
      lcd_buf.print(F("Accept"));
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
        lcd_buf.setCursor(valuePointer + numChangeHeaderSize + digits[0] + 1,1);
        lcd_buf.print(" ");
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
          lcd_buf.setCursor(valuePointer + numChangeHeaderSize - 1,1);
          if (floatToEdit < 0) 
          { 
            lcd_buf.print("-");
          }
          else
          {
            lcd_buf.print(" ");
          }
          byte dot = i + floatPower;
          while (i < DIGITS_LENGTH) 
          { 
            lcd_buf.print(digits[i]); 
            if (i == dot) lcd_buf.print(".");
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
          lcd_buf.setCursor(arrowPointer,1);
          lcd_buf.print(digits[editIndex]);
        }        
      }
      /// clear some stuff: REDO this more efficiently!!
      lcd_buf.setCursor(0,2);
      lcd_buf.print("                    ");
      lcd_buf.setCursor(6,3);
      lcd_buf.print(" ");
      lcd_buf.setCursor(13,3);
      lcd_buf.print(" ");
    }
    if (true)
    {      
      if (numChangeSelector == 0)
      {
        lcd_buf.setCursor(6,3);
        lcd_buf.print("<");
      }
      if (numChangeSelector == numChangeSelectorMax - 1)
      {
        lcd_buf.setCursor(valuePointer + numChangeHeaderSize + digits[0] + 1,1);
        if (numChange)
        {
          lcd_buf.write(2);
        }
        else
        {
          lcd_buf.write(60);
        }  
      }
      if (numChangeSelector == numChangeSelectorMax)
      {
        lcd_buf.setCursor(13,3);
        lcd_buf.print(">");
      }
      if (numChangeSelector > 0 && numChangeSelector < numChangeSelectorMax - 1)
      {
        arrowPointer = valuePointer + numChangeHeaderSize + (numChangeSelector - 1);
        lcd_buf.setCursor(arrowPointer, 2);
        if (numChange)
        {
          lcd_buf.write(4);
        }
        else
        {
          lcd_buf.write(3);
        }        
      }      
    }
    if (encoderBtnClicked)
    {    
      encoderBtnClicked = false;
      if (numChangeSelector == 0)
      {
        lcd_buf.setCursor(2,2);
        lcd_buf.print(F("Cancelled..."));
        lcd.setCursor(0,2);
        lcd.print(lcd_buf.line(2));
        delay(500);
      }
      else if (numChangeSelector == numChangeSelectorMax)
      {
        *inputIntAdress = numberToEdit;
        lcd_buf.setCursor(2,2);
        lcd_buf.print(F("Accepted..."));
        lcd.setCursor(0,2);
        lcd.print(lcd_buf.line(2));
        delay(500);
      }
      if (numChangeSelector == 0 || numChangeSelector == numChangeSelectorMax)
      {
        lcd.clear();
        lcd_buf.clear();
        setKnobTurn(0);
        
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
  // Refresh screen
  if (commonMillis > (lastLCDMillis + 100))
  {
    lastLCDMillis = commonMillis;
    lcd.home();
    lcd.print(lcd_buf.line(0));
    lcd.setCursor(0,1);
    lcd.print(lcd_buf.line(1));
    lcd.setCursor(0,2);
    lcd.print(lcd_buf.line(2));
    lcd.setCursor(0,3);
    lcd.print(lcd_buf.line(3));
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
  if (sendDataToCar && (lastRadioOutMillis + 100 < commonMillis))
  {
    if (radioStatus == R_STAT_READY)
    {
      // settings:
      radio.stopListening();    
      radio.write( &radioDataOut0, sizeof(radioDataOut0));          
      radio.startListening();
    }
    else if (radioStatus == R_STAT_ACK_0)
    {
      radioDataOut1.engine_throttle_corr = 0;
      radio.stopListening();    
      radio.write( &radioDataOut1, sizeof(radioDataOut1));          
      radio.startListening();
    }
    else if (radioStatus == R_STAT_ACK_1)
    {
      writeBool(&switches0, 1, false);
      sendDataToCar = true;
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

void headlightSwitch() 
{
  btn1 = true;
}

void neonSwitch() 
{
  btn2 = true;
}

void error_P(const char* str) {
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

void readSDSettings()
{
   char character;
   String settingName;
   String settingValue;   
   if (file.open("settings.TXT", O_READ)) 
   {
     while ((character = file.read()) > 0)
     {     
       while ((character = file.read()) > 0)  
       {
          if (character == '[') break;
       }
       while ((character = file.read()) > 0)
       {
         if (character == '=') break;
         settingName = settingName + character;
       }
       while ((character = file.read()) > 0)
       {
         if (character == ']') break;
         settingValue = settingValue + character;
       }
       if(character == ']')
       {       
         /* // Debugging Printing
         Serial.print("Name:");
         Serial.println(settingName);
         Serial.print("Value :");
         Serial.println(settingValue); */
         
         // Apply the value to the parameter
         applySetting(settingName,settingValue);
         // Reset Strings
         settingName = "";
         settingValue = "";
       }
     }
     // close the file:
     file.close();
   } 
   else 
   {
     // if the file didn't open, print an error:
   }
}
 
 /* Apply the value to the parameter by searching for the parameter name
 Using String.toInt(); for Integers
 toFloat(string); for Float
 toBoolean(string); for Boolean
 toLong(string); for Long
 */
void applySetting(String settingName, String settingValue) 
{
  if(settingName == "A") { radioDataOut0.steeringDeadzone = settingValue.toInt(); }
  if(settingName == "B") { radioDataOut0.straightSensitivity = toFloat(settingValue);}
  if(settingName == "C") { radioDataOut0.ESC_deadzone = toFloat(settingValue); }
  if(settingName == "D") { radioDataOut0.brake_deadzone = settingValue.toInt(); }
  if(settingName == "E") { radioDataOut0.oversteer_yaw_difference = toFloat(settingValue);  }
  if(settingName == "F") { radioDataOut0.oversteer_yaw_release = toFloat(settingValue);  }
  if(settingName == "G") { radioDataOut1.understeerXdiff = toFloat(settingValue);  }
  if(settingName == "H") { radioDataOut1.understeerYdiff = toFloat(settingValue);  }
  if(settingName == "I") { radioDataOut1.oversteer_steering_multi = toFloat(settingValue);  }
  if(settingName == "J") { radioDataOut1.oversteer_throttle_corr = settingValue.toInt();  }
  if(settingName == "K") { radioDataOut1.understeer_throttle_corr = settingValue.toInt();  }
  if(settingName == "L") { radioDataOut1.straightline_corr = settingValue.toInt();  }
  if(settingName == "M") { radioDataOut1.wheelCircumference =  settingValue.toInt(); }
}
 
 // converting string to Float
float toFloat(String settingValue)
{
  char floatbuf[settingValue.length()+1];
  settingValue.toCharArray(floatbuf, sizeof(floatbuf));
  float f = atof(floatbuf);
  return f;
}

long toLong(String settingValue)
{
  char longbuf[settingValue.length()+1];
  settingValue.toCharArray(longbuf, sizeof(longbuf));
  long l = atol(longbuf);
  return l;
}

// Converting String to integer and then to boolean
// 1 = true
// 0 = false
boolean toBoolean(String settingValue) 
{
  if(settingValue.toInt()==1) { return true; } 
  else { return false; }
}
 
 // Writes A Configuration file
void writeSDSettings() {
  // Delete the old One
  //file.remove("settings.TXT");
  // Create new one
  Serial.println(file.open("settings.TXT", O_WRITE));
  // writing in the file works just like regular print()/println() function
  file.print('#');
  String settingValue;
  for(char settingName = 'A'; settingName < 'N'; settingName++)
  {
    file.print('[');
    file.print(settingName);
    file.print('=');
    if(settingName == 'A') { settingValue = radioDataOut0.steeringDeadzone; }
    if(settingName == 'B') { settingValue = radioDataOut0.straightSensitivity;}
    if(settingName == 'C') { settingValue = radioDataOut0.ESC_deadzone; }
    if(settingName == 'D') { settingValue = radioDataOut0.brake_deadzone; }
    if(settingName == 'E') { settingValue = radioDataOut0.oversteer_yaw_difference;  }
    if(settingName == 'F') { settingValue = radioDataOut0.oversteer_yaw_release;  }
    if(settingName == 'G') { settingValue = radioDataOut1.understeerXdiff;  }
    if(settingName == 'H') { settingValue = radioDataOut1.understeerYdiff;  }
    if(settingName == 'I') { settingValue = radioDataOut1.oversteer_steering_multi;  }
    if(settingName == 'J') { settingValue = radioDataOut1.oversteer_throttle_corr;  }
    if(settingName == 'K') { settingValue = radioDataOut1.understeer_throttle_corr;  }
    if(settingName == 'L') { settingValue = radioDataOut1.straightline_corr;  }
    if(settingName == 'M') { settingValue = radioDataOut1.wheelCircumference; }
    file.println(']');
  }
  // close the file:
  file.close();
  Serial.println("Writing done.");
}

void useKnobTurn(int8_t * valueToChange, int8_t turns, byte mini, byte maxi)
{
  int8_t increment = turns / abs(turns);
  while (abs(turns) > 0)
  {
    *valueToChange += increment;
    if (*valueToChange > maxi)
    {
      *valueToChange = mini;
    }
    else if (*valueToChange < mini)
    {
      *valueToChange = maxi;
    }
    turns -= increment; // this moves the value one step closer to 0 regardless of its sign
  }
}

void readSDmenu(char* fileName, int8_t menuIndex, byte targetLine)
{
  file.open(fileName, O_READ);
  uint8_t index = 255;
  uint8_t currentByte;
  while ((currentByte = file.read()) > 0)
  {      
    if (currentByte == 35) // 35 is #
    {
    index = (uint8_t) file.read();
    index -= 48; // 48 is zero
    }
    if (index == menuIndex)
    {
    lcd_buf.setCursor(0,targetLine);
    for (byte i = 0; i < 20; i++)
    {
      lcd_buf.write(file.read());
    }
    break;
    /// if (file.read() != $) { read another line }
    }
  }
  file.close();
}

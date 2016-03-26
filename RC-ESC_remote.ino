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

#include <I2Cdev.h>
#include <EasyTransferI2C.h>
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include <Wire.h>
#endif

#include <EnableInterrupt.h>
#include <LiquidCrystal.h>

const int ST_BALANCED = 0;
const int ST_UNDERSTEER = 1;
const int ST_OVERSTEER = 2;
const int ST_SLIP = 3;
const int ST_SPINNING = 4;
const int ST_ROLLED = 5;

int pulses = 0;
int A_SIG=0, B_SIG=1;

float engineTemp;
uint32_t lastLCDMillis;

LiquidCrystal lcd(2, 3, 4, 5, 6, 7);

//// RADIO SETTINGS
bool radioNumber = 1;
RF24 radio(8,9);
byte addresses[][6] = {"1Node","2Node"};
bool role = 0;

struct RADIO_DATA {
  uint8_t state;
  uint8_t gMode;
  float dTrainSpeed;
};

RADIO_DATA radioData;

/// endof radio settings

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

int menuLevel = 0;
int subMenuOpen = 0;
int8_t mainMenuIndex = 0;
const byte mainMenuLength = 6;
String mainMenu[] = {
  "Sensitivity settings",
  "Current readings    ",
  "Idle screen content ",
  "Performance Tests   ",
  "Remote settings     ",
  "Exit menu           "
};

String* subMenuTree[2];

String subMenu0[] = {
  "Oversteer sens.     ",
  "Understeer sens.    ",
  "ABS sensitivity     ",
  "Straightline holding",
  "Exit menu           "
};

String subMenu1[] = {
  "Sensitivity settings",
  "Current readings    ",
  "Idle screen content ",
  "Performance Tests   ",
  "Remote settings     ",
  "Exit menu           "
};

bool encoderBtnPressed = false;


void setup() 
{  
  enableInterrupt(A0, A_RISE, RISING);
  enableInterrupt(A1, B_RISE, RISING);
  enableInterrupt(A2, BTN_PRESS, FALLING);

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
  
  lcd.createChar(1, newChar1);
  // set up the LCD's number of columns and rows:
  lcd.begin(20, 4);
  // Print a message to the LCD.

  subMenuTree[0] = subMenu0;
  subMenuTree[1] = subMenu1;
}

void loop() 
{    
  uint32_t commonMillis = millis();
    
  if(radio.available())
  {                                                             
    while (radio.available()) 
    {                                  
      radio.read( &radioData, sizeof(radioData));
    }  
    lcd.setCursor(0, 3);
    lcd.print("receiving");
  }
  else
  {
    
  }
  if (commonMillis > (lastLCDMillis + 100))
  {
    lastLCDMillis = commonMillis;
    if (menuLevel == 0)
    {
      lcd.setCursor(0, 0);
      lcd.print(mainMenu[mainMenuIndex]);
    }
    if (menuLevel == 1)
    {
      lcd.setCursor(0, 1);
      lcd.print(subMenuTree[subMenuOpen][mainMenuIndex]);
    }
    String stateString;
    switch(radioData.state)
    {
      case ST_BALANCED:
        stateString = "Balanced  ";
        break;
      case ST_UNDERSTEER:
        stateString = "Understeer";
        break;
      case ST_OVERSTEER:
        stateString = "Oversteer ";
        break;
      case ST_ROLLED:
        stateString = "Rolled    ";
        break;  
      default:
        stateString = "N/A       ";
        break;
        
    }
    lcd.setCursor(0,1);
    lcd.print(stateString);
    lcd.setCursor(0,2);
    lcd.print(radioData.dTrainSpeed,3);
    lcd.print(" m/s");
  }/*
  if (encoderBtnPressed)
  {
    encoderBtnPressed = false;
    subMenuOpen = mainMenuIndex;
    lcd.setCursor(0, 1);
    lcd.print(subMenuTree[mainMenuIndex][0]);
    menuLevel++;
  }*/
}

void A_RISE(){
 disableInterrupt(A0);
 A_SIG=1;
 
 if(B_SIG==0)
 pulses++;//moving forward
 if(B_SIG==1)
 pulses--;//moving reverse
 calcMenuIndex();
 enableInterrupt(A0, A_FALL, FALLING);
}

void A_FALL(){
  disableInterrupt(A0);
 A_SIG=0;
 
 if(B_SIG==1)
 pulses++;//moving forward
 if(B_SIG==0)
 pulses--;//moving reverse
 calcMenuIndex();
 enableInterrupt(A0, A_RISE, RISING);  
}

void B_RISE(){
 disableInterrupt(A1);
 B_SIG=1;
 
 if(A_SIG==1)
 pulses++;//moving forward
 if(A_SIG==0)
 pulses--;//moving reverse
 calcMenuIndex();
 enableInterrupt(A1, B_FALL, FALLING);
}

void B_FALL(){
 disableInterrupt(A1);
 B_SIG=0;
 
 if(A_SIG==0)
 pulses++;//moving forward
 if(A_SIG==1)
 pulses--;//moving reverse
 calcMenuIndex();
 enableInterrupt(A1, B_RISE, RISING);
}

void calcMenuIndex()
{
  if (pulses >= 4)
  {
    mainMenuIndex++;
    pulses = 0;
  }
  else if (pulses <= -4)
  {
    mainMenuIndex--;
    pulses = 0;
  } 
  
  if (mainMenuIndex >= mainMenuLength)
  {
    mainMenuIndex = 0;
  }
  else if (mainMenuIndex < 0)
  {
    mainMenuIndex = mainMenuLength - 1;
  }
}

void BTN_PRESS()
{
  encoderBtnPressed = true;
}


/////////////////////////////////////////////////////////////////////////////////////
///           Electronic Stability Control for RC Car v1.0 2016/03/26             ///
/////////////////////////////////////////////////////////////////////////////////////
/// IMPORTANT NOTICE: THIS PROJECT IS FAR FROM READY, SO USE IT AT YOUR OWN RISK!
/// This project features an electronic stability control for a 1/10th scale RC car.
/// The targetted car is a nitro-powered RC-car, but any 1/10th scale RC car is
/// compatible with the project that has a throttle and a steering servo.
/// The setup consists of two Arduino Nano-compatible boards on the car, an MPU-6050 
/// breakout board (GY-521), a DPDT-relay as a safety switch, a QRD1114 infrared 
/// LED and phototransistor, as speed sensor, and an RGB-LED for feedback.
/// For remotely changing settings, the in-car setup is equipped with an NRF24L01 
/// radio. The remote control is a third arduino nano, connected to an LCD screen,
/// an encoder and a similar NRF24L01 radio, providing two-way communication with the
/// car. The rest of the components and features are optional. These include an LM338-
/// based glow plug driver which is yet to be tested. This will rely on a thermistor
/// or an LM335 IC and a crankshaft RPM sensor.
/// As of 2016/03/07 I am waiting for some parts to test the whole circuit, the ESC is 
/// hooked up to the car, with the gyro and I am currently working on the transmission
/// of sensitivity settings along the arduinos, and constantly cleaning up the code.
/// The circuit is more or less designed, I want to test everything before printing
/// the circuit on a PCB
/////////////////////////////////////////////////////////////////////////////////////
/// This part contains description of the finished and yet-to-be finished features:
/// OPERATION: Currently the stability control is achieved only by throttle and 
/// steering servo control. 1/5th scale cars can include indvidual braking on each 
/// wheel, which can take stability control to a whole new level.
/// INTERVENTION: Oversteer and understeer corrections, holding straight,
/// ABS (anti-lock braking system), glow plug heating (startup and while running), 
/// startup help 
/////////////////////////////////////////////////////////////////////////////////////
/// Some details, so one can decipher my not-too-professional code :)
/// Inline comments are not always trustworthy :)
/// DECISION: From the gyroscope/accelerometer and RPM sensor (not yet) input, 
/// and the geometry of the car, the program calculates the intended course of the car
/// (a speed vector).
/// It stores this vector as X and Y vectors respective to the car and a timestamp
/// for it. In the next loop this is done again, and from the two vectors it calculates
/// an acceleration vector. The program also calculates the centripetal acceleration 
/// for the curve that the car is negotiating. This is done for both the front and 
/// rear axles. The vectors' sum is what the car "experiences" as acceleration at
/// its centerpoint. The difference of the sideways component of the front and rear
/// vector is what the car "experiences" as rotation around the vertical axis at its
/// centerpoint (called as yaw).
/// These are the optimal values, at optimal conditions, no slip, so some tolerance 
/// is added to tune the sensitivity (this is done by remote control). 
/// The actual values are measured by the gyroscope/accelerometer. As for the 
/// understeer detection, if the acceleration vector points more towards the front of 
/// the car than calculated, or its sideways absolute value is less than calculated, 
/// means that the car is negotiating a slighter curve (a bigger radius) than intended
/// (i.e. it is understeering). As for the oversteer detection, if there is more yaw, 
/// (i.e the car rotates around its center more) than expected, it means, that the rear
/// end of the car has slipped out from the curve (fishtailed). Also in extremely rare
/// cases similar slip can occur at the front, but the response is essentially the
/// same for both cases. 
/// OVERSTEER-CORRECTION: if the program finds that the car rotates around its vertical
/// axis more than it is supposed to be, that means the car is fishtailing. It stores 
/// the last steering input before the car started to skid, and adjusts the steering
/// so that the front wheels try to face the original course of the car (in one word:
/// countersteering) and reducing throttle to regain tyre grip. This behaviour can be 
/// fine-tuned at sensitivity, so it can let some sliding, and at intervention, i.e. 
/// how agressive the countersteering is.
/// UNDERSTEER-CORRECTION: as there are no seperate brakes for every wheel, the only 
/// intervention that is possible is reducing throttle, so the tyres get more grip.
/// However, one condition has to be added, braking is not allowed, as it will reduce
/// grip, and worsen the condition. No braking vs. ABS braking in this situation is 
/// yet to be tested. This feature is also fine tuneable, and needs testing for every
/// car so as not to reduce power too much. 
/// HOLDING-STRAIGHT: if the controller's steering wheel is centered, we want the car
/// to go straight, so if there is sideways acceleration in this situation the car
/// is negotiating a curve (to my experience this happens with entry-level RC cars).
/// To counteract this, the program uses very slight (of course) fine-tuneable
/// countersteering.
/// STARTUP AND GLOWPLUG HEATING: This is the feature that has to be developed and 
/// tested the most. At startup (as there is no choke valve here) a small throttle
/// is added, until the point that the wheels are not turning, and the thermistor
/// measures engine temp. As optimal temp is reached extra throttle is reduced, and 
/// glow is switched off. If during run engine cools down, if absolutely necessary
/// the glow plug is switched on again, hopefully not draining too much battery.
/// SAFETY AND OTHER: Battery level is monitored by the other arduino, and the other
/// arduino also acts as a failsafe. If the first arduino (with this program on)
/// freezes for some reason, the second arduino pulls a DPDT relay low, bypassing
/// remote control directly to the servos. It also resets the other arduino, hoping
/// that the problem can be resolved. 
////////////////////////////////////////////////////////////////////////////////////
/// Any feedback is appreciated, I am planning on posting circuit and other info
/// later on, I am not yet sure where. In the meantime enjoy this code.
/// Be gentle, as this is my first such project, and I am new to github.
/// Questions and feedback email: gmarci90+ESC@gmail.com 
////////////////////////////////////////////////////////////////////////////////////
/// VERY SPECIAL THANKS TO DuaneB for the Basics (interfacing with the receiver and 
/// servos, and reading/writing settings to and from EEPROM) were 
/// adapted from RCArduinoYawControlby DuaneB at rcarduino.blogspot.com.
/// RCArduinoYawControl by DuaneB is licensed under a 
/// Creative Commons Attribution-NonCommercial-ShareAlike 3.0 Unported License.
/// VERY SPECIAL THANKS TO Jeff Rowberg for the I2Cdevlib and the MPU6050 libraries
/// ALSO THANKS TO madsci1016 for the EasyTransfer library so I don't have to fiddle
/// with bits to send data over I2C
/////////////////////////////////////////////////////////////////////////////////////
/// This project is licensed under a 
/// Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
/// https://creativecommons.org/licenses/by-nc-sa/4.0/
/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////

#include <EnableInterrupt.h>
#include <Servo.h>
#include <EEPROM.h>

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include <I2Cdev.h>
#include <EasyTransferI2C.h>

#include <MPU6050_6Axis_MotionApps20.h>
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include <Wire.h>
#endif

/// If we want serial output set this to true (setting to false saves cca. 2 KB)
#define DEBUG false
#if DEBUG == true
  #define DEBUG_PRINTLN(x) Serial.println(x)
  #define DEBUG_PRINT(x) Serial.print(x)
#else
  #define DEBUG_PRINTLN(x) 
  #define DEBUG_PRINT(x)
#endif

const int RC_NEUTRAL = 1500;
const int RC_MAX = 2000;
const int RC_MIN = 1000;
const int RC_DIFF = 500; /// = (RC_MAX - RC_MIN)/2

///////////////////////////////////
/// CAR GEOMETRY
///////////////////////////////////
///
///      ^ y-axis
///      |
///  []--|--[] FR-wheel
///      |
///   ---+----> x-axis
///      |
///  []--|--[]
///
//////////////////////////////////
/// Car front wheel maximum turn angles (in radians)
const float MAX_IN_ANGLE = 0.523598776;
const float MAX_OUT_ANGLE = 0.34906585;
const float WHEELBASE = 0.26;

uint16_t unSteeringMin = RC_MIN;
uint16_t unSteeringMax = RC_MAX;
uint16_t unSteeringCenter = RC_NEUTRAL;

uint16_t unThrottleMin = RC_MIN;
uint16_t unThrottleMax = RC_MAX;
uint16_t unThrottleCenter = RC_NEUTRAL;

//////////////////////////////////////////////////////////////////
// PIN ASSIGNMENTS
//////////////////////////////////////////////////////////////////
// ANALOG PINS
//////////////////////////////////////////////////////////////////
const int THROTTLE_SENSITIVITY_PIN = 0;
const int STEERING_SENSITIVITY_PIN = 1;
const int THROTTLE_DECAY_PIN = 2;
const int STEERING_DECAY_PIN = 3;
#define SPEED_SENSOR_PIN 3
//////////////////////////////////////////////////////////////////
// DIGITAL PINS
//////////////////////////////////////////////////////////////////
const int PROGRAM_PIN = 9;
const int INFORMATION_INDICATOR_PIN = 12; /// 13 on my PCB
const int ERROR_INDICATOR_PIN = 6;
const int THROTTLE_IN_PIN = 2;
const int STEERING_IN_PIN = 3;
const int THROTTLE_OUT_PIN = 8;
const int STEERING_OUT_PIN = 7;
const int BRAKELIGHT_PIN = 4;

////////////////////////////////////////////////////////////////////
/// GYRO PINS and GYRO VARIABLES
//////////////////////////////////////////////////////////////////
MPU6050 mpu;

const int GYRO_LED_PIN = 10;
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

/// ENDGYRO

// These bit flags are set in bUpdateFlagsShared to indicate which
// channels have new signals
#define THROTTLE_FLAG 1
#define STEERING_FLAG 2

/// InterruptIndicators
// holds the update flags defined above
volatile uint8_t bUpdateFlagsShared;
// indicates whether MPU interrupt pin has gone high:
bool mpuInterrupt = false;

// shared variables are updated by the ISR and read by loop.
// In loop we immediatley take local copies so that the ISR can keep ownership of the
// shared ones. To access these in loop
// we first turn interrupts off with noInterrupts
// we take a copy to use in loop and the turn interrupts back on
// as quickly as possible, this ensures that we are always able to receive new signals
volatile uint16_t unThrottleInShared;
volatile uint16_t unSteeringInShared;

// These are used to record the rising edge of a pulse in the calcInput functions
// They do not need to be volatile as they are only used in the ISR. If we wanted
// to refer to these in loop and the ISR then they would need to be declared volatile
uint32_t ulThrottleStart;
uint32_t ulSteeringStart;

// time variables;
uint32_t ulLastThrottleIn;
uint32_t unRollTime;

/// SPEED SENSING
bool readFlag = true;

uint32_t whiteStart = 0;
volatile uint32_t interval;
uint32_t intervalLoop;
bool updateFlag = false;

uint32_t lastSpeedUpdateMillis = 0;

bool brake_blocked = false;

#define MODE_FORCEPROGRAM 0
#define MODE_RUN 1
#define MODE_QUICK_PROGRAM 2
#define MODE_FULL_PROGRAM 3
#define MODE_ERROR 4

uint8_t gMode = MODE_RUN;
uint32_t ulProgramModeExitTime = 0;

// Index into the EEPROM Storage assuming a 0 based array of uint16_t
// Data to be stored low byte, high byte
#define EEPROM_INDEX_STEERING_MIN 0
#define EEPROM_INDEX_STEERING_MAX 1
#define EEPROM_INDEX_STEERING_CENTER 2
#define EEPROM_INDEX_THROTTLE_MIN 3
#define EEPROM_INDEX_THROTTLE_MAX 4
#define EEPROM_INDEX_THROTTLE_CENTER 5

Servo servoThrottle;
Servo servoSteering;

EasyTransferI2C easyTransferIn; /// for setting values of ESC
EasyTransferI2C easyTransferOut; /// for sending telemetry data to other arduino

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
uint8_t ESC_deadzone = 0.0174532925; /* 2-3 degrees */
int brake_deadzone = 100;
/// Decision related sensitivity:
float oversteer_yaw_difference = 10;
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
byte engine_throttle_corr;
/// telemetry data used by loop, but declared outside loop so ET can access it
volatile float dTrainSpeed = 0;
uint16_t engineRPM; 
/// end of telemetry data

struct RECEIVE_SETTINGS_STRUCTURE{ /// for setting values of ESC
  // variables have the same meaning as above
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

struct SEND_DATA_STRUCTURE{ /// for sending telemetry data to other arduino
  uint8_t state;
  uint8_t gMode;
  float dTrainSpeed;
};

RECEIVE_SETTINGS_STRUCTURE rxdata;
SEND_DATA_STRUCTURE txdata;

/// States of the car, that have to be adressed
const int ST_BALANCED = 0;
const int ST_UNDERSTEER = 1;
const int ST_OVERSTEER = 2;
const int ST_SLIP = 3;
const int ST_SPINNING = 4;
const int ST_ROLLED = 5;

const int TURN_NONE = 0;
const int TURN_LEFT = 1;
const int TURN_RIGHT = 2;

uint8_t state = ST_BALANCED;

uint32_t lastWireSend = 0;

void setup()
{
  if (DEBUG) { Serial.begin(115200); }
  Serial.begin(9600);

  pinMode(PROGRAM_PIN, INPUT);
  pinMode(INFORMATION_INDICATOR_PIN, OUTPUT);
  pinMode(ERROR_INDICATOR_PIN, OUTPUT);
  pinMode(BRAKELIGHT_PIN, OUTPUT);

  // if pin 2 and 3 (int pin 0 and 1) change calThrottle/Steering is run
  enableInterrupt(2 /* INT0 = THROTTLE_IN_PIN */, calcThrottle, CHANGE);
  enableInterrupt(3 /* INT1 = STEERING_IN_PIN */, calcSteering, CHANGE);
  enableInterrupt(A3, calcSpeed, CHANGE);

  // reads potentiometers to get ESC settings (later will be input by wireless)
  readSettings();

  servoThrottle.attach(THROTTLE_OUT_PIN);
  servoSteering.attach(STEERING_OUT_PIN);

  // if there are no settings in EEPROM force programming
  if (false == readSettingsFromEEPROM())
  {
    gMode = MODE_FORCEPROGRAM;
  } 
  
  Wire.onReceive(receiveEvent);
  //////////////////////////////////////////////////////////
  //// GYRO SETUP
  //////////////////////////////////////////////////////////
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin(0x8);
    TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif  
  
  if (DEBUG) {while (!Serial);} // wait for Leonardo enumeration, others continue immediately

  // initialize device
  DEBUG_PRINTLN(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  DEBUG_PRINTLN(F("Testing device connections..."));
  DEBUG_PRINTLN(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  DEBUG_PRINTLN(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(43);
  mpu.setYGyroOffset(9);
  mpu.setZGyroOffset(26);
  mpu.setXAccelOffset(-606);
  mpu.setYAccelOffset(-911);
  mpu.setZAccelOffset(1670);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    DEBUG_PRINTLN(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    DEBUG_PRINTLN(F("Enabling interrupt detection (Arduino enableInterrupt on DigitalPin 5)..."));
    enableInterrupt(11, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    DEBUG_PRINTLN(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    DEBUG_PRINT(F("DMP Initialization failed (code "));
    DEBUG_PRINT(devStatus);
    DEBUG_PRINTLN(F(")"));
  }

  // configure LED for output
  pinMode(GYRO_LED_PIN, OUTPUT);
  //// GYRO SETUP END 

  easyTransferIn.begin(details(rxdata), &Wire);
  easyTransferOut.begin(details(txdata), &Wire);
  
  // time of the last throttle input to calculate failsafe
  ulLastThrottleIn = millis();
}

void loop()
{
  // create local variables to hold a local copies of the channel inputs
  // these are declared static so that thier values will be retained
  // between calls to loop.
  static uint16_t unThrottleIn;
  static uint16_t unSteeringIn;
  /// we need different output, so we can always calculate with the drivers intention
  static uint16_t unThrottleOut;
  static uint16_t unSteeringOut;
  static uint16_t unLastSteeringOut;
  // local copy of update flags
  static uint8_t bUpdateFlags;

  /// Measured and calculated phyisical variables:
  /// Wheel Angles:
  static float wAngleFL;
  static float wAngleFR;
  static uint8_t turn;
  /// v1.0 Only drivetrain speed, no wheelspeed (1 brake for the 4 wheels, )
  /// Intended course of the car NOTE: int prefix relates to intended not integer
  static float dSteering = 0; /// difference from center in micros
  static float intSpeedX;
  static float intSpeedY;
  static float intSpeedFAx;
  static float intSpeedFAy;
  static float intSpeedRAx;
  static float intSpeedRAy;
  static float normYaw;
  /// Last values:
  static float intLastSpeedX;
  static float intLastSpeedY;
  static float intLastSpeedFAx;
  static float intLastSpeedFAy;
  static float intLastSpeedRAx;
  static float intLastSpeedRAy;
  static uint16_t unLastBalancedSt;
  /// Acceleration calculated from last and current value:
  static float intAccelerationX;
  static float intAccelerationY;
  static float intAccFAx;
  static float intAccFAy;
  static float intAccRAx;
  static float intAccRAy;
  static float normAccelerationX;
  static float normAccelerationY;
  /// timestamp for calculated speed vectors
  static uint32_t ulSpeedMicros;
  static uint32_t ulLastSpeedMicros;
  /// Gyro input:
  static float yawAngle;
  static float yawV;
  static float yawA;
  static float lastYawAngle;
  static float lastYawV;
  static float lastYawA;
  static uint32_t ulYawAngleMillis;
  static uint32_t ulYawVMillis;
  static uint32_t ulLastYawAngleMillis;
  static uint32_t ulLastYawVMillis;
  static float accelerationZgrav; // z-axis acc. with gravity (to detect roll)
  static float accelerationZ;
  static float accelerationY;
  static float accelerationX;
  uint32_t ulMillis = millis();
  static int ABS_corr;
  static uint32_t lastSpeedInterval;
  static uint16_t blockedThrottleOut;
  
  // check shared update flags to see if any channels have a new signal
  if (bUpdateFlagsShared)
  {
    noInterrupts(); // turn interrupts off quickly while we take local copies of the shared variables

    // take a local copy of which channels were updated in case we need to use this in the rest of loop
    bUpdateFlags = bUpdateFlagsShared;

    // in the current code, the shared values are always populated
    // so we could copy them without testing the flags
    // however in the future this could change, so lets
    // only copy when the flags tell us we can.

    if (bUpdateFlags & THROTTLE_FLAG)
    {
      unThrottleIn = unThrottleInShared;
    }

    if (bUpdateFlags & STEERING_FLAG)
    {
      unSteeringIn = unSteeringInShared;
    }

    if (state == ST_BALANCED)
    {
      unSteeringOut = unSteeringIn;
      if (!brake_blocked) { unThrottleOut = unThrottleIn; }
    }

    /// TODO: rename these to more recognizable
    intervalLoop = interval;

    // clear shared copy of updated flags as we have already taken the updates
    // we still have a local copy if we need to use it in bUpdateFlags
    bUpdateFlagsShared = 0;

    interrupts(); // we have local copies of the inputs, so now we can turn interrupts back on
    // as soon as interrupts are back on, we can no longer use the shared copies, the interrupt
    // service routines own these and could update them at any time. During the update, the
    // shared copies may contain junk. Luckily we have our local copies to work with :-)

  }  

  ////////////////////////////////////////////////////////////////////////
  //// READING GYRO VALUES
  ////////////////////////////////////////////////////////////////////////ű
  // if programming failed, don't try to do anything (dmpReady)
  // only read if there was an MPU interrupt or extra packet(s) available
  if (dmpReady && mpuInterrupt)
  {
    Wire.beginTransmission(0x68);
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) 
    {
      // reset so we can continue cleanly
      mpu.resetFIFO();
      DEBUG_PRINTLN(F("FIFO overflow!"));

      // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } 
    else if (mpuIntStatus & 0x02) 
    {
      // wait for correct available data length, should be a VERY short wait
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

      // read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);

      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;

      
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetAccel(&aa, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      yawAngle = ypr[0];
      ulYawAngleMillis = millis();
      /// Get real acceleration, adjusted to remove gravity
      mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
      accelerationZ = aaReal.z;
      accelerationY = aaReal.y;
      accelerationX = aaReal.x;
      accelerationZgrav = aa.z;
      accelerationZ = accelerationZ / 8192 * 10;
      accelerationY = accelerationY / 8192 * 10;
      accelerationX = accelerationX / 8192 * 10;
      accelerationZgrav = accelerationZgrav / 8192 * 10;
      DEBUG_PRINT("areal\t");
      DEBUG_PRINT(accelerationX);
      DEBUG_PRINT("\t");
      DEBUG_PRINT(accelerationY);
      DEBUG_PRINT("\t");
      DEBUG_PRINTLN(accelerationZgrav);
      DEBUG_PRINT(normAccelerationX);
      DEBUG_PRINT("\t");
      DEBUG_PRINTLN(normAccelerationY);
      
      // blink LED to indicate activity
      blinkState = true;
      Wire.endTransmission(true);
    }
  }
  ////////////////////////////////////////////////////////////////////////
  //// ENDOF READING GYRO VALUES
  ////////////////////////////////////////////////////////////////////////

  // program pin should be pulled to ground through a resistor to activate.
  // when program button is pushed:
  if (false == digitalRead(PROGRAM_PIN) && gMode != MODE_FULL_PROGRAM)
  {
    // give 10 seconds to program
    gMode = MODE_QUICK_PROGRAM;

    // turn indicators off for QUICK PROGRAM mode
    digitalWrite(INFORMATION_INDICATOR_PIN, LOW);
    digitalWrite(ERROR_INDICATOR_PIN, LOW);

    // wait two seconds and test program pin again, if pin if still held low, enter full program mode
    // otherwise read sensitivity pots and return to run mode with new sensitivity readings.
    delay(2000);

    if (false == digitalRead(PROGRAM_PIN))
    {
      gMode = MODE_FULL_PROGRAM;
      // give 10 seconds to program
      ulProgramModeExitTime = ulMillis + 10000;

      unThrottleMin = RC_NEUTRAL;
      unThrottleMax = RC_NEUTRAL;
      unSteeringMin = RC_NEUTRAL;
      unSteeringMax = RC_NEUTRAL;

      unThrottleCenter = unThrottleIn;
      unSteeringCenter = unSteeringIn;
    }
    else
    {
      gMode = MODE_RUN;
      ulMillis = millis();
      ulLastThrottleIn = ulMillis;
    }

    // Take new sensitivity and decay readings for quick program and full program modes here
    readSettings();
  }

  if (gMode == MODE_FULL_PROGRAM)
  {
    if (ulProgramModeExitTime < ulMillis)
    {
      // set to 0 to exit program mode
      ulProgramModeExitTime = 0;
      gMode = MODE_RUN;

      writeSettingsToEEPROM();

      ulLastThrottleIn = ulMillis;
    }
    else
    {
      if (unThrottleIn > unThrottleMax && unThrottleIn <= RC_MAX)
      {
        unThrottleMax = unThrottleIn;
      }
      else if (unThrottleIn < unThrottleMin && unThrottleIn >= RC_MIN)
      {
        unThrottleMin = unThrottleIn;
      }

      if (unSteeringIn > unSteeringMax && unSteeringIn <= RC_MAX)
      {
        unSteeringMax = unSteeringIn;
      }
      else if (unSteeringIn < unSteeringMin && unSteeringIn >= RC_MIN)
      {
        unSteeringMin = unSteeringIn;
      }
    }
  }
  else if (gMode == MODE_RUN)
  {
    // if there is no throttle input for 500 ms, ERROR mode/failsafe has to turn on:
    if ((ulLastThrottleIn + 500) < ulMillis)
    {
      gMode = MODE_ERROR;
    }
    else /// we have input
    {
      /// Here is what the serious shit goes down:
      /// Calculate drivers intention (or the ESCs):
      if (updateFlag)
      {
        updateFlag = false;
        brake_blocked = false;
        
        float intervalMillis = intervalLoop / 1000;
        float RPms = 1 / intervalMillis;
        float RPS = RPms * 1000 / 9;
        dTrainSpeed = RPS * (float) wheelCircumference / 1000; 

        lastSpeedInterval = intervalMillis;
        lastSpeedUpdateMillis = ulMillis;        
      }  
      else if ((ulMillis - lastSpeedUpdateMillis) > 167)
      {
        dTrainSpeed = 0;    
        brake_blocked = false;
      }    
      /// TODO: Check if casting the values to float in the calculation is sufficient
      float neutral = unSteeringCenter;
      float steering = unSteeringOut;
      dSteering = steering - neutral;
      float maxIn = MAX_IN_ANGLE;
      float maxOut = MAX_OUT_ANGLE;
      float diff = RC_DIFF;
      /// by changing steering deadzone we can eliminate drifting of the car at
      /// a centered controller, useful if trimming doesn't yield the correct results
      if (dSteering < (float) - steeringDeadzone) /// Going left, right wheel is outer wheel
      {
        wAngleFL = dSteering * maxIn / diff;
        wAngleFR = dSteering * maxOut / diff;
        turn = TURN_LEFT;
      }
      else if (dSteering > (float) steeringDeadzone) /// Going right, left wheel is outer wheel
      {
        wAngleFL = dSteering * maxOut / diff;
        wAngleFR = dSteering * maxIn / diff;
        turn = TURN_RIGHT;
      }
      else /// We are going straight
      {
        wAngleFL = 0;
        wAngleFR = 0;
        turn = TURN_NONE;
      }
      float tAngle;
      float tRadiusFA;
      float tRadiusRA;
      if (wAngleFL < -ESC_deadzone || wAngleFR > ESC_deadzone) // we are effectively turning
      {
        /// Single track vehicle model, for 1:5 with seperately applicable brakes,
        /// 4-wheeel model is more precise and useful
        tAngle = (wAngleFL + wAngleFR) / 2;
        tRadiusFA = WHEELBASE / sin(tAngle);
        tRadiusRA = WHEELBASE / tan(tAngle);

        /// Intended course of the car:
        intSpeedFAx = dTrainSpeed * sin(tAngle);
        intSpeedFAy = dTrainSpeed * cos(tAngle);
        intSpeedRAx = 0;
        intSpeedRAy = dTrainSpeed;
        /// Average out wheel speed vectors to get centerpoint velocity:
        intSpeedX = intSpeedFAx / 2;
        intSpeedY = (intSpeedFAy + intSpeedRAy) / 2;
      }
      else // no risky calculations to avoid overflows
      {
        intSpeedFAx = 0;
        intSpeedFAy = dTrainSpeed;
        intSpeedRAx = 0;
        intSpeedRAy = dTrainSpeed;
        intSpeedX = 0;
        intSpeedY = dTrainSpeed;
      }
      ulSpeedMicros = micros(); /// record time to calculate acceleration
      if (intLastSpeedX != intSpeedX || intLastSpeedY != intSpeedY)
      { /// Maybe refine condition to a specific difference based on gyro sensitivity
        /// i.e. why calculate with it, if the sensor won't pick it up

        /// Acceleration along the intended course:
        /// magnitude and/or direction: vector difference over time
        float deltaTime = ulSpeedMicros - ulLastSpeedMicros;
        intAccFAx = (intSpeedFAx - intLastSpeedFAx) / deltaTime;
        intAccFAy = (intSpeedFAy - intLastSpeedFAy) / deltaTime;
        intAccRAx = (intSpeedRAx - intLastSpeedRAx) / deltaTime;
        intAccRAy = (intSpeedRAy - intLastSpeedRAy) / deltaTime;
        intAccelerationX = (intSpeedX - intLastSpeedX) / deltaTime;
        intAccelerationY = (intSpeedY - intLastSpeedY) / deltaTime;
      }
      else
      {
        intAccFAx = 0;
        intAccFAy = 0;
        intAccRAx = 0;
        intAccRAy = 0;
        intAccelerationX = 0;
        intAccelerationY = 0;
      }
      /// Optimal parameters calculated: measured parameters will be compared to these:
      float centripetalFAx = 0;
      float centripetalFAy = 0;
      float centripetalRAx = 0;
      float centripetalRAy = 0;
      /// TODO: Condition these for angles > or < than zero:
      if (wAngleFL < -ESC_deadzone || wAngleFR > ESC_deadzone)
      {
        centripetalFAx = pow(dTrainSpeed, 2) / tRadiusFA * cos(tAngle);
        centripetalFAy = pow(dTrainSpeed, 2) / tRadiusFA * sin(tAngle);
        centripetalRAx = pow(dTrainSpeed, 2) / tRadiusRA;
        centripetalRAy = 0;
      }

      float sumAccFAx = intAccFAx + centripetalFAx;
      float sumAccFAy = intAccFAy + centripetalFAy;
      float sumAccRAx = intAccRAx + centripetalRAx;
      float sumAccRAy = intAccRAy + centripetalRAy;

      normAccelerationX = (sumAccFAx + sumAccRAx) / 2;
      normAccelerationY = (sumAccFAy + sumAccRAy) / 2;
      
      /// TODO: Adjust for the position of the gyro... or not?
      normYaw = (sumAccFAx - sumAccRAx) / (WHEELBASE / 2);
      
      bool newAngleCame = false;
      float dYawAngle = yawAngle - lastYawAngle;
      if (ulLastYawAngleMillis - millis() > 10 && abs(dYawAngle) > (PI / 64))
      {
        float dYawAngleTime = ulYawAngleMillis - ulLastYawAngleMillis;
        yawV = dYawAngle * 1000 / dYawAngleTime;
        ulYawVMillis = millis();
        float t1 = (float) ulLastYawVMillis / 1000;
        float t2 = (float) ulYawVMillis / 1000;
        yawA = (yawV - lastYawV) / (t2 - t1) ;
        newAngleCame = true;

        lastYawAngle = yawAngle;
        lastYawV = yawV;
        lastYawA = yawA;
        ulLastYawAngleMillis = ulYawAngleMillis;
        ulLastYawVMillis = ulYawVMillis;       
      }
      else
      {

      }

      bool unstable = false;
      /// TODO: condition for speed as well
      if (abs(accelerationX) < (abs(normAccelerationX) - understeerXdiff) || accelerationY > (normAccelerationY + understeerYdiff))
      { // i.e. if acceleration is smaller than normal or
        // acceleration points more to the front of the car than calculated
        if (turn != TURN_NONE)
        {
          state = ST_UNDERSTEER;
          unstable = true;
        }
      }
      
      if (abs(yawA) > (abs(normYaw) + oversteer_yaw_difference))
      {
        state = ST_OVERSTEER;
        unstable = true;
      }
      
      if (!unstable)
      {
        state = ST_BALANCED;
      }

      /// This is deliberately independent from and comes after the rest,
      /// So we dont assume under/oversteer when skidding upside down :)
      if (accelerationZgrav < -4) 
      {/// we tipped over or flying up with 14 m/s^2, less can be triggered by vibrations
        state = ST_ROLLED;
      }
      

      if (state == ST_BALANCED)
      {
        /// What to do with steering:
        if (turn == TURN_NONE)
        {
          if (accelerationX > straightSensitivity) /// deviating to the right
          {
            unSteeringOut = unSteeringOut + straightline_corr;
          }
          else if (accelerationX < -straightSensitivity) /// deviating to the left
          {
            unSteeringOut = unSteeringOut - straightline_corr;
          }
          else
          {
            unSteeringOut = unSteeringCenter;
          }
          unSteeringOut = constrain(unSteeringOut, unSteeringMin, unSteeringMax);
        }
        else
        {
          unSteeringOut = unSteeringIn;
        }        
        unLastBalancedSt = unSteeringOut;
        /// What to do with throttle:
        /// if there is no other issue:
        if (!brake_blocked) { unThrottleOut = unThrottleIn; }
      }
      else if (state == ST_OVERSTEER)
      {
        int8_t corrInt = 0;
        float corrFloat = 0;
        if (newAngleCame) 
        {
          corrFloat = dYawAngle * diff / 0.436332313 * oversteer_steering_multi;
          corrInt = round(corrFloat);
        }
        else
        {
          corrInt = 0;
        }
        
        unSteeringOut = unSteeringOut + corrInt;
        /// ESC can respond with any range not only what's set on the controller:
        unSteeringOut = constrain(unSteeringOut, RC_MIN, RC_MAX);
        unThrottleOut = unThrottleIn - oversteer_throttle_corr;
        unThrottleOut = constrain(unThrottleOut, RC_MIN, RC_MAX);
      }
      else if (state == ST_UNDERSTEER)
      {
        /// throttleMax means full braking on this car
        unThrottleOut = unThrottleOut - understeer_throttle_corr;
        unThrottleOut = constrain(unThrottleOut, unThrottleMin, unThrottleMax);
        unSteeringOut = unSteeringIn;
      }
      else if (state == ST_SPINNING)
      {

      }
      else if (state == ST_ROLLED)
      {        
        unSteeringOut = unSteeringCenter;
        unThrottleOut = unThrottleMax;
        unThrottleOut = constrain(unThrottleOut, RC_MIN, RC_MAX);
        /// TODO: tell the other arduino to stop keeping the engine running
      }
      
      if (unThrottleOut > (unThrottleCenter + brake_deadzone)) /// i.e we are braking
      {
        if ((ulMillis - lastSpeedUpdateMillis) > lastSpeedInterval)
        {
          brake_blocked = true;
          blockedThrottleOut = unThrottleOut;
        }
        if (brake_blocked)
        {
          ABS_corr += 10;
          unThrottleOut = blockedThrottleOut - ABS_corr;
          unThrottleOut = constrain(unThrottleOut, unThrottleCenter + brake_deadzone, unThrottleMax); 
        }
        else
        {
          ABS_corr = 0;
        }
        digitalWrite(BRAKELIGHT_PIN, LOW);        
      }
      else /// we are not braking
      {
        digitalWrite(BRAKELIGHT_PIN, HIGH);
      }
      
      // Checking for a new value can be a good idea
      if (bUpdateFlags & THROTTLE_FLAG)
      {
        ulLastThrottleIn = ulMillis;
      }
      if (bUpdateFlags & STEERING_FLAG)
      {        
        
      }
      /// Last values:
      intLastSpeedFAx = intSpeedFAx;
      intLastSpeedFAy = intSpeedFAy;
      intLastSpeedRAx = intSpeedRAx;
      intLastSpeedRAy = intSpeedRAy;
      intLastSpeedX = intSpeedX;
      intLastSpeedY = intSpeedY;
      ulLastSpeedMicros = ulSpeedMicros;
      unLastSteeringOut = unSteeringOut;            
    }    
    if (easyTransferIn.receiveData())
    {
      gMode = rxdata.gMode;
      engine_throttle_corr = rxdata.engine_throttle_corr;
    }
  }
  else if (gMode == MODE_ERROR)
  {
    /// TODO: intensive braking, while maintaining stability, intended course straight
    /// TODO: if possible turn around and start slowly advancing back
    /// TODO: flash lights to indicate error
    /// TODO: send error signal to controller

    // allow steering to get to safety
    if (bUpdateFlags & STEERING_FLAG)
    {
      unSteeringIn = constrain(unSteeringIn, unSteeringMin, unSteeringMax);
    }
  }

  /// Only set servo output here at the end, so that all modulations can be done first
  servoThrottle.writeMicroseconds(unThrottleOut);
  servoSteering.writeMicroseconds(unSteeringOut);

  bUpdateFlags = 0;

  if (state == ST_OVERSTEER)
  {
    // yel0:
    digitalWrite(ERROR_INDICATOR_PIN, HIGH);    
    digitalWrite(INFORMATION_INDICATOR_PIN, HIGH);
    digitalWrite(GYRO_LED_PIN, LOW);
  }
  else if (state == ST_UNDERSTEER)
  {
    // purple:
    digitalWrite(ERROR_INDICATOR_PIN, HIGH);    
    digitalWrite(INFORMATION_INDICATOR_PIN, LOW);
    digitalWrite(GYRO_LED_PIN, HIGH);
  }
  else if (state == ST_ROLLED)
  {
    // blue:
    digitalWrite(ERROR_INDICATOR_PIN, LOW);    
    digitalWrite(INFORMATION_INDICATOR_PIN, LOW);
    digitalWrite(GYRO_LED_PIN, HIGH);
  }
  else
  {
    animateIndicatorsAccordingToMode(gMode, ulMillis);
  }
  
  txdata.state = state;
  txdata.gMode = gMode;
  txdata.dTrainSpeed = dTrainSpeed;

  easyTransferOut.sendData(0x9);
}

void animateIndicatorsAccordingToMode(uint8_t gMode, uint32_t ulMillis)
{
  static uint32_t ulLastUpdateMillis;
  static boolean bAlternate;
  byte value = 0;

  if (bAlternate) { value = 100; }
  else { value = 0; }

  if (ulMillis > (ulLastUpdateMillis + 1000))
  {
    ulLastUpdateMillis = ulMillis;
    bAlternate = (!bAlternate);
    switch (gMode)
    {
      // flash alternating info and error once a second
      case MODE_FORCEPROGRAM:
        digitalWrite(ERROR_INDICATOR_PIN, bAlternate);
        digitalWrite(INFORMATION_INDICATOR_PIN, false == bAlternate);
        break;
      // steady info, turn off error
      case MODE_RUN:
        digitalWrite(ERROR_INDICATOR_PIN, LOW);
        digitalWrite(INFORMATION_INDICATOR_PIN, HIGH);
        digitalWrite(GYRO_LED_PIN, bAlternate);
        break;
      // flash info once a second, turn off error
      case MODE_FULL_PROGRAM:
        digitalWrite(INFORMATION_INDICATOR_PIN, bAlternate);
        digitalWrite(ERROR_INDICATOR_PIN, LOW);
        break;
      // alternate error, turn off info
      // MODE_QUICK_PROGRAM is self contained and should never get here,
      // if it does we have an error.
      default:
      case MODE_ERROR:
        digitalWrite(INFORMATION_INDICATOR_PIN, LOW);
        digitalWrite(ERROR_INDICATOR_PIN, bAlternate);
        break;
    }
    blinkState = false;
  }
}

// ================================================================
// ===               INTERRUPT DETECTION ROUTINES               ===
// ================================================================
void dmpDataReady() {
  mpuInterrupt = true;
}

void calcThrottle()
{
  // if the pin is high, its a rising edge of the signal pulse, so lets record its value
  if (PIND & 4)
  {
    ulThrottleStart = micros();
  }
  else
  {
    // else it must be a falling edge, so lets get the time and subtract the time of the rising edge
    // this gives use the time between the rising and falling edges i.e. the pulse duration.
    unThrottleInShared = (uint16_t)(micros() - ulThrottleStart);
    // use set the throttle flag to indicate that a new throttle signal has been received
    bUpdateFlagsShared |= THROTTLE_FLAG;
  }
}

void calcSteering()
{
  /// TODO: try digitalRead() to get cross-chip compatibility
  if (PIND & 8)
  {
    ulSteeringStart = micros();
  }
  else
  {
    unSteeringInShared = (uint16_t)(micros() - ulSteeringStart);
    bUpdateFlagsShared |= STEERING_FLAG;
  }  
}

void calcSpeed()
{
  bool readSensor = digitalRead(A3);
  if(readSensor) // white
  {
    if (readFlag)
    {
      interval = micros() - whiteStart;
      whiteStart = micros();
      readFlag = false;
      updateFlag = true;
    }    
  }
  else // black
  {
    readFlag = true;
  }  
}

uint8_t readSettingsFromEEPROM()
{
  uint8_t bError = false;

  unSteeringMin = readChannelSetting(EEPROM_INDEX_STEERING_MIN);
  if (unSteeringMin < RC_MIN || unSteeringMin > RC_NEUTRAL)
  {
    unSteeringMin = RC_MIN;
    bError = true;
  }
  DEBUG_PRINTLN(unSteeringMin);

  unSteeringMax = readChannelSetting(EEPROM_INDEX_STEERING_MAX);
  if (unSteeringMax > RC_MAX || unSteeringMax < RC_NEUTRAL)
  {
    unSteeringMax = RC_MAX;
    bError = true;
  }
  DEBUG_PRINTLN(unSteeringMax);

  unSteeringCenter = readChannelSetting(EEPROM_INDEX_STEERING_CENTER);
  if (unSteeringCenter < unSteeringMin || unSteeringCenter > unSteeringMax)
  {
    unSteeringCenter = RC_NEUTRAL;
    bError = true;
  }
  DEBUG_PRINTLN(unSteeringCenter);

  unThrottleMin = readChannelSetting(EEPROM_INDEX_THROTTLE_MIN);
  if (unThrottleMin < RC_MIN || unThrottleMin > RC_NEUTRAL)
  {
    unThrottleMin = RC_MIN;
    bError = true;
  }
  DEBUG_PRINTLN(unThrottleMin);

  unThrottleMax = readChannelSetting(EEPROM_INDEX_THROTTLE_MAX);
  if (unThrottleMax > RC_MAX || unThrottleMax < RC_NEUTRAL)
  {
    unThrottleMax = RC_MAX;
    bError = true;
  }
  DEBUG_PRINTLN(unThrottleMax);

  unThrottleCenter = readChannelSetting(EEPROM_INDEX_THROTTLE_CENTER);
  if (unThrottleCenter < unThrottleMin || unThrottleCenter > unThrottleMax)
  {
    unThrottleCenter = RC_NEUTRAL;
    bError = true;
  }
  DEBUG_PRINTLN(unThrottleCenter);

  return (false == bError);
}

void writeSettingsToEEPROM()
{
  writeChannelSetting(EEPROM_INDEX_STEERING_MIN, unSteeringMin);
  writeChannelSetting(EEPROM_INDEX_STEERING_MAX, unSteeringMax);
  writeChannelSetting(EEPROM_INDEX_STEERING_CENTER, unSteeringCenter);
  writeChannelSetting(EEPROM_INDEX_THROTTLE_MIN, unThrottleMin);
  writeChannelSetting(EEPROM_INDEX_THROTTLE_MAX, unThrottleMax);
  writeChannelSetting(EEPROM_INDEX_THROTTLE_CENTER, unThrottleCenter);

  DEBUG_PRINTLN(unSteeringMin);
  DEBUG_PRINTLN(unSteeringMax);
  DEBUG_PRINTLN(unSteeringCenter);
  DEBUG_PRINTLN(unThrottleMin);
  DEBUG_PRINTLN(unThrottleMax);
  DEBUG_PRINTLN(unThrottleCenter);
}


uint16_t readChannelSetting(uint8_t nStart)
{
  uint16_t unSetting = (EEPROM.read((nStart * sizeof(uint16_t)) + 1) << 8);
  unSetting += EEPROM.read(nStart * sizeof(uint16_t));

  return unSetting;
}

void writeChannelSetting(uint8_t nIndex, uint16_t unSetting)
{
  EEPROM.write(nIndex * sizeof(uint16_t), lowByte(unSetting));
  EEPROM.write((nIndex * sizeof(uint16_t)) + 1, highByte(unSetting));
}

void readSettings()
{

}

void receiveEvent(int howMany)
{
  
}

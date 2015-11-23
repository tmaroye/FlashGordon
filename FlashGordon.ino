/* FlashGordon Mini Sumo Robot
 Sumo Code 1.0 Beta
 LMR User: Thierry MAROYE
 October 2015
 */

// SETUP PINS

//-- MOTEURS L  --
#define PWRL 5 //Connecté à Arduino pin 5(sortie pwm)
#define DIRL 7 //Connecté à Arduino pin 7

//-- MOTEURS R --
#define PWRR 6 //Connecté à Arduino pin 6(Sortie pwm)
#define DIRR 8 //Connecté à Arduino pin 4

// -- IR --
#define lineL 9 //Connecté à Arduino pin 9
#define lineR 3 //Connecté à Arduino pin 3
#define lineB 4 //Connecté à Arduino pin 4

// -- IR reception --
#define IRPin 10 // Connecté à Arduino pin 10

// -- 8x8 Matrix --
#define MatrixClkPin  11 // Connecté à Arduino pin 11
#define MatrixCsPin 12 // Connecté à Arduino pin 12
#define MatrixDinPin  13 // Connecté à Arduino pin 13


// -- Definition --
#define Matrix_0 0
#define Matrix_1 1
#define Matrix_2 2
#define Matrix_3 3
#define Matrix_4 4
#define Matrix_5 5
#define Matrix_6 6
#define Matrix_7 7
#define Matrix_8 8
#define Matrix_9 9
#define Matrix_Vide 10
#define Matrix_Good 11
#define Matrix_Bad 12
#define Matrix_Up 13
#define Matrix_Stop 14
#define Matrix_On 15
#define Matrix_Plus 16
#define Matrix_Moin 17
#define Matrix_Left 18
#define Matrix_Right 19
#define Matrix_Down 20
#define Matrix_Pause 21
#define Matrix_Eq 22
#define Matrix_Cal 23
#define Matrix_LineLeft 24
#define Matrix_LineRight 25
#define Matrix_LineLeftRight 26
#define Matrix_LineBack 27
#define Matrix_St 28
#define Matrix_Warm 29

// Define commands that can be executed, first 2 are internally generated
#define NONE 0
#define IMU_WARM 1
#define CAL 2
#define Stop 3
#define UP 5
#define DN 6
#define LEFT 7
#define RIGHT 8
#define GO 9
#define IMU_OK 10
#define SPEEDP 11
#define SPEEDM 12
#define BorderBack 13
#define BorderLeft 14
#define BorderRight 15
#define BorderLeftRight 16
#define DELAY 17


// -- The library --
#include <IRremote.h>
//We always have to include the library
#include <LedControl.h>
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include <I2Cdev.h>
#include "MPU6050_6Axis_MotionApps20.h"
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// Declaration
long setPointBack;
long setPointLeft;
long setPointRight;
long setPointDef = 25; // Default value in case we don't use de calibration procedur
long readIRL;
long readIRR;
long readIRB;
int matrixOldCode = 999;

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
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// Now we need a LedControl to work with. We have only a single MAX72XX.
LedControl lc = LedControl(MatrixDinPin, MatrixClkPin, MatrixCsPin, 1);

IRrecv reception_ir(IRPin); // crée une instance
decode_results decode_ir; // stockage données reçues

MPU6050 mpu;

// ISR
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
  #endif
  // initialize serial communication
  // (115200 chosen because it is required for FiFo not OverFlow
    Serial.begin(115200);

    Serial.println("Flash Gordon V1.0 Start...");
  reception_ir.enableIRIn(); // démarre la réception
  pinMode(DIRL, OUTPUT);
  pinMode(DIRR, OUTPUT);
  pinMode(lineB, INPUT);
  reception_ir.blink13(false);
  // The MAX72XX is in power-saving mode on startup, we have to do a wakeup call
  lc.shutdown(0, false);
  lc.setIntensity(0, 5); // Set the brightness to a medium values
  lc.clearDisplay(0); // and clear the display */
  // initialize device
  //Serial.println(F("Initializing I2C devices...f="));
  mpu.initialize();

  // verify connection
  //Serial.println(F("Testing device connections..."));
  //Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  if (!mpu.testConnection())
  {
    WriteMatrix(Matrix_Bad);
    delay(5000);
    WriteMatrix(Matrix_0);
    while (1);
  }

  // load and configure the DMP
  // Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  // Note - use the 'raw' program to get these.
  // Expect an unreliable or long startup if you don't bother!!!
  mpu.setXGyroOffset(255);
  mpu.setYGyroOffset(255);
  mpu.setZGyroOffset(255);
  mpu.setXAccelOffset(-1);
  mpu.setYAccelOffset(1023);
  mpu.setZAccelOffset(1791);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    // Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    // Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    // Serial.println(F("DMP ready! Waiting for first interrupt..."));
    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    // Serial.print(F("DMP Initialization failed (code "));
    // Serial.print(devStatus);
    // Serial.println(F(")"));
WriteMatrix(Matrix_Bad);
delay(5000);
WriteMatrix(devStatus);
    while (1); // If bug ws stop !
  }//END Setup
  mStop(); // we stop the motor.
  WriteMatrix(Matrix_Warm);
}

// Main loop of program
// Repeat forever

void loop() {
  static int    SubLoop, Demand;
  static byte   Moving;
  static float  Heading, HeadingTgt;
  static byte Command;
  float Reduction = .3;
  int i;

  SubLoop++;

  // *********************************************************************
  // ** 100Hz Fast Loop                                                 **
  // *********************************************************************
  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) 
  {
  }

  GetHeading(&Heading, &HeadingTgt, Moving);
  Heading = ecretage(Heading); // do noting now :(

  PID(Heading, HeadingTgt, &Demand, Reduction * 15, Reduction * .08, 0, Moving);	// If not moving zero integral

  // *********************************************************************
  // ** 10Hz Loop                                                      **
  // *********************************************************************
  if ((SubLoop % 2) == 0)
  {
    // Get user command
    DecodeUserSwitch(&Command, &Moving);
    //Get system gen commands
    CheckIMU(&Command, Heading); // Look to see when the IMU has warmed up, issue a CMD when it has otherwise prevent start

    BorderCheck(&Command);

    Serial.print( " Moving : ");
    Serial.print(Moving);
    Serial.print("; Command : ");
    Serial.print(Command);
    Serial.print("; Heading : ");
    Serial.print(Heading);
    Serial.print("; HeadingTgt : ");
    Serial.print(HeadingTgt);
    Serial.print("; Demand : ");
    Serial.println(Demand);

    MatrixRun(&Command);
    //Execute commads
    ExecuteCommand(&Command, &Moving, &HeadingTgt, Demand) ;
  }//END 10HZ
    }

//                                   Procedur
// #############################################################################


// #############################################################################
// ###                    IR Sensor Calibation Procedur                      ###
// #############################################################################
 
void Calibrate() {
  Serial.println("Calibation");
  setPointBack = RCTime(lineB);
  setPointRight = RCTime(lineR);
  setPointLeft = RCTime(lineL);
  setPointDef = (setPointBack - (setPointLeft + setPointRight) / 2) / 8;
  // debug
  /*  Serial.print(" White : ");
    Serial.print(setPointWhite);
    Serial.print(" Black : ");
    Serial.print(setPointBlack);
    Serial.print(" Moy : ");
    Serial.println(setPointDef);*/
  }

// #############################################################################
// ###                             RCTime procedur                           ###
// #############################################################################

long RCTime(int sensorIn) {
  long duration = 0;
  pinMode(sensorIn, OUTPUT);     // Make pin OUTPUT
  digitalWrite(sensorIn, HIGH);  // Pin HIGH (discharge capacitor)
  delay(1);                      // Wait 1ms
  pinMode(sensorIn, INPUT);      // Make pin INPUT
  digitalWrite(sensorIn, LOW);   // Turn off internal pullups
  while (digitalRead(sensorIn)) { // Wait for pin to go LOW
    duration++;
  }
  return duration;
}

// #############################################################################
// ###                            QTI procedur                               ###
// #############################################################################

bool QTI(int sensorIn) {
  long duration = 0;
  bool line = false;

  pinMode(sensorIn, OUTPUT);     // Make pin OUTPUT
  digitalWrite(sensorIn, HIGH);  // Pin HIGH (discharge capacitor)
  delay(1);                      // Wait 1ms
  pinMode(sensorIn, INPUT);      // Make pin INPUT
  digitalWrite(sensorIn, LOW);   // Turn off internal pullups
  while (digitalRead(sensorIn) & duration <= setPointDef * 2) { // Wait for pin to go LOW
    duration++;
  }
  if (duration < setPointDef) line = true;
  return line;
}


// #############################################################################
// ###                       mStop motor procedur                            ###
// #############################################################################

void mStop() {
  digitalWrite(DIRL, HIGH);
  digitalWrite(DIRR, HIGH);
  analogWrite(PWRL, LOW);
  analogWrite(PWRR, LOW);
}

// #############################################################################
// ###                     Matrix 8x8 Max7219 procedur                       ###
// #############################################################################

unsigned char ListCode[31][8] = {
  {0x3C, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x3C}, //0
  {0x10, 0x18, 0x14, 0x10, 0x10, 0x10, 0x10, 0x10}, //1
  {0x7E, 0x2, 0x2, 0x7E, 0x40, 0x40, 0x40, 0x7E}, //2
  {0x3E, 0x2, 0x2, 0x3E, 0x2, 0x2, 0x3E, 0x0}, //3
  {0x8, 0x18, 0x28, 0x48, 0xFE, 0x8, 0x8, 0x8}, //4
  {0x3C, 0x20, 0x20, 0x3C, 0x4, 0x4, 0x3C, 0x0}, //5
  {0x3C, 0x20, 0x20, 0x3C, 0x24, 0x24, 0x3C, 0x0}, //6
  {0x3E, 0x22, 0x4, 0x8, 0x8, 0x8, 0x8, 0x8}, //7
  {0x0, 0x3E, 0x22, 0x22, 0x3E, 0x22, 0x22, 0x3E}, //8
  {0x3E, 0x22, 0x22, 0x3E, 0x2, 0x2, 0x2, 0x3E}, //9
  {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0}, // 10 - vide
  {0x3c, 0x42, 0xa5, 0x81, 0xa5, 0x99, 0x42, 0x3c}, // 11 - Smile good
  {0x3c, 0x42, 0xa5, 0x81, 0x99, 0xa5, 0x42, 0x3c}, // 12 - Smile bad
  {0x18, 0x3c, 0x7e, 0xff, 0x18, 0x18, 0x18, 0x18}, //13 - Forward
  {0x81, 0x42, 0x24, 0x18, 0x18, 0x24, 0x42, 0x81}, // 14 - Stop
  {0x00, 0x00, 0x00, 0xed, 0xad, 0xab, 0xeb, 0x00}, // 15 - On
  {0x00, 0x00, 0x10, 0x10, 0x7c, 0x10, 0x10, 0x00}, // 16 - +
  {0x00, 0x00, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x00}, // 17 - -
  {0x10, 0x30, 0x70, 0xff, 0xff, 0x70, 0x30, 0x10}, // 18 Left
  {0x08, 0x0c, 0x0e, 0xff, 0xff, 0x0e, 0x0c, 0x08}, // 19 Right
  {0x18, 0x18, 0x18, 0x18, 0xff, 0x7e, 0x3c, 0x18}, // 20 Reverce
  {0x00, 0x6c, 0x6c, 0x6c, 0x6c, 0x6c, 0x6c, 0x00}, // 21 Pause
  {0x00, 0x00, 0xee, 0x8a, 0xca, 0x8a, 0xee, 0x01}, // 22 Eq
  {0x00, 0x00, 0xe1, 0x81, 0x9d, 0x95, 0xff, 0x00}, // 23 Cal
  {0xc0, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // 24 LineLeft
  {0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // 25 LineRight
  {0xc3, 0xc3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // 26 LineLeftRight
  {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x18}, // 27 LineBack
  {0x00, 0x00, 0x00, 0xee, 0x84, 0xe4, 0x24, 0xe4}, // 28 St
  {0x0, 0x49, 0x49, 0x49, 0x49, 0x2A, 0x1C, 0x0}, // 29 W (Warm)

};

// Write the matrix code
void WriteMatrix(int code)
{
  unsigned char DATA ;

  for (int ii = 0 ; ii <= 7 ; ii++)
  {
    DATA = ListCode[code][ii];
    lc.setRow(0, ii, DATA);
  }
} // End WriteMatrix

// #############################################################################
// ###                             GetHeading procedur                       ###
// #############################################################################

// Use the IMU to get the curreent heading.  This is 0-360 degrees.
// It's not relative to North but where the robot was pointing when the
// GO was demand.
void  GetHeading(float *Heading, float *HeadingTgt, byte Moving)
{
  {
    //calc heading from IMU
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
      // reset so we can continue cleanly
      mpu.resetFIFO();
      WriteMatrix(Matrix_Bad);
      delay(5000);
      // error code 3 is FOFO overflow
      WriteMatrix(Matrix_3);
      while (1);
      //  Serial.println(F("FIFO overflow!"));
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

      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      //Serial.print("ypr\t");
      *Heading = (ypr[0] * 180 / M_PI) + 180;

    }//done
  }

  if (!Moving)
  {
    *HeadingTgt = *Heading;
  }
}//END GetHeading

// #############################################################################
// ###                    Decode User Ir Keyboard procedur                   ###
// #############################################################################

void DecodeUserSwitch(byte *Command, byte *Moving)
{
  if (reception_ir.decode(&decode_ir))
  {
    // Serial.print("decode ir : ");
    // Serial.println(decode_ir.value);
    switch (decode_ir.value)
    {
      case  16580863 : // Touche On
        //  WriteMatrix(Matrix_On);
        *Command = GO;
        *Moving = 1;
        break;

      case  16613503 : // Touche Vol +
        //  Serial.println("Touche Vol +");
        *Command = SPEEDP;
        break;

      case  16597183 : // Touche Func/Stop
        // Serial.println("Touche Func/Stop");
        *Command = Stop;
        break;

      case  16589023 : // Touche FLeft
        // Serial.println("Touche FLeft");
        *Command = LEFT;
        break;

      case  16621663 : // Touche Pause
        // Serial.println("Touche Pause");
        WriteMatrix(Matrix_Pause);
        break;

      case  16605343 : // Touche FRight
        // Serial.println("Touche FRight");
        *Command = RIGHT;
        break;

      case  16584943 : // Touche FDown
        // Serial.println("Touche FDown");
        *Command = DN;
        break;

      case  16617583 : // Touche Vol -
        // Serial.println("Touche Vol -");
        *Command = SPEEDM ;
        break;

      case  16601263 : // Touche FUp
        // Serial.println("Touche FUp");
        *Command = UP;
        break;

      case  16593103 : // Touche 0
        // Serial.println("Touche 0");
        WriteMatrix(Matrix_0);
        break;

      case  16625743 : // Touche EQ
        // Serial.println("Touche EQ");
        *Command = CAL;
        break;

      case  16609423 : // Touche ST
        // Serial.println("Touche ST");
        WriteMatrix(Matrix_St);
        break;

      case  16582903 : // Touche 1
        // Serial.println("Touche 1");
        WriteMatrix(Matrix_1);

        break;

      case  16615543 : // Touche 2
        // Serial.println("Touche 2");
        WriteMatrix(Matrix_2);
        break;

      case  16599223 : // Touche 3
        // Serial.println("Touche 3");
        WriteMatrix(Matrix_3);
        break;
      case  16591063 : // Touche 4
        // Serial.println("Touche 4");
        WriteMatrix(Matrix_4);
        break;

      case  16623703 : // Touche 5
        // Serial.println("Touche 5");
        WriteMatrix(Matrix_5);
        break;

      case  16607383 : // Touche 6
        // Serial.println("Touche 6");
        WriteMatrix(Matrix_6);
        break;

      case  16586983 : // Touche 7
        // Serial.println("Touche 7");
        WriteMatrix(Matrix_7);
        break;

      case  16619623 : // Touche 8
        // Serial.println("Touche 8");
        WriteMatrix(Matrix_8);
        break;

      case 16603303 : // Touche 9
        // Serial.println("Touche 9");
        WriteMatrix(Matrix_9);
        break;
      }

    reception_ir.resume(); // reçoit le prochain code
  }
}//END DecodeUSerSwitch

// #############################################################################
// ###                               CheckIMU procedur                       ###
// #############################################################################
//
// Check to see if the IMU has settled down and is giving a steady heading.
// If it hasn't disable the go button.

void CheckIMU(byte *Command, float  Heading)
{
  static int Init = 1, Count;
  static float oHeading;

  if (Init)
  {
    //If IMU not stable don't allow the robot to start navigating.
    *Command = IMU_WARM;

    Count++;
    if (Count == 100)
    {
      Count = 0;
      if (abs(Heading - oHeading) < 1)
      {
        Init = 0;
        *Command = IMU_OK;
      }
      else
      oHeading = Heading;
    }
  }
  //  Serial.print(" ChekIMU Command");
  //  Serial.println(*Command);

}//END Check IMU

// #############################################################################
// ###                           Matrix Run procedur                         ###
// #############################################################################

void MatrixRun(byte *Command)
{
  static int Code = 9999;

  switch (*Command)
  {
    case  GO: // Touche On
    Code = Matrix_On;
    break;

    case  IMU_WARM:
    Code = Matrix_Warm;
    break;

    case  IMU_OK:
    Code = Matrix_Good;
    break;

    case  SPEEDP: // Touche Vol +
    Code = Matrix_Plus;
    break;

    case  Stop: // Touche Func/Stop
    Code = Matrix_Stop;
    break;

    case LEFT : // Touche FLeft
    Code = Matrix_Left;
    break;

    case RIGHT: // Touche FRight
    Code = Matrix_Right;
    break;

    case DN: // Touche FDown
    Code = Matrix_Down;
    break;

    case 16617583: // Touche Vol -
    Code = Matrix_Moin;
    break;

    case UP: // Touche FUp
    Code = Matrix_Up;
    break;

    case CAL: // Touche EQ
    Code = Matrix_Cal;
    break;

    case NONE:
    Code = Matrix_Good;
    break;

    case BorderBack:
    Code = Matrix_LineBack;
    break;

    case BorderLeftRight:
    Code = Matrix_LineLeftRight;
    break;

    case BorderLeft:
    Code = Matrix_LineLeft;
    break;

    case BorderRight:
    Code = Matrix_LineRight;
    break;

    case DELAY:
    Code = Matrix_Pause;
    break;

  }

  if (matrixOldCode != Code)
  {
    //Serial.print("Matrix code");
    //Serial.println(Code);
    WriteMatrix(Code);
    matrixOldCode = Code;
  }
}

// #############################################################################
// ###                               PID procedur                            ###
// #############################################################################

// A PID implementation; control an error with 3 constants and
// of 350 as a result of the motor tests.  If not moving do nothing.

void PID(float Hdg, float HdgTgt, int *Demand, float kP, float kI, float kD, byte Moving)
{
  static unsigned long lastTime;
  static float Output;
  static float errSum, lastErr, error ;

  // IF not moving then
  if (!Moving)
  {
    errSum = 0;
    lastErr = 0;
    *Demand = 0;
    return;
  }

  if (!Moving)
  {
    Serial.println("BUG");
    delay(5000);
  }

  //error correction for angular overlap
  error = Hdg - HdgTgt;
  if (error < 180)
  error += 360;
  if (error > 180)
  error -= 360;

  //http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/

  /*How long since we last calculated*/
  unsigned long now = millis();
  float timeChange = (float)(now - lastTime);
  /*Compute all the working error variables*/
  //float error = Setpoint - Input;
  errSum += (error * timeChange);

  //integral windup guard
  LimitFloat(&errSum, -300, 300); // 300 a l'origine

  float dErr = (error - lastErr) / timeChange;

  /*Compute PID Output*/
  *Demand = kP * error + kI * errSum + kD * dErr;
  /*Remember some variables for next time*/
  lastErr = error;
  lastTime = now;

  // Serial.print(" PID demand : ");
  //  Serial.println(*Demand);

  //limit demand
  LimitInt(Demand, -400, 400);  // 400 400
  //  Serial.print(" PID demand : ");
  // Serial.println(*Demand);
}//END getPID

// #############################################################################
// ###                            LimitInt procedur                          ###
// #############################################################################

//  LimitInt
//  Clamp an int between a min and max.

void LimitInt(int *x, int Min, int Max)
{
  if (*x > Max)
  *x = Max;
  if (*x < Min)
  *x = Min;

}//END LimitInt

// #############################################################################
// ###                             LimitFloat procedur                       ###
// #############################################################################
//
// Clamp a float between a min and max.  Note doubles are the same
// as floats on this platform.

void LimitFloat(float *x, float Min, float Max)
{
  if (*x > Max)
  *x = Max;
  if (*x < Min)
  *x = Min;

}//END LimitInt

// #############################################################################
// ###                    Execute Command procedur                           ###
// #############################################################################
//
// This routine executes the command in *CurrentCMD then zeroes it when done.
// This only occurs if we are moving otherwise we are in standby.

void ExecuteCommand(byte *Command, byte *Moving, float *HeadingTgt, float Demand)
{
  static byte state;
  static int ForeDmd, Time, TimeDelta;
  const int ExecuteDelay = 80;

  if (*Moving)
  {

    //Serial.print("; State : ");
    //Serial.println(state);

    if (state == 0)
    {

      switch (*Command)
      {
        case UP:
        ForeDmd = 200;
        break;

        case DN:
        ForeDmd = -ForeDmd;
        *Command = NONE;
        break;

        case LEFT:
        *HeadingTgt -= 90;
        if (*HeadingTgt < 0) *HeadingTgt += 360;
        *Command = NONE;
        break;

        case RIGHT:
        *HeadingTgt += 90;
        if (*HeadingTgt > 360) *HeadingTgt -= 360;
        *Command = NONE;
        break;

        case GO:
        Time = millis();
        state++;
        break;

        case Stop:
        state = 0;
        ForeDmd = 0;
        Demand = 0;
        *Moving = 0;
        *Command = NONE;
        break;

        case CAL:
        ForeDmd = 0;
        *Moving = 0;
        state++;
        break;

        case BorderBack:
        ForeDmd = abs(ForeDmd);
        state++;
        Time = millis();
        *Command = NONE;
        break;

        case BorderLeftRight:
        ForeDmd = -(abs(ForeDmd));
        state++;
        Time = millis();
        *Command = NONE;
        break;

        case BorderLeft:
        *HeadingTgt += 175;
        if (*HeadingTgt > 360) *HeadingTgt -= 360;
        state++;
        Time = millis();
        *Command = NONE;
        break;

        case BorderRight:
        *HeadingTgt -= 175;
        if (*HeadingTgt < 0) *HeadingTgt += 360;
        *Command = NONE;
        state++;
        Time = millis();
        break;

        case NONE:
        break;
      }//END switch

    }//END IF State == 0
    else // State != 0
    {
      // Serial.print( " Else State 1 Command : ");
      // Serial.println(*Command);

      if (*Command == GO)
      {
        TimeDelta = millis() - Time;

        // Serial.print( "TimeDelta : ");
        //  Serial.println(TimeDelta);

        if (TimeDelta < 1000)
        {
          ForeDmd = 0;
          Demand = 0;
          WriteMatrix(Matrix_5);
        }
        else if (TimeDelta >= 1000 & TimeDelta < 2000)
        {
          ForeDmd = 0;
          Demand = 0;
          WriteMatrix(Matrix_4);
        }
        else if (TimeDelta >= 2000 & TimeDelta < 3000)
        {
          ForeDmd = 0;
          Demand = 0;
          WriteMatrix(Matrix_3);
        }
        else if (TimeDelta >= 3000 & TimeDelta < 4000)
        {
          ForeDmd = 0;
          Demand = 0;
          WriteMatrix(Matrix_2);
        }
        else if (TimeDelta >= 4000 & TimeDelta < 5000)
        {
          ForeDmd = 0;
          Demand = 0;
          WriteMatrix(Matrix_0);
        }
        else if (TimeDelta >= 5000)
        {
          Time = 0;
          *Command = UP;
          state = 0;
        }
      }
      else if (*Command == CAL)
      {
        Calibrate();
        state = 0;
        *Command = NONE;
      }
      else // neutralise les commandes le temps de se égader de la bande blanche
        // attention il faudra modifier en cas d'attaque.
        {
          TimeDelta = millis() - Time;
        if (TimeDelta >= 700)  // delay de neutralisation a mettre en varialble apres
        {
          Time = 0;
          state = 0;
          *Command = NONE;
        }
        else
        {
          *Command = DELAY;
        }
      }
    } // State == 0
  }
  else  // Moving
  {
    //idle waiting for go cmd
    state = 0;
    //NOT Moving
    ForeDmd = 0;
    Demand = 0;
  } // end If Moving
  /*  Serial.print( "Command : ");
    Serial.print(*Command);
    Serial.print( "State : ");
    Serial.print(state);
    Serial.print( "ForeDmd : ");
    Serial.print(ForeDmd);
    Serial.print( "Moving : ");
    Serial.print(*Moving);
    Serial.print( "Demand : ");
    Serial.print(Demand);
    Serial.print( "Time : ");
    Serial.println(Time);*/

    DriveMotors( (Demand * -1) + ForeDmd, ( Demand * 1) + ForeDmd, *Moving);

}//END ExecuteCommand

// #############################################################################
// ###                           Drive Motor procedur                        ###
// #############################################################################
//
// Drive port / stbd motors fwd or backwards using PWM.
// A breakout calc is needed to linearise the response since
// torque is proportional to voltage on a DC mottor the wheels
// don't move on the lower 25% range.
//
// A L9110 IC controls the motors and 2 PWMs are used.  2 DOs control
// direction.
//
// Inputs
// -----
// DriveVal    +/-1000
//
// For PID Distance = 400 and PID gets 300
//
void DriveMotors(int PDrive, int SDrive, byte Moving)
{
  int Mag;

  if (!Moving)
  {
    PDrive = 0;
    SDrive = 0;
  }

  LimitInt(&PDrive, -1000, 1000);
  LimitInt(&SDrive, -1000, 1000);

  // ========= Port drive =========
  Mag = abs(PDrive) * .205 + 45;
  if (PDrive == 0) Mag = 0;

  if (PDrive > 0)
  {
    // fwds
    digitalWrite(DIRL, HIGH);
    analogWrite(PWRL, Mag);
  }
  else
  {
    // backwards
    digitalWrite(DIRL, LOW);
    analogWrite(PWRL, Mag);
  }

  // ========= Stbd drive =========
  Mag = abs(SDrive) * .205 + 45;
  if (SDrive == 0) Mag = 0;

  if (SDrive > 0)
  {
    //
    digitalWrite(DIRR, HIGH);
    analogWrite(PWRR, Mag); 
  }
  else
  {
    // fwd
    digitalWrite(DIRR, LOW);
    analogWrite(PWRR, Mag);
  }
  /*
     // Serial.println( "Parametres moteur  : ");
      Serial.print(" P Drive  : ");
      Serial.print(PDrive);
      Serial.print(" S Drive  : ");
      Serial.print(SDrive);
      Serial.print(" Moving  : ");
      Serial.println(Moving);*/

}//END DriveMotors

// #############################################################################
// ###                            BorderCheck procedur                       ###
// #############################################################################

void BorderCheck(byte *Command)
{
  if ((*Command != IMU_WARM) & (*Command != GO))
  {
    readIRL = QTI(lineL);
    readIRR = QTI(lineR);
    readIRB = QTI(lineB);

    // Serial.print(readIRB);

    if ((readIRL) & (readIRR))
    {
      *Command = BorderLeftRight;
    }
    else if (readIRL  )
    {
      // Serial.println(" border L ");
      *Command = BorderLeft;
    }
    else if (readIRR  )
    {
      //Serial.println(" border R ");
      *Command = BorderRight;
    }
    else if (readIRB  )
    {
      //Serial.println(" boreder Back ");
      *Command = BorderBack;
    }
  }
}

// #############################################################################
// ###                               Ecretage procedur                       ###
// #############################################################################
//
// Filtre ecretage
// Ne marche pas du tout !!!
float ecretage(float val)
{
  /*  static bool init = true;
    static float oldVal;
    /*
      Serial.print(" val : ");
      Serial.print(val);
      Serial.print(" oldVal : ");
      Serial.print(oldVal);
      */
  /* if (init)
   {
     if (val != 0 )
     {
       // Serial.print("init");
       oldVal = val;
       init = false;
     }
   }
   else
   {
     if ((val < oldVal * .5) | (val > oldVal * 1.5))
     {
       val = oldVal;
     }
     else
     {
       oldVal = val;
     }
   }

   //  Serial.print(" val 2: ");
   // Serial.println(val);
   */
   return val;
 }

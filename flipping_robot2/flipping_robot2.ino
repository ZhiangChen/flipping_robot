/// Flipping Robot 
/// This is for the EMAE 488 course project

/*The MIT License (MIT)

Copyright (c) 2016 Zhiang Chen

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.*/
//========================
//=== joystick
//========================
#define JS_x A2
#define JS_y A3
#define JS_key 12
//========================
//=== infrared receivers
//========================
#define IR_r1 A0
#define IR_r2 A1
//========================
//=== motor
//========================
#include <AccelStepper.h>
#define HALFSTEP 8
#define motorPin5  8     // Blue   - 28BYJ48 pin 1
#define motorPin6  9     // Pink   - 28BYJ48 pin 2
#define motorPin7  10    // Yellow - 28BYJ48 pin 3
#define motorPin8  11    // Orange - 28BYJ48 pin 4
AccelStepper stepper(HALFSTEP, motorPin5, motorPin7, motorPin6, motorPin8);
#define motor0 5
#define motor1 6
#define motor2 7
#define ON LOW
#define OFF HIGH
//========================
//=== gyroscopes
//========================
#include "I2Cdev.h" 
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
MPU6050 accelgyroIC1(0x68); // AD0 = low
MPU6050 accelgyroIC2(0x69); // AD0 = high
#define OUTPUT_READABLE_YAWPITCHROLL
bool dmpReady1 = false;  // set true if DMP init was successful
bool dmpReady2 = false;
uint8_t mpuIntStatus1;   // holds actual interrupt status byte from MPU
uint8_t mpuIntStatus2;
uint8_t devStatus1;      // return status after each device operation (0 = success, !0 = error)
uint8_t devStatus2;
uint16_t packetSize1;    // expected DMP packet size (default is 42 bytes)
uint16_t packetSize2;
uint16_t fifoCount1;     // count of all bytes currently in FIFO
uint16_t fifoCount2;
uint8_t fifoBuffer1[64]; // FIFO storage buffer
uint8_t fifoBuffer2[64];
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll] 
//========================
//=== gyroscopes interrupt
//========================
volatile bool mpuInterrupt1 = false;     // indicates whether MPU1 interrupt pin has gone high
void dmpDataReady1() { mpuInterrupt1 = true;}
volatile bool mpuInterrupt2 = false;     // indicates whether MPU2 interrupt pin has gone high
void dmpDataReady2() {mpuInterrupt2 = true;}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void setup() {
//========================
//=== serial
//========================
  Serial.begin(38400);

//========================
//=== gyroscopes
//========================
  Serial.println("Initializing I2C devices..."); 
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif
  while (!Serial); // wait for Serial connection
  //accelgyro.initialize(); 
  accelgyroIC1.initialize(); 
  accelgyroIC2.initialize(); 
  // verify connection 
  Serial.println("Testing device connections..."); 
  Serial.println(accelgyroIC1.testConnection() ? "MPU6050 #1 connection successful" : "MPU6050 connection failed"); 
  Serial.println(accelgyroIC2.testConnection() ? "MPU6050 #2 connection successful" : "MPU6050 connection failed"); 
  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus1 = accelgyroIC1.dmpInitialize();
  devStatus2 = accelgyroIC2.dmpInitialize();
  // supply your own gyro offsets here, scaled for min sensitivity
  accelgyroIC1.setXGyroOffset(220);
  accelgyroIC1.setYGyroOffset(76);
  accelgyroIC1.setZGyroOffset(-85);
  accelgyroIC1.setZAccelOffset(1788); // 1688 factory default for my test chip
  accelgyroIC2.setXGyroOffset(220);
  accelgyroIC2.setYGyroOffset(76);
  accelgyroIC2.setZGyroOffset(-85);
  accelgyroIC2.setZAccelOffset(1788); // 1688 factory default for my test chip
  if (devStatus1 == 0) {
      // turn on the DMP, now that it's ready
      Serial.println(F("Enabling DMP1..."));
      accelgyroIC1.setDMPEnabled(true);

      // enable Arduino interrupt detection
      Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
      attachInterrupt(0, dmpDataReady1, RISING);
      mpuIntStatus1 = accelgyroIC1.getIntStatus();

      // set our DMP Ready flag so the main loop() function knows it's okay to use it
      Serial.println(F("DMP1 ready! Waiting for first interrupt..."));
      dmpReady1 = true;

      // get expected DMP packet size for later comparison
      packetSize1 = accelgyroIC1.dmpGetFIFOPacketSize();
  } else {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      Serial.print(F("DMP1 Initialization failed (code "));
      Serial.print(devStatus1);
      Serial.println(F(")"));
  }
  if (devStatus2 == 0) {
      // turn on the DMP, now that it's ready
      Serial.println(F("Enabling DMP2..."));
      accelgyroIC2.setDMPEnabled(true);

      // enable Arduino interrupt detection
      Serial.println(F("Enabling interrupt detection (Arduino external interrupt 1)..."));
      attachInterrupt(1, dmpDataReady2, RISING);
      mpuIntStatus2 = accelgyroIC2.getIntStatus();

      // set our DMP Ready flag so the main loop() function knows it's okay to use it
      Serial.println(F("DMP2 ready! Waiting for first interrupt..."));
      dmpReady2 = true;

      // get expected DMP packet size for later comparison
      packetSize2 = accelgyroIC2.dmpGetFIFOPacketSize();
  } else {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      Serial.print(F("DMP2 Initialization failed (code "));
      Serial.print(devStatus2);
      Serial.println(F(")"));
  }

//========================
//=== joystick
//========================
  pinMode(JS_key,INPUT_PULLUP);
//========================
//=== infrared receivers
//========================
/*Nothing required to setup IR*/
  //pinMode(IR_r1,HIGH);//pull-up resistor for input
  //pinMode(IR_r2,HIGH);
//========================
//=== motor
//========================
  pinMode(motor0,OUTPUT);
  pinMode(motor1,OUTPUT);
  pinMode(motor2,OUTPUT);
  stepper.setMaxSpeed(1000.0);
  stepper.setAcceleration(1000.0);
}
// ================================================================
// ===             User Functions Reference                     ===
// ================================================================
/* Basic functions */
#define K_GY 100
#define K_IR 5
void move0(long dist);
void move1(long dist);
void move2(long dist);
int readIR1();
int readIR2();
float readGyro1();
float readGyro2();
void readJS(int &x, int &y);
/* Operation functions */
#define Eo 10 // orientation angle tolerance
#define Ed 600
bool adjustOrientation1();
bool liftTo1();
bool compressLF();
bool adjustOrientation2();
bool liftTo2();
bool compressRT();
void one_flipping();
int commandStepper(); // return the command code
// 0: compress left
// 1: compress right
// 2: 2 to 1
// 3: 1 to 2
void moveRobot_w_JS();
// ================================================================
// ===                   Main Loop                              ===
// ================================================================

void loop() {
  int key;
  while(1)
  {
    key = digitalRead(JS_key);
    if (key = LOW)
    {
      one_flipping();
    }
    else
    moveRobot_w_JS();
  }
  
}

// ================================================================
// ===             User Functions Definition                    ===
// ================================================================
//========================
//=== Operation Functions
//========================
void moveRobot_w_JS()
{
  int command;
  command = commandStepper();
  switch (command)
  {
    case 1:
        Serial.print("\n");
        Serial.print("compressing right");
        Serial.print("\n");
        Serial.print("stepper0: ");
        move0(8000);
        break;
    case 0:
        Serial.print("\n");
        Serial.print("compressing left");
        Serial.print("\n");
        Serial.print("stepper0: ");
        move0(-8000);
        break;
    case 2:
        Serial.print("\n");
        Serial.print("moving 1 to 2");
        Serial.print("\n");
        Serial.print("stepper2: ");
        move1(+2000);
        Serial.print("   stepper1: ");
        move2(-2000);
        break;
    case 3:
        Serial.print("\n");
        Serial.print("moving 2 to 1");
        Serial.print("\n");
        Serial.print("stepper1: ");
        move2(2000);
        Serial.print("   stepper2: ");
        move1(-2000);
        break;
    default:
        break;
  }
}

int commandStepper()
{
  int x,y;
  readJS(x,y);
  if(x<-450)
    if(y>-20 && y<20)
    {
      return 3;    
    }
  if(x>250)
    if(y>-20 && y<20)
    {
      return 2;
    }
  if(y<-450)
    if(x>-20 && x<20)
    {
      return 0;
    }
  if(y>450)
    {
      return 1;
    }
  return 10;
}



void one_flipping()
{
  Serial.println("flipping automatively...");
  adjustOrientation1();
  liftTo1();
  compressLF();
  adjustOrientation2();
  liftTo2();
  compressRT();
}

bool adjustOrientation1()
{
  float angle,err;
  long dist;
  do
  {
    angle = readGyro1();
    err = angle - 0; // Note the direction. This might be:
    //err = 0 - angle; // which depends on how to install the gyro
    dist = K_GY*err;
    move0(dist); 
  }while(err>Eo);
  return true;
}

bool liftTo1()
{
  long led;
  long dist;
  led = readIR1();
  while(led<Ed)
  {
    dist = led*K_IR;
    move2(-dist); // release wire from motor2
    move1(dist); // tighten wire from motor1
    led = readIR1();
  }
  return true;
}

bool compressLF()
{
  float angle;
  move0(10000);
  do
  {
    move0(1000);
    angle = readGyro1();
  }while(abs(angle)<Eo);
  return true;
}

bool adjustOrientation2()
{
  return true;
}

bool liftTo2()
{
  long led;
  long dist;
  led = readIR2();
  while(led<Ed)
  {
    dist = K_IR*led;
    move1(-dist); // release wire from motor1
    move2(dist); // tighten wire from motor2
    led = readIR2();
  }
  return true;
}

bool compressRT()
{
  float angle;
  move0(-10000);
  do
  {
    move0(-1000);
    angle = readGyro1();
  }while(abs(angle)<Eo);
  return true;
}
//========================
//=== Basic Functions
//========================
void readJS(int &x, int &y)
{
  
  x = analogRead(JS_x)-512;
  y = analogRead(JS_y)-512;
}

void move0(long dist)
{
  digitalWrite(motor0,HIGH); // HIGH enabled
  digitalWrite(motor1,HIGH); // LOW enabled
  digitalWrite(motor2,HIGH);
  Serial.print("Moving with distance: ");
  Serial.print(dist);
  stepper.move(dist);
  while(stepper.run());
  digitalWrite(motor0,LOW);
}

void move1(long dist)
{
  digitalWrite(motor0,LOW);
  digitalWrite(motor1,LOW);
  digitalWrite(motor2,HIGH);
  Serial.print("Moving with distance: ");
  Serial.print(dist);
  stepper.move(dist);
  while(stepper.run());
  digitalWrite(motor1,HIGH);
}

void move2(long dist)
{
  digitalWrite(motor0,LOW);
  digitalWrite(motor1,HIGH);
  digitalWrite(motor2,LOW);
  Serial.print("Moving with distance: ");
  Serial.print(dist);
  stepper.move(dist);
  while(stepper.run());
  digitalWrite(motor2,HIGH);
}

int readIR1()
{
  int dist1,dist2,err,E;
  E = 15;
  do // naive noise filter
  {
    dist1 = analogRead(IR_r1);
    delay(5);
    dist2 = analogRead(IR_r1);
    err = dist1 - dist2;
  }while(abs(err)>E);
  Serial.print("IR receiver1: ");
  Serial.print(dist1);  
  Serial.print("\n");
  return dist1;
}

int readIR2()
{
  int dist1,dist2,err,E;
  E = 15;
  do
  {
    dist1 = analogRead(IR_r2);
    delay(5);
    dist2 = analogRead(IR_r2);
    err = dist1 - dist2;
  }while(abs(err)>E);
  Serial.print("IR receiver2: ");
  Serial.print(dist1);  
  Serial.print("\n");
  return dist1;
}

float readGyro1()
{
  // read the pitch (x-axil)
  float pitch1=0,pitch2=0,err=100,E=10;
  // if programming failed, don't try to do anything
  if (!dmpReady1) return -1000;
  do
  { 
    pitch2 = pitch1;
    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt1 && fifoCount1 < packetSize1);
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt1 = false;
    mpuIntStatus1 = accelgyroIC1.getIntStatus();
    // get current FIFO count
    fifoCount1 = accelgyroIC1.getFIFOCount();
    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus1 & 0x10) || fifoCount1 == 1024) {
        // reset so we can continue cleanly
        accelgyroIC1.resetFIFO();
        Serial.println(F("FIFO-1 overflow!"));
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus1 & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount1 < packetSize1) fifoCount1 = accelgyroIC1.getFIFOCount();
  
        // read a packet from FIFO
        accelgyroIC1.getFIFOBytes(fifoBuffer1, packetSize1);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount1 -= packetSize1;
        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            accelgyroIC1.dmpGetQuaternion(&q, fifoBuffer1);
            accelgyroIC1.dmpGetGravity(&gravity, &q);
            accelgyroIC1.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("MPU1\t");
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);
            pitch1 = ypr[1] * 180/M_PI;
            err = pitch1 -pitch2;
        #endif
    }
  }while(abs(err)>E);
  return pitch1; 
}

float readGyro2()
{
  // read the pitch (x-axil)
  float pitch1=0,pitch2=0,err=100,E=10;
  // if programming failed, don't try to do anything
  if (!dmpReady2) return -1000;
  do
  { 
    pitch2 = pitch1;
    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt2 && fifoCount2 < packetSize2);
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt2 = false;
    mpuIntStatus2 = accelgyroIC2.getIntStatus();

    // get current FIFO count
    fifoCount2 = accelgyroIC2.getFIFOCount();
    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus2 & 0x10) || fifoCount2 == 1024) {
        // reset so we can continue cleanly
        accelgyroIC2.resetFIFO();
        Serial.println(F("FIFO-2 overflow!"));
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus2 & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount2 < packetSize2) fifoCount2 = accelgyroIC2.getFIFOCount();
        // read a packet from FIFO
        accelgyroIC2.getFIFOBytes(fifoBuffer2, packetSize2);
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount2 -= packetSize2;
        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            accelgyroIC2.dmpGetQuaternion(&q, fifoBuffer2);
            accelgyroIC2.dmpGetGravity(&gravity, &q);
            accelgyroIC2.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("MPU2\t");
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);
            pitch1 = ypr[1] * 180/M_PI;
            err = pitch1 -pitch2;
        #endif
    }
  }while(abs(err)>E);
  return pitch1; 
}

#include <Adafruit_PWMServoDriver.h>


Adafruit_PWMServoDriver sb = Adafruit_PWMServoDriver(0x40);
#include "MPU9250.h"
MPU9250 IMU(Wire1,0x68);

#define CONSTRAIN(x,a,b) (x < a ? a : (x > b ? b : x))
#define MAP(x, a, b, c, d) (CONSTRAIN((c + (float(d) - c) * (x - a) / (float(b) - a)),c,d))

// Defining servo motor operating ranges
#define OPPOSE_MIN 140
#define OPPOSE_MAX 240
#define OPPOSE_HOME 100

#define FE_MIN 260 //flex
#define FE_MAX 480 //extend
#define FE_HOME 390 //380

#define UR_MIN 320 //radial 320
#define UR_MAX 510 //ulnar 490
#define UR_HOME 440

#define SP_MIN 120 //pronate 180
#define SP_MAX 480 //supinate 430
#define SP_HOME 340 //280

#define INDEX_MAX 1000//2500
#define MIDDLE_MAX 1000
#define RING_MAX 1000
#define THUMB_MAX 1000

#define SampleTime 10 // 100 Hz - How fast the encoders are read
#define UpdateTime 20 // 50 Hz - How fast the motors are updating
#define PrintTime 100 // 10 Hz 1- How fast we print to serial
#define IMUSampleTime 50 // 100 Hz - How fast we sample the IMU in ms

unsigned long CurrentTime;
unsigned long LastSampleTime;
unsigned long LastUpdateTime;
unsigned long LastPrintTime;

const uint8_t JointPins[8] = {7, 4, 5, 6, 0, 1, 2, 3};
const uint8_t FingerFlx[4] = {15, 8, 11, 12};
const uint8_t FingerExt[4] = {14, 9, 10, 13};
uint8_t dir[4] = {0, 0, 0, 0};

// index, middle, ring, thumb, oppose, flex, ulnar, suppination
const int16_t HomePos[8] = {0, 0, 0, 0, OPPOSE_HOME, FE_HOME, UR_HOME, SP_HOME};
const int16_t MinPos[8] = { -INDEX_MAX, -MIDDLE_MAX, -RING_MAX, -THUMB_MAX, OPPOSE_MIN, FE_MIN, UR_MIN, SP_MIN};
const int16_t MaxPos[8] = {INDEX_MAX, MIDDLE_MAX, RING_MAX, THUMB_MAX, OPPOSE_MAX, FE_MAX, UR_MAX, SP_MAX};
const uint8_t Error = 3;

uint8_t DataIn[8];
int16_t  DesiredPositions[8];
int16_t PrevPositions[8];
int16_t  JointPos[8];
uint16_t JointVel[8];

uint16_t minV = 500;
uint16_t maxV = 4095;
uint16_t ROI = 100;
byte homed = 0x00;

// Encoder pins: A is the interrupt pin, B is the counter
const uint8_t INencoderA = 10; // index finger encoder A
const uint8_t INencoderB = 11; // index finger encoder B
const uint8_t MIencoderA = 8; // middle finger encoder A
const uint8_t MIencoderB = 9; // middle finger encoder B
const uint8_t RIencoderA = 4; // ring finger encoder A
const uint8_t RIencoderB = 5; // ring finger encoder B
const uint8_t THencoderA = 6; // thumb finger encoder A
const uint8_t THencoderB = 7; // thumb finger encoder B

volatile int16_t enc0Count = JointPos[0];
volatile int16_t enc1Count = JointPos[1];
volatile int16_t enc2Count = JointPos[2];
volatile int16_t enc3Count = JointPos[3];

//int16_t OneFPinch[5] = {2000, 0, 0, 500, OPPOSE_MAX};
//int16_t TwoFPinch[5] = {2000, 2000, 0, 1500, OPPOSE_MAX};
//int16_t Power[5] = {2000, 2000, 2000, 2000, OPPOSE_MAX};
int16_t KeyPinch[5] = {2000, 2000, 2000, 1000, OPPOSE_MAX};
int16_t Pistol[5] = {0,2000, 2000, 0, 0};
int16_t Point[5] = {0, 2000, 2000, 2000, OPPOSE_MAX};
int16_t MidF[5] = {2000, 0, 2000, 2000, OPPOSE_MAX};
int16_t ThumbUp[5] = {2000, 2000, 2000, 0, 0};

int16_t OneFPinch[5] = {1000, 0, 0, 500, 0};
int16_t TwoFPinch[5] = {1000, 1000, 0, 575, 0};
int16_t Power[5] = {1000, 1000, 1000, 650, 0};

float anglesT_1[] = {0,0,0};
float anglesT[] = {0,0,0};
bool initIMUCounter = true;
float previousTimeIMU = 0;
float CurrentTimeIMU = 0;
float theta_pitch = 0;
float theta_roll = 0;
float theta_yaw = 0;
float dt = 0.0;
bool armStationary = true;
int status;
uint8_t gripID = 0;
uint8_t handCloseSpeed = 10;

void setup() {
  sb.begin();
  sb.setPWMFreq(50);
  delay(10);

  // Defining an IMU object
  IMU.begin();
//  IMU.setSrd(9); // Setting IMU data outpur rate to 10Hz (1000 / (1 + 99))
//  IMU.enableDataReadyInterrupt(); // Enabling the interrupt pin on the MPU9250
//  pinMode(22,INPUT); // attaching the interrupt to microcontroller pin 2
//  attachInterrupt(22,getIMU,RISING); // Calling getIMU every time the interrupt fires
  //NVIC_SET_PRIORITY(IRQ_GPIO6789, 255); //Setting the IMU interrupt priority to be the lowest
  
  // Homing all joints and their desired values
  for (uint8_t i = 0; i < 8 ; i++) {
    JointPos[i] = HomePos[i]; 
    JointVel[i] = 0;
    DesiredPositions[i] = HomePos[i];
    PrevPositions[i] = HomePos[i];
  }
  enc0Count = JointPos[0];
  enc1Count = JointPos[1];
  enc2Count = JointPos[2];
  enc3Count = JointPos[3];

  Serial.begin(115200);
  while (!Serial);

  // Encoder pins: A is the interrupt pin, B is the counter
  pinMode(INencoderA, INPUT); pinMode(INencoderB, INPUT);
  pinMode(MIencoderA, INPUT); pinMode(MIencoderB, INPUT);
  pinMode(RIencoderA, INPUT); pinMode(RIencoderB, INPUT);
  pinMode(THencoderA, INPUT); pinMode(THencoderB, INPUT);
  
  // On a rising edge, increment the number of motor turns
  attachInterrupt(INencoderA, tick0, RISING);
  attachInterrupt(MIencoderA, tick1, RISING);
  attachInterrupt(RIencoderA, tick2, RISING);
  attachInterrupt(THencoderA, tick3, RISING);
}

void loop() {
  CurrentTime = millis();
  GetCommand(); //Sets the DesiredPositions array
  GetJointPos(); // Sets the JointPos array, specifying the current position of all joint positions
  Move(); // commands motor motion
  //Print(); // prints output to serial
  getArmOrientation();
}

float CalibrateAccelerometer(float * accelData){
   float ax_scale = 1.004;
   float ax_bias = 2.1;
   float ay_scale = 1.0004;
   float ay_bias = 2.55;
   float az_scale = 1.1;
   float az_bias = 2.1;
   accelData[0] = (-accelData[0] - ax_bias) / ax_scale;
   accelData[1] = (-accelData[1] - ay_bias) / ay_scale;
   accelData[2] = (accelData[2] - az_bias) / az_scale;
}

float OrientationFromAccelerometer(float * accelOrientation){
  float ay = IMU.getAccelX_mss();
  float ax = IMU.getAccelY_mss();
  float az = IMU.getAccelZ_mss();
  float accelData[3]={ax, ay, az};
  CalibrateAccelerometer(accelData);
  ax = accelData[0];
  ay = accelData[1];
  az = accelData[2];
  float theta_y = -((atan2(az, ay) * (180 / PI)) + 90);
  float theta_x = -((atan2(-az, ax) * (180 / PI)) - 90);
  float theta_z = atan2(ay, ax) * (180 / PI);
  accelOrientation[0] = theta_x;
  accelOrientation[1] = theta_y;
  accelOrientation[2] = theta_z;
}

float CalibrateMagnetometer(float * magCalibrated){
  float mx = IMU.getMagX_uT();
  float my = IMU.getMagY_uT();
  float mz = IMU.getMagZ_uT();
  float A[3][3] = {{0.9932,   -0.0120,    0.0243},
                   {-0.0120,    1.0106,    0.0184},
                   {0.0243,    0.0184,    0.9974}};
//  float b[]= {57.4634, 78.6249, 9.6638};
  float b[]= {52.8039,   92.1576,  -20.8409};
  float mag_uncalib[] = {mx, my, mz};
  float mag_minus_bias[] = {0,0,0};
  for (int i = 0; i < 3; i++) {
    mag_minus_bias[i] =  mag_uncalib[i] - b[i];
  }
  float mag_calib[] = {0,0,0};
  for (int i = 0; i < 3; i++) {
    float sum = 0;
    for (int j = 0; j < 3; j++) {
      sum = sum + A[j][i]*mag_minus_bias[j];
    }
    mag_calib[i] = sum;
  }
  float memory = mag_calib[1];
  magCalibrated[1] = mag_calib[0];
  magCalibrated[0] = mag_calib[1];
  magCalibrated[2] = -mag_calib[2];
}
  
float getArmOrientation(){
  CurrentTimeIMU = micros()/1000000.0; // in seconds
//  CurrentTimeIMU = millis();
  if (initIMUCounter){
    previousTimeIMU = CurrentTimeIMU;
    initIMUCounter = false;
    }
  dt = (CurrentTimeIMU - previousTimeIMU);// in seconds
  if (dt >= IMUSampleTime/1000.0){
//      Serial.print(dt, 6);
//      Serial.print('\t');
//      Serial.println(IMUSampleTime/1000.0, 6);
    previousTimeIMU = CurrentTimeIMU;
    IMU.readSensor();
    float gy = IMU.getGyroX_rads();
    float gx = IMU.getGyroY_rads();
    float gz = IMU.getGyroZ_rads();
    // Gyro Angles
    float gyroOrientation[3]={0,0,0};
    float gyroData[3]={gx,gy,gz};
    for (int j = 0; j < 3; j++) {
        gyroOrientation[j] = gyroData[j]*dt*(180/PI);
      }
   
    // Magnetometer Angles
    float magCalibrated[3]={0,0,0};
    CalibrateMagnetometer(magCalibrated);
    float mx = magCalibrated[0];
    float my = magCalibrated[1];
    float mz = magCalibrated[2];
  
    // Accelerometer Angles
    float accelOrientation[3]={0,0,0};
    OrientationFromAccelerometer(accelOrientation);
  
    // Complementary Filter
    float G = 0.8;
    float A = 0.2;
    
    for (int j = 0; j < 3; j++) {
      anglesT[j] = (anglesT_1[j] + gyroOrientation[j])*G + A*accelOrientation[j];
      anglesT_1[j] = anglesT[j];
    }
  
    theta_pitch = anglesT[1]* (PI/180.0);
    theta_roll = anglesT[0] * (PI/180.0);
  
    float x_heading = mx * cos(theta_roll) + mz * (sin(theta_roll));
    float y_heading = mx * (sin(theta_roll) * sin(theta_pitch)) + my * cos(theta_pitch) - mz * (
                  cos(theta_roll) * sin(theta_pitch));
    if (theta_yaw !=0){
      theta_yaw = 0.9*theta_yaw + 0.1*(atan2(y_heading, x_heading) * (180 / PI));
    }
    else{
      theta_yaw = atan2(y_heading, x_heading) * (180 / PI);
      }
    
    theta_pitch = theta_pitch * (180.0/PI);
    theta_roll = theta_roll * (180.0/PI);
  }
}

void getIMU(){ 
  // read the sensor
  IMU.readSensor();
//   display the data
  Serial.print(IMU.getAccelX_mss(),3);
  Serial.print("\t");
  Serial.print(IMU.getAccelY_mss(),3);
  Serial.print("\t");
  Serial.print(IMU.getAccelZ_mss(),3);
  Serial.print("\t");
  Serial.print(IMU.getGyroX_rads(),3);
  Serial.print("\t");
  Serial.print(IMU.getGyroY_rads(),3);
  Serial.print("\t");
  Serial.print(IMU.getGyroZ_rads(),3);
  Serial.print("\t");
  Serial.print(IMU.getMagX_uT(),3);
  Serial.print("\t");
  Serial.print(IMU.getMagY_uT(),3);
  Serial.print("\t");
  Serial.print(IMU.getMagZ_uT(),3);
  Serial.print("\t");
  Serial.println(IMU.getTemperature_C(),3);
}

void Move() {
  int jointsToMove = 0;
  if (CurrentTime - LastUpdateTime >= UpdateTime) {
    for (uint8_t i = 0; i < 8; i++) {
      if (i < 4) {
        dir[i] = JointPos[i] - DesiredPositions[i] < 1 ? 1 : 0;

        uint16_t dev = abs(JointPos[i] - DesiredPositions[i]);
        if (dev > Error){
          JointVel[i] = MAP(dev, 0, ROI, minV, maxV);
          jointsToMove++;
        }
        else
          JointVel[i] = 0;

        if (JointPos[i] != DesiredPositions[i]){
          sb.setPWM(FingerFlx[i], 0, maxV * dir[i]);
          sb.setPWM(FingerExt[i], 0, maxV * !dir[i]);
          sb.setPWM(JointPins[i], 0, JointVel[i]);
        }
      }
      else {
      if (JointPos[i] != DesiredPositions[i]){
          sb.setPWM(JointPins[i], 0, JointPos[i]);
          if (abs(JointPos[i] - DesiredPositions[i]) > 10)
            jointsToMove++;
        }
      }
    }
    delayMicroseconds(500);
    LastUpdateTime = CurrentTime;
    if (jointsToMove == 0){
        armStationary = true;
      }
     else{
      armStationary = false;
      }
  }
}

void GetJointPos() {
  if (CurrentTime - LastSampleTime >= SampleTime) {
    for (uint8_t i = 0; i < 4; i++)
      JointPos[i] = getEncoderVal(i);
    for (uint8_t i = 4; i < 8; i++)
      JointPos[i] = JointPos[i]*0.9 + DesiredPositions[i]*0.1;
    //JointPos[i] = JointPos[i]*0.90 + DesiredPositions[i]*0.1;
      
    LastSampleTime = CurrentTime;

    homed = 0x00; // specifies if the joint is at the home position
    for (uint8_t i = 0; i < 8; i++) {
      if (abs(JointPos[i] - HomePos[i]) <= Error)
        homed = homed | (0x01 << i);
    }
  }
}

void Print() {
  if (CurrentTime - LastPrintTime >= PrintTime) {
    for (uint8_t i = 0; i < 8; i++) {
      Serial.print(DesiredPositions[i]); Serial.print(F(","));
      Serial.print(JointPos[i]); Serial.print(F(","));
    }
    Serial.println(homed, BIN);
    LastPrintTime = CurrentTime;
  }
}

void GetCommand() {
  if (Serial.available()) {
    
    uint8_t mode = Serial.read();

    if (mode == 'r'){
      Serial.print(theta_pitch,6);
      Serial.print("\t");
      Serial.print(theta_roll,6);
      Serial.print("\t");
      Serial.println(theta_yaw,6);
    }

    //close hand
    else if (mode == 'c'){
      Serial.print(theta_pitch,6);
      Serial.print("\t");
      Serial.print(theta_roll,6);
      Serial.print("\t");
      Serial.println(theta_yaw,6);
      for (uint8_t i = 0; i <= gripID; i++){
          DesiredPositions[i] = CONSTRAIN(DesiredPositions[i] + handCloseSpeed,0,MaxPos[i]);
        }
      }

    //open hand
    else if (mode == 'o'){
      Serial.print(theta_pitch,6);
      Serial.print("\t");
      Serial.print(theta_roll,6);
      Serial.print("\t");
      Serial.println(theta_yaw,6);
      for (uint8_t i = 0; i <= gripID; i++){
          DesiredPositions[i] = CONSTRAIN(DesiredPositions[i] - handCloseSpeed,0,MaxPos[i]);
        }
      }

    else if (mode == 'j') {
      uint8_t jointID  = Serial.parseInt();
      int16_t StepTo = Serial.parseInt();
      
      for (uint8_t i = 0; i < 8; i++)
          PrevPositions[i] = DesiredPositions[i];
      // Serial.println();Serial.println(PrevPositions[jointID]);
      
      // Limiting the commanded joint position to the values defined in the min and max positions
      DesiredPositions[jointID] = CONSTRAIN(StepTo, MinPos[jointID], MaxPos[jointID]);
      // Serial.println(DesiredPositions[jointID]);

      // Oppose
      if (jointID == 4)
          DesiredPositions[3] = getEncoderVal(3) + OpposeFactor;

      // Wrist flex/extend
      else if (jointID == 5) {
        for (uint8_t i = 0; i < 4; i++)
          DesiredPositions[i] = getEncoderVal(i) - FlxExtFactor(i);
      }
    }
    
    else if (mode == 'g') {
      gripID  = Serial.parseInt();
      uint8_t apperture = Serial.parseInt();
      int16_t *pointToArray;
      switch (gripID) {
        case 0:
          pointToArray = OneFPinch;
          break;
        case 1:
          pointToArray = TwoFPinch;
          break;
        case 2:
          pointToArray = Power;
          break;
        case 3:
          pointToArray = KeyPinch;
          break;
        case 4:
          pointToArray = Pistol;
          break;
        case 5:
          pointToArray = Point;
          break;
        case 6:
          pointToArray = MidF;
          break;
        case 7:
          pointToArray = ThumbUp;
          break;
      }
      for (uint8_t i = 0; i < 8; i++)
          PrevPositions[i] = DesiredPositions[i];
      
      for (uint8_t i = 0; i < 4; i ++){
        if (DesiredPositions[5] != HomePos[5])
        {
          if (i == 3)
            DesiredPositions[i] = CONSTRAIN(*(pointToArray + i) + DesiredPositions[i],MinPos[i],MaxPos[i]);
          else
            DesiredPositions[i] = CONSTRAIN(*(pointToArray + i)* apperture/255.0 + DesiredPositions[i],MinPos[i],MaxPos[i]);
          }
          
        else
        {
          if (i == 3)
            DesiredPositions[i] = CONSTRAIN(*(pointToArray + i),MinPos[i],MaxPos[i]);
          else
            DesiredPositions[i] = CONSTRAIN(*(pointToArray + i) * apperture/255.0,MinPos[i],MaxPos[i]);
        }
      }
      //DesiredPositions[4] = MaxPos[4];
//        
//      if (PrevPositions[4] != DesiredPositions[4])
//          DesiredPositions[3] = getEncoderVal(3) + OpposeFactor();
        
    }

    else if (mode == 'w') {
      uint8_t fe  = Serial.parseInt();
      uint8_t ur  = Serial.parseInt();
      uint8_t sp  = Serial.parseInt();
      
      for (uint8_t i = 0; i < 8; i++)
          PrevPositions[i] = DesiredPositions[i];
          
      DesiredPositions[5] = MAP(fe,0,255,MinPos[5],MaxPos[5]);
      DesiredPositions[6] = MAP(ur,0,255,MinPos[6],MaxPos[6]);
      DesiredPositions[7] = MAP(sp,0,255,MinPos[7],MaxPos[7]);
    
      if (PrevPositions[5] != DesiredPositions[5]){
          for (uint8_t i = 0; i < 4; i++)
            DesiredPositions[i] = getEncoderVal(i) - FlxExtFactor(i);
      } 
    }
    
    else if (mode == 'h') {
      for (uint8_t i = 0; i < 8; i ++)
        DesiredPositions[i] = HomePos[i];
    }

    while (Serial.available())
      Serial.read();
  }
}

int16_t getEncoderVal(uint8_t jointID) {
  switch (jointID) {
    case 0:
      return enc0Count;
    case 1:
      return enc1Count;
    case 2:
      return enc2Count;
    case 3:
      return enc3Count;
  }
}

int16_t FlxExtFactor(uint8_t jointID) {
  int16_t Delta = DesiredPositions[5] - PrevPositions[5];
  int16_t minDev = MinPos[5] - MaxPos[5];
  int16_t maxDev = MaxPos[5] - MinPos[5];

  //  Serial.println(Delta);Serial.println(minDev);Serial.println(maxDev);
  int16_t val = 0;
  if (jointID == 3){
    val = MAP(Delta, minDev, maxDev, -800, 800);
    //Serial.println(val);
    return val;
  }
  else {
//    val = MAP(Delta, minDev, maxDev, -5, 5);
    val = MAP(Delta, minDev, maxDev, -500, 500);
    //Serial.println(val);
    return val;
  }
}

int16_t OpposeFactor() {
  int16_t Delta = DesiredPositions[4] - PrevPositions[4];
  int16_t minDev = MinPos[4] - MaxPos[4];
  int16_t maxDev = MaxPos[4] - MinPos[4];
  int16_t val = MAP(Delta, minDev, maxDev, -200, 200);
//  Serial.println(Delta);Serial.println(minDev);Serial.println(maxDev);Serial.println(val);
  return val;
}

int16_t tick0() {
  return digitalRead(INencoderB) ? enc0Count-- : enc0Count++;
}
int16_t tick1() {
  return digitalRead(MIencoderB) ? enc1Count++ : enc1Count--;
}
int16_t tick2() {
  return digitalRead(RIencoderB) ? enc2Count-- : enc2Count++;
}
int16_t tick3() {
  return digitalRead(THencoderB) ? enc3Count++ : enc3Count--;
}

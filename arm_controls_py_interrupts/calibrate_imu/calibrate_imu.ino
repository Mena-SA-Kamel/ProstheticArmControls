#include "MPU9250.h"
MPU9250 IMU(Wire1,0x68);


#define SampleTime 10 // 100 Hz - How fast the encoders are read
#define UpdateTime 20 // 50 Hz - How fast the motors are updating
#define PrintTime 100 // 10 Hz 1- How fast we print to serial
#define IMUSampleTime 50 // 100 Hz - How fast we sample the IMU in ms

unsigned long CurrentTime;
unsigned long LastSampleTime;
unsigned long LastUpdateTime;
unsigned long LastPrintTime;

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

void setup() {
  // Defining an IMU object
  IMU.begin();
}

void loop() {
  getIMU();
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
  float A[3][3] = {{0.9953, -0.0146, 0.0201},
                   {-0.0146, 1.0135, 0.0253},
                   {0.0201, 0.0253, 0.9926}};
  float b[]= {57.4634, 78.6249, 9.6638};
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
  if (Serial.available()) {
    uint8_t mode = Serial.read();

    if (mode == 'r'){
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
  }
}

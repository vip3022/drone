#include <Wire.h>
#include <Servo.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(8,9);
const byte rxADDR[6] = "00001"; // it could be any 5 digit number
float value[4];

const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float roll, pitch, yaw;
float rollAcc, pitchAcc, rollGyro, pitchGyro;
float totalMagnitude;

float pkroll[] = {0.5,0,0,0.01};
float pkpriroll[] = {0.5,0,0,0.01};
float pk1roll[] = {0.5,0,0,0.01};
float pkpitch[] = {0.5,0,0,0.01};
float pkpripitch[] = {0.5,0,0,0.01};
float pk1pitch[] = {0.5,0,0,0.01};
float deltaT = 0.04;
float xkroll[] = {0,0};
float xkpriroll[] = {0,0};
float xk1roll[] = {0,0};
float xkpitch[] = {0,0};
float xkpripitch[] = {0,0};
float xk1pitch[] = {0,0};
float k[] = {0,0};
float phi[] = {1,deltaT,0,1};
float psi[] = {deltaT,0};
float I[] = {1,0,0,1};
float R = 0.03;
float Q[] = {0.0002*0.0002,0,0,0.0001*0.0001};
float H[] = {1,0};
float S, ukRoll,zkRoll,ukPitch,zkPitch,nuRoll,nuPitch;

float pid_p_roll, pid_i_roll,pid_d_roll,pid_p_pitch,pid_i_pitch,pid_d_pitch,pid_p_yaw,pid_i_yaw,pid_d_yaw;

float elapsedTime,currentTime,previousTime;
float elapsedTimeG,currentTimeG,previousTimeG;
float PIDRoll,PIDPitch,PIDYaw;
float kp= 1.15;//
float ki=0;//0.04
float kd= 0;//18
float kpyaw= 0;//4
float kiyaw=0;//0.02
float kdyaw=0;
int rollDesired = 0;
int pitchDesired = 0;
float errorRoll,errorPitch,prev_error_roll,prev_error_pitch,errorYaw,prev_error_yaw;
float FR_motor,FL_motor,BR_motor,BL_motor;
float throttle;
float last_recieved_time,data_time;
float errorPi, errorRo;

int min_throttle=1050;
int max_throttle = 1800;
//
//int timenow,diff,oldtime;

Servo FR_esc;
Servo FL_esc;
Servo BR_esc;
Servo BL_esc;


void setup() {
  
  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(0,rxADDR);
  radio.setPALevel(RF24_PA_LOW); // If you want to save power use "RF24_PA_MIN" but keep in mind that reduces the module's range
  radio.setDataRate(RF24_1MBPS );
  radio.startListening();
  
  Serial.begin(19200);
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);         //end the transmission

  FR_esc.attach(6);
  FL_esc.attach(10);
  BL_esc.attach(5);
  BR_esc.attach(3);

  FR_esc.writeMicroseconds(1000);
  BR_esc.writeMicroseconds(1000);
  FL_esc.writeMicroseconds(1000);
  BL_esc.writeMicroseconds(1000);


delay(2000);

errorR();
errorP();

  
  
}

void loop() {
//  oldtime= timenow;
//  timenow = millis();
//  diff =oldtime-timenow;
//  Serial.println(diff);
  
  // === Read acceleromter data === //
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value

  totalMagnitude = sqrt((AccX*AccX) + (AccY*AccY) + (AccZ*AccZ));
  rollAcc = asin((float)AccX/totalMagnitude) * 57.296  ;
  pitchAcc = asin((float)AccY/totalMagnitude) * 57.296 ;
  
  // === Read gyroscope data === //
 
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  GyroX = ((Wire.read() << 8 | Wire.read()) / 131.0); 
  GyroY = ((Wire.read() << 8 | Wire.read()) / 131.0);
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0 ;

  
  previousTimeG = currentTimeG;
  currentTimeG = millis();
  elapsedTimeG = (currentTimeG - previousTimeG)/1000; 
  yaw = yaw + GyroZ*elapsedTimeG +0.00406*2;
 
  if(radio.available()){
    radio.read(&value,sizeof(value));
    last_recieved_time = millis();
    throttle = value[0];
  rollDesired = map(value[1],1000,2000,10,-10);
  pitchDesired = map(value[2],1000,2000,10,-10);
  }

 
 reset();
  
 kalmanRoll();
 kalmanPitch();
 pid();
 

}


void kalmanRoll(){
  
  ukRoll = GyroY/131.;
  zkRoll = rollAcc;
  
  xkpriroll[0] = phi[0]*xkroll[0] + phi[1]*xkroll[1] + psi[0]*ukRoll;
  xkpriroll[1] = phi[2]*xkroll[0] + phi[3]*xkroll[1] + psi[1]*ukRoll;

  pkpriroll[0] = (phi[0]*pkroll[0] + phi[1]*pkroll[2])*phi[0] + \
                 (phi[0]*pkroll[1] + phi[1]*pkroll[3])*phi[1] + Q[0];
              
  pkpriroll[1] = (phi[0]*pkroll[0] + phi[1]*pkroll[2])*phi[2] + \
                 (phi[0]*pkroll[1] + phi[1]*pkroll[3])*phi[3] + Q[1];
                 
  pkpriroll[2] = (phi[2]*pkroll[0] + phi[3]*pkroll[2])*phi[0] + \
                 (phi[2]*pkroll[1] + phi[3]*pkroll[3])*phi[1] + Q[2];

  pkpriroll[3] = (phi[2]*pkroll[0] + phi[3]*pkroll[2])*phi[2] + \
                 (phi[2]*pkroll[1] + phi[3]*pkroll[3])*phi[2] + Q[3];



  S = (H[0]*pkpriroll[0] +H[1]*pkpriroll[2])*H[0] + \
      (H[0]*pkpriroll[1] +H[1]*pkpriroll[3])*H[0]  + R;

  k[0] = (pkpriroll[0]*H[0] + pkpriroll[1]*H[1]) /S;
  k[1] = (pkpriroll[2]*H[0] + pkpriroll[3]*H[1]) /S; 

  nuRoll = zkRoll - (H[0]*xkpriroll[0] + \
                     H[1]*xkpriroll[1]);

  xk1roll[0] = xkpriroll[0] + k[0] * nuRoll;
  xk1roll[1] = xkpriroll[1] + k[1] * nuRoll;

  
  pk1roll[0] = (1-k[0]*H[0])*pkpriroll[0] + \
               (0-k[0]*H[1])*pkpriroll[2];
  pk1roll[1] = (1-k[0]*H[0])*pkpriroll[1] + \
               (0-k[0]*H[1])*pkpriroll[3];          
  pk1roll[2] = (1-k[1]*H[0])*pkpriroll[0] + \
               (0-k[1]*H[1])*pkpriroll[2];             
  pk1roll[3] = (1-k[1]*H[0])*pkpriroll[1] + \
               (0-k[1]*H[1])*pkpriroll[3]; 


  xkroll[0] = xk1roll[0];
  xkroll[1] = xk1roll[1];

  pkroll[0] = pk1roll[0];
  pkroll[1] = pk1roll[1];
  pkroll[2] = pk1roll[2];
  pkroll[3] = pk1roll[3];
  
  roll = xk1roll[0] - xk1roll[1]  -errorRo/500;
  //roll += pitch*sin(GyroZ*ellapsedTime*(PI/180));
  Serial.print(roll);
  Serial.print(",");

                             
}

void kalmanPitch(){

  
  ukPitch = GyroX/131.0;
  zkPitch = pitchAcc;
  
  xkpripitch[0] = phi[0]*xkpitch[0] + phi[1]*xkpitch[1] + psi[0]*ukPitch;
  xkpripitch[1] = phi[2]*xkpitch[0] + phi[3]*xkpitch[1] + psi[1]*ukPitch;

  pkpripitch[0] = (phi[0]*pkpitch[0] + phi[1]*pkpitch[2])*phi[0] + \
                 (phi[0]*pkpitch[1] + phi[1]*pkpitch[3])*phi[1] + Q[0];
              
  pkpripitch[1] = (phi[0]*pkpitch[0] + phi[1]*pkpitch[2])*phi[2] + \
                 (phi[0]*pkpitch[1] + phi[1]*pkpitch[3])*phi[3] + Q[1];
                 
  pkpripitch[2] = (phi[2]*pkpitch[0] + phi[3]*pkpitch[2])*phi[0] + \
                 (phi[2]*pkpitch[1] + phi[3]*pkpitch[3])*phi[1] + Q[2];

  pkpripitch[3] = (phi[2]*pkpitch[0] + phi[3]*pkpitch[2])*phi[2] + \
                 (phi[2]*pkpitch[1] + phi[3]*pkpitch[3])*phi[2] + Q[3];


  S = (H[0]*pkpripitch[0] +H[1]*pkpripitch[2])*H[0] + \
      (H[0]*pkpripitch[1] +H[1]*pkpripitch[3])*H[0]  + R;

  k[0] = (pkpripitch[0]*H[0] + pkpripitch[1]*H[1]) /S;
  k[1] = (pkpripitch[2]*H[0] + pkpripitch[3]*H[1]) /S; 

  nuPitch = zkPitch - (H[0]*xkpripitch[0] + \
                     H[1]*xkpripitch[1]);

  xk1pitch[0] = xkpripitch[0] + k[0] * nuPitch;
  xk1pitch[1] = xkpripitch[1] + k[1] * nuPitch;

  
  pk1pitch[0] = (1-k[0]*H[0])*pkpripitch[0] + \
               (0-k[0]*H[1])*pkpripitch[2];
  pk1pitch[1] = (1-k[0]*H[0])*pkpripitch[1] + \
               (0-k[0]*H[1])*pkpripitch[3];          
  pk1pitch[2] = (1-k[1]*H[0])*pkpripitch[0] + \
               (0-k[1]*H[1])*pkpripitch[2];             
  pk1pitch[3] = (1-k[1]*H[0])*pkpripitch[1] + \
               (0-k[1]*H[1])*pkpripitch[3]; 


  xkpitch[0] = xk1pitch[0];
  xkpitch[1] = xk1pitch[1];

  pkpitch[0] = pk1pitch[0];
  pkpitch[1] = pk1pitch[1];
  pkpitch[2] = pk1pitch[2];
  pkpitch[3] = pk1pitch[3];
  
  pitch = -(xk1pitch[0] - xk1pitch[1] ) - errorPi/500;  
  //pitch -= roll*sin(GyroZ*ellapsedTime*(PI/180));
  Serial.println(pitch);
 
}  

void pid(){
  previousTime = currentTime;
  currentTime = millis();
  elapsedTime = (currentTime - previousTime)/1000;

  //ROLLLLLLL
  
  errorRoll = roll - rollDesired;
  
  pid_p_roll = kp*errorRoll; 
  if (-3< errorRoll<3)pid_i_roll = pid_i_roll + ki*errorRoll;  
  pid_d_roll = (kd*(errorRoll - prev_error_roll))/elapsedTime;

  PIDRoll = pid_i_roll + pid_p_roll + pid_d_roll;

  if(PIDRoll < -400)PIDRoll = -400; 
  if(PIDRoll>400)PIDRoll= 400;

  //pitchhhhhhh


  errorPitch = pitch - pitchDesired;
  
  pid_p_pitch = kp*errorPitch; 
  if (-3< errorPitch<3)pid_i_pitch = pid_i_pitch + ki*errorPitch;
  pid_d_pitch = (kd*(errorPitch - prev_error_pitch))/elapsedTime;
  
  PIDPitch = pid_i_pitch + pid_p_pitch + pid_d_pitch;

  if(PIDPitch < -400)PIDPitch = -400; 
  if(PIDPitch>400)PIDPitch= 400;

  //yawwwwww
  errorYaw = yaw;
  
  pid_p_yaw = kp*errorYaw; 
  if (-3< errorYaw<3)pid_i_yaw = pid_i_yaw + ki*errorYaw;
  pid_d_yaw = (kd*(errorYaw - prev_error_yaw))/elapsedTime;
  
  PIDYaw = pid_i_yaw + pid_p_yaw + pid_d_yaw;

  if(PIDYaw < -400)PIDYaw = -400; 
  if(PIDYaw>400)PIDYaw= 400;
 
 
  //total PIDDDDDDD
  FR_motor = throttle - PIDRoll - PIDPitch;  + PIDYaw;  //cw
  FL_motor = throttle - PIDRoll + PIDPitch;  - PIDYaw;  //ccw
  BR_motor = throttle + PIDRoll - PIDPitch;  - PIDYaw;  //ccw
  BL_motor = throttle + PIDRoll + PIDPitch;  + PIDYaw;  //cw

  if(FR_motor<min_throttle)FR_motor = min_throttle;
  if(FR_motor>max_throttle)FR_motor = max_throttle;

  if(FL_motor<min_throttle)FL_motor = min_throttle;
  if(FL_motor>max_throttle)FL_motor = max_throttle;

  if(BR_motor<min_throttle)BR_motor = min_throttle;
  if(BR_motor>max_throttle)BR_motor = max_throttle;

  if(BL_motor<min_throttle)BL_motor = min_throttle;
  if(BL_motor>max_throttle)BL_motor = max_throttle;

  
  
  FR_esc.writeMicroseconds(FR_motor);
  BR_esc.writeMicroseconds(BR_motor);
  FL_esc.writeMicroseconds(FL_motor);
  BL_esc.writeMicroseconds(BL_motor);

//  Serial.print(FR_motor);
//  Serial.print(",");
//  Serial.print(FL_motor);
//  Serial.print(",");
//  Serial.print(BR_motor);
//  Serial.print(",");
//  Serial.println(BL_motor);
  
  prev_error_roll = errorRoll;
  prev_error_pitch = errorPitch;
  prev_error_yaw = errorYaw;  
}

void reset(){
  data_time = millis();
  if((data_time - last_recieved_time)/1000 > 0.6){
    throttle = 0;
    rollDesired = 0;
    pitchDesired = 0;
    min_throttle = 0;
    max_throttle=0;
  }
}

void errorP(){

  for(int i=0; i<500; i++){
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value

  totalMagnitude = sqrt((AccX*AccX) + (AccY*AccY) + (AccZ*AccZ));
  rollAcc = asin((float)AccX/totalMagnitude) * 57.296  ;
  pitchAcc = asin((float)AccY/totalMagnitude) * 57.296 ;
  
  // === Read gyroscope data === //
 
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  GyroX = ((Wire.read() << 8 | Wire.read()) / 131.0); 
  GyroY = ((Wire.read() << 8 | Wire.read()) / 131.0);
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0 ;

  
  previousTimeG = currentTimeG;
  currentTimeG = millis();
  elapsedTimeG = (currentTimeG - previousTimeG)/1000; 

  ukPitch = GyroX/131.0;
  zkPitch = pitchAcc;
  
  xkpripitch[0] = phi[0]*xkpitch[0] + phi[1]*xkpitch[1] + psi[0]*ukPitch;
  xkpripitch[1] = phi[2]*xkpitch[0] + phi[3]*xkpitch[1] + psi[1]*ukPitch;

  pkpripitch[0] = (phi[0]*pkpitch[0] + phi[1]*pkpitch[2])*phi[0] + \
                 (phi[0]*pkpitch[1] + phi[1]*pkpitch[3])*phi[1] + Q[0];
              
  pkpripitch[1] = (phi[0]*pkpitch[0] + phi[1]*pkpitch[2])*phi[2] + \
                 (phi[0]*pkpitch[1] + phi[1]*pkpitch[3])*phi[3] + Q[1];
                 
  pkpripitch[2] = (phi[2]*pkpitch[0] + phi[3]*pkpitch[2])*phi[0] + \
                 (phi[2]*pkpitch[1] + phi[3]*pkpitch[3])*phi[1] + Q[2];

  pkpripitch[3] = (phi[2]*pkpitch[0] + phi[3]*pkpitch[2])*phi[2] + \
                 (phi[2]*pkpitch[1] + phi[3]*pkpitch[3])*phi[2] + Q[3];


  S = (H[0]*pkpripitch[0] +H[1]*pkpripitch[2])*H[0] + \
      (H[0]*pkpripitch[1] +H[1]*pkpripitch[3])*H[0]  + R;

  k[0] = (pkpripitch[0]*H[0] + pkpripitch[1]*H[1]) /S;
  k[1] = (pkpripitch[2]*H[0] + pkpripitch[3]*H[1]) /S; 

  nuPitch = zkPitch - (H[0]*xkpripitch[0] + \
                     H[1]*xkpripitch[1]);

  xk1pitch[0] = xkpripitch[0] + k[0] * nuPitch;
  xk1pitch[1] = xkpripitch[1] + k[1] * nuPitch;

  
  pk1pitch[0] = (1-k[0]*H[0])*pkpripitch[0] + \
               (0-k[0]*H[1])*pkpripitch[2];
  pk1pitch[1] = (1-k[0]*H[0])*pkpripitch[1] + \
               (0-k[0]*H[1])*pkpripitch[3];          
  pk1pitch[2] = (1-k[1]*H[0])*pkpripitch[0] + \
               (0-k[1]*H[1])*pkpripitch[2];             
  pk1pitch[3] = (1-k[1]*H[0])*pkpripitch[1] + \
               (0-k[1]*H[1])*pkpripitch[3]; 


  xkpitch[0] = xk1pitch[0];
  xkpitch[1] = xk1pitch[1];

  pkpitch[0] = pk1pitch[0];
  pkpitch[1] = pk1pitch[1];
  pkpitch[2] = pk1pitch[2];
  pkpitch[3] = pk1pitch[3];
  
  pitch = -(xk1pitch[0] - xk1pitch[1]);

  

  errorPi = errorPi + pitch;
  }
  
}

void errorR(){

  for(int i=0; i<500; i++){
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value

  totalMagnitude = sqrt((AccX*AccX) + (AccY*AccY) + (AccZ*AccZ));
  rollAcc = asin((float)AccX/totalMagnitude) * 57.296  ;
  pitchAcc = asin((float)AccY/totalMagnitude) * 57.296 ;
  
  // === Read gyroscope data === //
 
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  GyroX = ((Wire.read() << 8 | Wire.read()) / 131.0); 
  GyroY = ((Wire.read() << 8 | Wire.read()) / 131.0);
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0 ;

  
  previousTimeG = currentTimeG;
  currentTimeG = millis();
  elapsedTimeG = (currentTimeG - previousTimeG)/1000; 

  ukRoll = GyroY/131.0;
  zkRoll = rollAcc;
  
  xkpriroll[0] = phi[0]*xkroll[0] + phi[1]*xkroll[1] + psi[0]*ukRoll;
  xkpriroll[1] = phi[2]*xkroll[0] + phi[3]*xkroll[1] + psi[1]*ukRoll;

  pkpriroll[0] = (phi[0]*pkroll[0] + phi[1]*pkroll[2])*phi[0] + \
                 (phi[0]*pkroll[1] + phi[1]*pkroll[3])*phi[1] + Q[0];
              
  pkpriroll[1] = (phi[0]*pkroll[0] + phi[1]*pkroll[2])*phi[2] + \
                 (phi[0]*pkroll[1] + phi[1]*pkroll[3])*phi[3] + Q[1];
                 
  pkpriroll[2] = (phi[2]*pkroll[0] + phi[3]*pkroll[2])*phi[0] + \
                 (phi[2]*pkroll[1] + phi[3]*pkroll[3])*phi[1] + Q[2];

  pkpriroll[3] = (phi[2]*pkroll[0] + phi[3]*pkroll[2])*phi[2] + \
                 (phi[2]*pkroll[1] + phi[3]*pkroll[3])*phi[2] + Q[3];



  S = (H[0]*pkpriroll[0] +H[1]*pkpriroll[2])*H[0] + \
      (H[0]*pkpriroll[1] +H[1]*pkpriroll[3])*H[0]  + R;

  k[0] = (pkpriroll[0]*H[0] + pkpriroll[1]*H[1]) /S;
  k[1] = (pkpriroll[2]*H[0] + pkpriroll[3]*H[1]) /S; 

  nuRoll = zkRoll - (H[0]*xkpriroll[0] + \
                     H[1]*xkpriroll[1]);

  xk1roll[0] = xkpriroll[0] + k[0] * nuRoll;
  xk1roll[1] = xkpriroll[1] + k[1] * nuRoll;

  
  pk1roll[0] = (1-k[0]*H[0])*pkpriroll[0] + \
               (0-k[0]*H[1])*pkpriroll[2];
  pk1roll[1] = (1-k[0]*H[0])*pkpriroll[1] + \
               (0-k[0]*H[1])*pkpriroll[3];          
  pk1roll[2] = (1-k[1]*H[0])*pkpriroll[0] + \
               (0-k[1]*H[1])*pkpriroll[2];             
  pk1roll[3] = (1-k[1]*H[0])*pkpriroll[1] + \
               (0-k[1]*H[1])*pkpriroll[3]; 


  xkroll[0] = xk1roll[0];
  xkroll[1] = xk1roll[1];

  pkroll[0] = pk1roll[0];
  pkroll[1] = pk1roll[1];
  pkroll[2] = pk1roll[2];
  pkroll[3] = pk1roll[3];
  
  roll = xk1roll[0] - xk1roll[1] ;

  errorRo = errorRo + roll;

  

  }
}

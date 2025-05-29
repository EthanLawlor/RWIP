/*
  Reaction Wheel Inverted Pendulum

  Cascade Loop - Speed of Motor as Outer, Angle From Vertical as Inner

  Ethan Lawlor - B00149346
  28/03/2025
*/

//#define DEBUG_ENCODER
//#define DEBUG_SPEED_PID
//#define DEBUG_ANGLE_PID

// MPU6050 Globals ---------------------------------------------------------------------------------
#include <Wire.h>          // Include the Wire library for I2C communication
#include <MPU6050.h>        // Include the MPU6050 library

MPU6050 mpu;
int16_t ax, ay, az, gx, gy, gz;
float gz_offset = 0;

struct mpuData {
  float theta_z;
  float omega_z;
};
// -------------------------------------------------------------------------------------------------

// Encoder Globals ---------------------------------------------------------------------------------
volatile long counter = 0;  
long lastCounter = 0;
unsigned long lastMillis = 0; 

struct encoderData {
  float rpm;
  bool direction;
};
// -------------------------------------------------------------------------------------------------

// Motor Globals -----------------------------------------------------------------------------------
#include <PWM.h>
#define Nidec_Frequency 30000
#define motorPWM_Pin 9
#define motorDIR_Pin 10
#define motorBrake_Pin 11
int motorPWM = 255;
bool motorDIR = 0;
// -------------------------------------------------------------------------------------------------

// LED Globals -------------------------------------------------------------------------------------
#define LED_PIN 13
bool blinkState = false;
// -------------------------------------------------------------------------------------------------

// Speed PID Globals -------------------------------------------------------------------------------
float sKp, sKi, sKd;
float sTarget = 0, sLastErr = 0, sErrSum = 0, sErrOrderF = 0, sErrOrderFprev = 0, sErrorPrev = 0;
double sLastTime;
// -------------------------------------------------------------------------------------------------

// Angle PID Globals -------------------------------------------------------------------------------
float aKp, aKi, aKd;
float aLastErr = 0, aErrSum = 0, aErrOrderF = 0, aErrOrderFprev = 0, aErrorPrev = 0;
double aLastTime;
// -------------------------------------------------------------------------------------------------

// Main Loop Globals -------------------------------------------------------------------------------
#include <EEPROM.h>
float angle = 0;
float alpha = 0.996;
float beta = 0.996;
float rpm;
float controlSpeed;
float controlAngle;
bool run = true;
bool stream = false;
unsigned long compTimeLast = 0;
// -------------------------------------------------------------------------------------------------

float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch, AngleYaw;
uint32_t LoopTimer;

float KalmanAngleRoll=0, KalmanUncertaintyAngleRoll=2*2;
float KalmanAnglePitch=0, KalmanUncertaintyAnglePitch=2*2;
float KalmanAngleYaw=0, KalmanUncertaintyAngleYaw=2*2;
float Kalman1DOutput[]={0,0};
void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState=KalmanState+0.004*KalmanInput;
  KalmanUncertainty=KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
  float KalmanGain=KalmanUncertainty * 1/(1*KalmanUncertainty + 3 * 3);
  KalmanState=KalmanState+KalmanGain * (KalmanMeasurement-KalmanState);
  KalmanUncertainty=(1-KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0]=KalmanState; 
  Kalman1DOutput[1]=KalmanUncertainty;
}

void setup() {
  Serial.begin(38400);
  Serial.println("<Arduino Initializing>");

  motorInit();  // Initialize motor
  encoderInit();  // Initialize encoder and interrupts
  mpuInit();  // Initialize mpu
  pidInit();  // Initialize PID variables with EEPROM

  Serial.println("<Arduino Ready>");
  delay(500);
}

void ai0() {
  // Interrupt for pin 2 (A signal)
  if (digitalRead(3) == LOW) {
    counter++;  // Counter increments if B signal is LOW
  } else {
    counter--;  // Counter decrements if B signal is HIGH
  }
}

void ai1() {
  // Interrupt for pin 3 (B signal)
  if (digitalRead(2) == LOW) {
    counter--;  // Counter decrements if A signal is LOW
  } else {
    counter++;  // Counter increments if A signal is HIGH
  }
}

void loop() {
  if (Serial.available()){
    tunePID();
  }

  
  gyro_signals();
  RateYaw-=RateCalibrationYaw;

  kalman_1d(KalmanAngleYaw, KalmanUncertaintyAngleYaw, RateYaw, AngleYaw);
  KalmanAngleYaw=Kalman1DOutput[0]; 
  KalmanUncertaintyAngleYaw=Kalman1DOutput[1];



  //mpuData mpuData = getMpuData();

  //unsigned long compTime = millis() - compTimeLast; compTimeLast = millis();

  //angle = alpha*(angle + (mpuData.omega_z * (compTime/1000.0))) + ((1-alpha)*mpuData.theta_z);

  angle = KalmanAngleYaw + 0.5 ;

  encoderData encoderData = getEncoderData();

  rpm = rpm - (beta * (rpm - encoderData.rpm));
  
  controlSpeed = speedPID();
  
  controlAngle = anglePID();

  if (run && abs(angle) < 20){    
    digitalWrite(motorDIR_Pin, (controlAngle >= 0) ? 0 : 1);
    analogWrite(motorPWM_Pin, (255-abs(int(controlAngle))));
    digitalWrite(motorBrake_Pin, HIGH);
  }else{
    analogWrite(motorPWM_Pin, 255);
    digitalWrite(motorBrake_Pin, LOW);
  }

  if (stream){    
    Serial.print("SET:"); Serial.print(0); Serial.print(",");
    Serial.print("Angle:"); Serial.print(angle, 4); Serial.print(",");
    Serial.print("RPM:"); Serial.print(rpm, 4); Serial.print(",");
    Serial.print("PIDspeed:"); Serial.print(controlSpeed, 4); Serial.print(",");
    Serial.print("PIDangle:"); Serial.println(controlAngle, 4);
  }

  


  while (micros() - LoopTimer < 4000);
  LoopTimer=micros();




}

float speedPID(){
  unsigned long sCurrentTime = millis();
  float sTime = (sCurrentTime - sLastTime)/1000;

  float sError = sTarget - rpm;

  sErrSum += sError * sTime;
  sErrSum = constrain(sErrSum, -50, 50);

  float sErrOrder = (sError - sLastErr) / sTime;
  sErrOrderF = (0.9 * sErrOrderFprev) + (0.1 * sErrOrder);
  sErrOrderFprev = sErrOrderF;

  float sPterm = sKp * sError;
  sPterm = constrain(sPterm, -3, 3);
  float sIterm = sKi * sErrSum;
  float sDterm = sKd * sErrOrder;

  float controlSpeed1 = sPterm + sIterm + sDterm;

  sLastErr = sError;
  sLastTime = sCurrentTime;
  
  #ifdef DEBUG_SPEED_PID
  Serial.print("sPterm:"); Serial.print(sPterm, 8); Serial.print(",");
  Serial.print("sIterm:"); Serial.print(sIterm); Serial.print(",");
  Serial.print("sDterm:"); Serial.println(sDterm);
  #endif

  return controlSpeed1;
}

float anglePID(){
  unsigned long aCurrentTime = millis();
  float aTime = (aCurrentTime - aLastTime)/1000;

  float aError = (controlSpeed - angle)*10;
  
  aErrSum += aError * aTime;
  aErrSum = constrain(aErrSum, -50, 50);

  float aErrOrder = ((aError - aLastErr) / aTime);
  aErrOrderF = (0.9 * aErrOrderFprev) + (0.1 * aErrOrder);
  aErrOrderFprev = aErrOrderF;

  float aPterm = aKp * aError;
  aPterm = constrain(aPterm, -255, 255);
  
  float aIterm = aKi * aErrSum;
  float aDterm = aKd * aErrOrderF;

  float controlAngle1 = aPterm + aIterm + aDterm;
  controlAngle1 = constrain(controlAngle1, -255, 255);

  aLastErr = aError;
  aLastTime = aCurrentTime;
  
  #ifdef DEBUG_ANGLE_PID
  Serial.print("aIterm:"); Serial.print(aIterm); Serial.print(",");
  Serial.print("sIterm:"); Serial.print(sIterm); Serial.print(",");
  Serial.print("sDterm:"); Serial.println(sDterm);
  #endif

  return controlAngle1;
}

void gyro_signals(void) {
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(); 
  Wire.requestFrom(0x68,6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
  Wire.beginTransmission(0x68);
  Wire.write(0x1B); 
  Wire.write(0x8);
  Wire.endTransmission();     
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  int16_t GyroX=Wire.read()<<8 | Wire.read();
  int16_t GyroY=Wire.read()<<8 | Wire.read();
  int16_t GyroZ=Wire.read()<<8 | Wire.read();
  RateRoll=(float)GyroX/65.5;
  RatePitch=(float)GyroY/65.5;
  RateYaw=(float)GyroZ/-65.5;
  AccX=(float)AccXLSB/4096-0.01;
  AccY=(float)AccYLSB/4096+0.01;
  AccZ=(float)AccZLSB/4096+0.05;
  AngleRoll=atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*1/(3.142/180);
  AnglePitch=-atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*1/(3.142/180);
  AngleYaw=atan2(AccY, AccX)*1/(3.142/180);
}

encoderData getEncoderData(){
  unsigned long currentMillis = millis();
  unsigned long time = (currentMillis - lastMillis);

  if (time > 10){
    long count = (counter - lastCounter);

    bool encDIR;

    float seconds = (time/1000.0);

    float revolutions = count/200.0;

    float rps = revolutions/seconds;

    if (counter > lastCounter){
      encDIR = 1;
    } else{
      encDIR = -1;
    }
    
    lastMillis = currentMillis;
    lastCounter = counter;

    encoderData result;
    result.rpm = rps * encDIR;
    result.direction = encDIR;

    #ifdef DEBUG_ENCODER
    Serial.print("rps:");  Serial.print(rps, 3); Serial.print(",");
    Serial.print("counter:");  Serial.println(counter);
    #endif

    return result;
  }
}

void pidInit(){
  // Read float values from EEPROM
  EEPROM.get(0, sKp); // Read sKp (float) from EEPROM starting at address 0
  EEPROM.get(4, sKi); // Read sKi (float) from EEPROM starting at address 4
  EEPROM.get(8, sKd); // Read sKd (float) from EEPROM starting at address 8
  EEPROM.get(12, aKp); // Read aKp (float) from EEPROM starting at address 12
  EEPROM.get(16, aKi); // Read aKi (float) from EEPROM starting at address 16
  EEPROM.get(20, aKd); // Read aKd (float) from EEPROM starting at address 20
  
  Serial.print("sKp = "); Serial.print(sKp, 3); Serial.print(" , "); 
  Serial.print("sKi = "); Serial.print(sKi); Serial.print(" , "); 
  Serial.print("sKd = "); Serial.print(sKd); Serial.print(" , "); 
  Serial.print("aKp = "); Serial.print(aKp, 3); Serial.print(" , "); 
  Serial.print("aKi = "); Serial.print(aKi); Serial.print(" , "); 
  Serial.print("aKd = "); Serial.println(aKd, 3);
}

void mpuInit(){
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68); 
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  for (RateCalibrationNumber=0; RateCalibrationNumber<2000; RateCalibrationNumber ++) {
    gyro_signals();
    RateCalibrationRoll+=RateRoll;
    RateCalibrationPitch+=RatePitch;
    RateCalibrationYaw+=RateYaw;
    delay(2);
  }
  RateCalibrationRoll/=2000;
  RateCalibrationPitch/=2000;
  RateCalibrationYaw/=2000;

  LoopTimer=micros();
}

void encoderInit(){
  pinMode(2, INPUT_PULLUP); // Internal pull-up for pin 2
  pinMode(3, INPUT_PULLUP); // Internal pull-up for pin 3

  attachInterrupt(digitalPinToInterrupt(2), ai0, RISING);  // Interrupt on pin 2 (encoder A)
  attachInterrupt(digitalPinToInterrupt(3), ai1, RISING);  // Interrupt on pin 3 (encoder B)

  Serial.println("Encoder connection successful");
}

void motorInit(){
  InitTimersSafe();
  SetPinFrequencySafe(motorPWM_Pin, Nidec_Frequency);

  pinMode(motorPWM_Pin, OUTPUT);
  pinMode(motorDIR_Pin, OUTPUT);
  pinMode(motorBrake_Pin, OUTPUT);

  pwmWrite(motorPWM_Pin, 255);
  digitalWrite(motorDIR_Pin, HIGH);
  digitalWrite(motorBrake_Pin, LOW);

  motorDIR = 1;
  Serial.println("Motor connection successful");
}

void tunePID(){
  char cmd = Serial.read();

  switch (cmd){
    case 'q': sKp += 0.001; EEPROM.put(0, sKp);
    break;
    
    case 'a': sKp -= 0.001; EEPROM.put(0, sKp);
    break;
    
    case 'w': sKi += 0.01; EEPROM.put(4, sKi);
    break;
  
    case 's': sKi -= 0.01; EEPROM.put(4, sKi);
    break;
    
    case 'e': sKd += 0.01; EEPROM.put(8, sKd);
    break;
    
    case 'd': sKd -= 0.01; EEPROM.put(8, sKd);
    break;
    
    case 'r': aKp += 0.01; EEPROM.put(12, aKp);
    break;
    
    case 'f': aKp -= 0.01; EEPROM.put(12, aKp);
    break;
    
    case 't': aKi += 0.01; EEPROM.put(16, aKi);
    break;
    
    case 'g': aKi -= 0.01; EEPROM.put(16, aKi);
    break;
    
    case 'y': aKd += 0.001; EEPROM.put(20, aKd);
    break;
    
    case 'h': aKd -= 0.001; EEPROM.put(20, aKd);
    break;

    case 'z': 
      sKp = 0; sKi = 0; sKd = 0; aKp = 0; aKi = 0; aKd = 0;
      for (int i = 0; i < 25; i++){
        EEPROM.write(i, 0);
      }
    break;
 
    case 'x': 
      sKp = 0.276; sKi = 0.02; sKd = 0; aKp = 2.470; aKi = 0; aKd = 0.200;
      EEPROM.put(0, sKp); EEPROM.put(4, sKi); EEPROM.put(8, sKd); EEPROM.put(12, aKp); EEPROM.put(16, aKi); EEPROM.put(20, aKd);
    break;

    case 'c': 
      sKp = 0.375; sKi = 0.02; sKd = 0; aKp = 2.070; aKi = 0.01; aKd = 0.300;
      EEPROM.put(0, sKp); EEPROM.put(4, sKi); EEPROM.put(8, sKd); EEPROM.put(12, aKp); EEPROM.put(16, aKi); EEPROM.put(20, aKd);
    break;

    case 'l': run = !run;
    break;
    
    case 'p': stream = !stream;
    break;
    
    default: Serial.println("<Error - no tuning value>");
    cmd = Serial.read();    
  }

  if (stream == false){
    Serial.print("sKp = "); Serial.print(sKp, 3); Serial.print(" , "); 
    Serial.print("sKi = "); Serial.print(sKi); Serial.print(" , "); 
    Serial.print("sKd = "); Serial.print(sKd); Serial.print(" , "); 
    Serial.print("aKp = "); Serial.print(aKp, 3); Serial.print(" , "); 
    Serial.print("aKi = "); Serial.print(aKi); Serial.print(" , "); 
    Serial.print("aKd = "); Serial.println(aKd, 3);
  }
}
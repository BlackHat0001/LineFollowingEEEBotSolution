#include <MPU6050_tockn.h>


#include <NewPing.h>


#include <MPU6050_tockn.h>


#include <Wire.h>
#define I2C_SLAVE_ADDR 0x04


#define TRIGGER_PIN 4
#define ECHO_PIN 2
#define MAX_DISTANCE 800

#define SENSOR1 A14
#define SENSOR2 A15
#define SENSOR3 A16
#define SENSOR4 A17
#define SENSOR5 A18
#define SENSOR6 A19

MPU6050 mpu(Wire);


NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

float sensor_1_val = 1;
float sensor_2_val = 0;
float sensor_3_val = 0;
float sensor_4_val = 0;
float sensor_5_val = 0;
float sensor_6_val = 0;

int leftMotor_speed = 0;
int rightMotor_speed = 0;
int servoAngle = 90;

float Weighted_Average = 0;

bool leftright = false;
bool CanSeeLine = true;

float SensorThreshArr[] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
float SensorOffsetArr[] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
bool SensingArr[] = {false, false, false, false, false, false};

int servomiddleangle = 103;

float error = 0;

float constantCoef = 0.275f;
float differentialCoef = 0.18f;
float integralCeof = 0.0000f;
float Ks = 0.9f;

void setup()
{
  Serial.begin(9600);
  Wire.begin();

  CalculateThresholds();


  //mpu.begin();
  //mpu.setGyroOffsets(5.11, -0.29, 0.23);
}

void loop()
{
  
  UpdateIRSensors();

  WeightedAverage();

  float PIDOut = PID(Weighted_Average);

  float Corrected_Average = PIDOut*6.0f;

  float Corrected_Angle = 160*Corrected_Average;

  leftMotor_speed = abs(MapFunction(0.15f-constrain(error, 0.0f, 0.15f), 0.02f, 0.15f, 110, 210))+(Ks*PIDOut);
  rightMotor_speed = abs(MapFunction(0.15f-constrain(error, 0.0f, 0.15f), 0.02f, 0.15f, 110, 210))-(Ks*PIDOut);

  if(!CanSeeLine) {
    if(!leftright) {
      servoAngle = 103 + 35;
      leftMotor_speed = -120;
      rightMotor_speed = -120;
    } else {
      servoAngle = 103 - 35;
      leftMotor_speed = -120;
      rightMotor_speed = -120; 
    }
    error = 1;
  } else {
    if(SensingArr[0] == true && SensingArr[1] == false && SensingArr[2] == false && SensingArr[3] == false && SensingArr[4] == false && SensingArr[5] == false) {
      Weighted_Average = -1.0f;
    }
    if(SensingArr[0] == false && SensingArr[1] == false && SensingArr[2] == false && SensingArr[3] == false && SensingArr[4] == false && SensingArr[5] == true) {
      Weighted_Average = 1.0f;
    }
    if(Weighted_Average < 0) {
    leftright = false;
    } else {
      leftright = true;
    }
    
    if(leftMotor_speed < 130 || rightMotor_speed < 130) {
      constantCoef = 0.315f;
      differentialCoef = 0.18f;
    } else {
      constantCoef = 0.22f;
      differentialCoef = 0.2f;
    }

    error = abs(Weighted_Average);

    servoAngle = servomiddleangle + -1*Corrected_Angle;
    
  }

  Transmit();
}

void MeasureThresholds(float arr[]) {

  int repeat = 500;

  for(int i=0; i<repeat; i++) {
    UpdateIRSensors();

    arr[0] = arr[0] + sensor_1_val;
    arr[1] = arr[1] + sensor_2_val;
    arr[2] = arr[2] + sensor_3_val;
    arr[3] = arr[3] + sensor_4_val;
    arr[4] = arr[4] + sensor_5_val;
    arr[5] = arr[5] + sensor_6_val;
  }
  arr[0] = arr[0]/repeat;
  arr[1] = arr[1]/repeat;
  arr[2] = arr[2]/repeat;
  arr[3] = arr[3]/repeat;
  arr[4] = arr[4]/repeat;
  arr[5] = arr[5]/repeat;

}

void CalculateThresholds() {

  float White_ThreshArr[] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  float Black_ThreshArr[] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  
  delay(2000);
  MeasureThresholds(White_ThreshArr);
  WiggleSteering();
  delay(2000);
  MeasureThresholds(Black_ThreshArr);
  WiggleSteering();

  for(int i=0; i<6; i++) {
    //Fix this
    if(SensorOffsetArr[i]<0.20f) {
      SensorOffsetArr[i] = 0.20f - White_ThreshArr[i];
    }
    if(SensorOffsetArr[i]>0.20f) {
      SensorOffsetArr[i] = White_ThreshArr[i] - 0.20f;
    }
  
    SensorThreshArr[i] = 0.2f + (Black_ThreshArr[i] - White_ThreshArr[i])/2;
  }



}

void UpdateIRSensors() {

  sensor_1_val = 1-SmoothStep(0, 4095, analogRead(SENSOR1)) + SensorOffsetArr[0];
  sensor_2_val = 1-SmoothStep(0, 4095, analogRead(SENSOR2)) + SensorOffsetArr[1];
  sensor_3_val = 1-SmoothStep(0, 4095, analogRead(SENSOR3)) + SensorOffsetArr[2];
  sensor_4_val = 1-SmoothStep(0, 4095, analogRead(SENSOR4)) + SensorOffsetArr[3];
  sensor_5_val = 1-SmoothStep(0, 4095, analogRead(SENSOR5)) + SensorOffsetArr[4];
  sensor_6_val = 1-SmoothStep(0, 4095, analogRead(SENSOR6)) + SensorOffsetArr[5];


  SensingArr[0] = false;
  SensingArr[1] = false;
  SensingArr[2] = false;
  SensingArr[3] = false;
  SensingArr[4] = false;
  SensingArr[5] = false;
  if(sensor_1_val > SensorThreshArr[0]) {
    CanSeeLine = true;
    SensingArr[0] = true;
  } else if(sensor_2_val > SensorThreshArr[1]) {
    CanSeeLine = true;
    SensingArr[1] = true;    
  } else if(sensor_3_val > SensorThreshArr[2]) {
    CanSeeLine = true;
    SensingArr[2] = true;
  } else if(sensor_4_val > SensorThreshArr[3]) {
    CanSeeLine = true;
    SensingArr[3] = true;
  } else if(sensor_5_val > SensorThreshArr[4]) {
    CanSeeLine = true;
    SensingArr[4] = true;
  } else if(sensor_6_val > SensorThreshArr[5]) {
    CanSeeLine = true;
    SensingArr[5] = true;
  } else {
    CanSeeLine = false;
  }

}


void WeightedAverage() {

  Weighted_Average = (sensor_1_val * -1.0f + sensor_2_val * -0.666f + sensor_3_val * -0.333f + sensor_4_val * 0.333f + sensor_5_val * 0.666f + sensor_6_val * 1.0f)/6;

}

void WiggleSteering() {
  servoAngle = servomiddleangle+25;
  Transmit();
  delay(400);
  servoAngle = servomiddleangle-25;
  Transmit();
  delay(400);
  servoAngle = servomiddleangle;
  Transmit();
}


void Transmit()
{
  Wire.beginTransmission(I2C_SLAVE_ADDR); // transmit to device #4
  /* depending on the mirocontroller, the int variable is stored as 32-bits or 16-bits
     if you want to increase the value range, first use a suitable variable type and then modify the code below
     for example; if the variable used to store x and y is 32-bits and you want to use signed values between -2^31 and (2^31)-1
     uncomment the four lines below relating to bits 32-25 and 24-17 for x and y
     for my microcontroller, int is 32-bits hence x and y are AND operated with a 32 bit hexadecimal number - change this if needed


     >> X refers to a shift right operator by X bits
  */
  //Wire.write((byte)((x & 0xFF000000) >> 24)); // bits 32 to 25 of x
  //Wire.write((byte)((x & 0x00FF0000) >> 16)); // bits 24 to 17 of x
  Wire.write((byte)((leftMotor_speed & 0x0000FF00) >> 8));    // first byte of x, containing bits 16 to 9
  Wire.write((byte)(leftMotor_speed & 0x000000FF));           // second byte of x, containing the 8 LSB - bits 8 to 1
  //Wire.write((byte)((y & 0xFF000000) >> 24)); // bits 32 to 25 of y
  //Wire.write((byte)((y & 0x00FF0000) >> 16)); // bits 24 to 17 of y
  Wire.write((byte)((rightMotor_speed & 0x0000FF00) >> 8));    // first byte of y, containing bits 16 to 9
  Wire.write((byte)(rightMotor_speed & 0x000000FF));           // second byte of y, containing the 8 LSB - bits 8 to 1
 
  Wire.write((byte)((servoAngle & 0x0000FF00) >> 8));
  Wire.write((byte)(servoAngle & 0x000000FF));  
  Wire.endTransmission();   // stop transmitting
  delay(100);
}


long timePassed = 0;
long lastTime = 0;


float constant = 0;
float constantLast = 0;
float differential = 0;
float integral = 0;

float PID(float deviationAngle) {
  float time = millis();
  timePassed = time - lastTime;


  constant = 0 - deviationAngle;
  differential = (constant - constantLast)/timePassed;
  integral = integral + (timePassed * constant);


  constantLast = constant;
  lastTime = time;


  return (constantCoef*constant + differentialCoef*differential + integralCeof*integral);
}




float SmoothStep(float edge0, float edge1, float x) {
  float t = constrain((x - edge0) / (edge1 - edge0), 0, 1.0);
  return t * t * (3.0 - 2.0 * t);
}

float MapFunction(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
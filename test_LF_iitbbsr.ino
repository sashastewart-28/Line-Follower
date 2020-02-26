#include <QTRSensors.h>

//QTRA
#define NUM_SENSORS              6  // number of sensors used
#define NUM_SAMPLES_PER_SENSOR   4  // average 8 analog samples per sensor reading (For noise suppression during calliberation)
#define NUM_SAMPLES_PER_SENSORC  8  // average 4 analog samples per sensor reading (For running)
#define EMITTER_PIN              11 // emitter is controlled by digital pin 11(beacause same side on nano)
//-------------------------------------------------

//QTRA Sensor
QTRSensorsAnalog qtra((unsigned char[]) {A0, A1, A2, A3, A4, A5},
                                        NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
unsigned int sensorValues[8];

//Motor Driver
// motor left
int rightMotor1 =49;
int rightMotor2 =48;
int rightMotorPWM = 3; 
// motor right
int leftMotor1 =38;
int leftMotor2 =39;
int leftMotorPWM = 2;
//-------------------------------------------------
//PID CONTROL DEFINATIONS
//FOR IR.........
#define Kp 0.1 //to be determined experimentally 
#define Kd 4.1 //to be determined experimentally(Kp<Kd)
#define Ki 0 //to be determined experimentally
double last_error=0;
double pos;
double error;
double correction;
double integral;
double lastError;

//....................................................
//MOTOR SPEED CONTROL PARAMETERS
#define speedturn 150
#define MaxSpeed 210
#define BaseSpeed 210
int motorSpeed;
int rightMotorSpeed;
int leftMotorSpeed;
//int rgtspdUSS;
//int lftspdUSS; 
//..........................................................


//TIME DEFINATIONS
long current_time;
long start_time;
long start_time_right;
long start_time_st;
long duration;
//.............................................................


void setup() 
{
  //digitalWrite(push,LOW);
  //pinMode(push,INPUT);
  // put your setup code here, to run once:
  //QTRA Setup----------------------------------------------
  digitalWrite(13, HIGH);        // turn on Arduino's LED to indicate we are in calibration mode
  for (int i = 0; i < 300; i++)  // make the calibration take about 10 seconds
    {
      /*if(i<150)
      {
        digitalWrite(LDIR1,HIGH);
        digitalWrite(LDIR2,LOW);
        digitalWrite(RDIR1,LOW);
        digitalWrite(RDIR2,HIGH);
        analogWrite(LENB,150);
        analogWrite(RENB,150);
      }
      else
      {
        digitalWrite(LDIR1,LOW);
        digitalWrite(LDIR2,HIGH);
        digitalWrite(RDIR1,HIGH);
        digitalWrite(RDIR2,LOW);
        analogWrite(LENB,150);
        analogWrite(RENB,150);
      }*/
      qtra.calibrate();          // reads all sensors 10 times at 2.5 ms per six sensors (i.e. ~25 ms per call) (Rotates at 100/255 % speed of motor)
    }
  digitalWrite(13, LOW);         // turn off Arduino's LED to indicate we are through with calibration

 // print the calibration minimum values measured when emitters were on
  for (int i = 0; i < NUM_SENSORS; i++)
    {
      Serial.print(qtra.calibratedMinimumOn[i]);
      Serial.print(' ');
    }
  Serial.println();
  
  // print the calibration maximum values measured when emitters were on
   for (int i = 0; i < NUM_SENSORS; i++)
    {
      Serial.print(qtra.calibratedMaximumOn[i]);
      Serial.print(' ');
    }
   Serial.println();
   Serial.println();
  //---------------------------------------------------------- 
}

//BASIC LINE FOLLOWING MOVEMENT FUNCTION---------------------------------------------------------------------
void Move()
{
  pos=qtra.readLine(sensorValues,QTR_EMITTERS_ON,0); //qtra.readLine(sensorValues,QTR_EMITTERS_ON,TRUE/1);for white on black
  Serial.print("POSITION");
  Serial.print("=");
  Serial.print(pos);
  Serial.println();
  if(pos>6700)
  {
    move(1, speedturn, 1);
    move(0, speedturn, 0);
    return;    
  }
  if(pos<300)
  { 
    move(1, speedturn, 0);
    move(0, speedturn, 1);
    return;
  }
 /* if(sensorValues[6]>800 && sensorValues[7]>800 && sensorValues[0]<700 && sensorValues[1]<700 && sensorValues[2]<700 && sensorValues[3]<700 && sensorValues[4]<700 && sensorValues[5]<700)
  {
      digitalWrite(leftMotor1, LOW);
      digitalWrite(leftMotor2, HIGH);
      analogWrite(leftMotorPWM, 140);
      digitalWrite(rightMotor1, HIGH);
      digitalWrite(rightMotor2, LOW);
      analogWrite(rightMotorPWM, 140);
      delay(200);
      analogWrite(leftMotorPWM,0);
      analogWrite(rightMotorPWM,0);
  }*/
  error = pos - 2500;
  motorSpeed = Kp * error + Kd * (error - lastError);
  lastError = error;

  rightMotorSpeed = BaseSpeed + motorSpeed;
  leftMotorSpeed = BaseSpeed - motorSpeed;
  
  if (rightMotorSpeed > MaxSpeed ) rightMotorSpeed = MaxSpeed; // prevent the motor from going beyond max speed
  if (leftMotorSpeed > MaxSpeed ) leftMotorSpeed = MaxSpeed; // prevent the motor from going beyond max speed
  if (rightMotorSpeed < 0)rightMotorSpeed = 0;    
  if (leftMotorSpeed < 0)leftMotorSpeed = 0;
    
  move(1, rightMotorSpeed, 1);
  move(0, leftMotorSpeed, 1);
}
void move(int motor, int speed, int direction)
{
  //digitalWrite(motorPower, HIGH); //disable standby

  boolean inPin1=HIGH;
  boolean inPin2=LOW;
  
  if(direction == 1){
    inPin1 = HIGH;
    inPin2 = LOW;
  }  
  if(direction == 0){
    inPin1 = LOW;
    inPin2 = HIGH;
  }

  if(motor == 0){
    digitalWrite(leftMotor1, inPin1);
    digitalWrite(leftMotor2, inPin2);
    analogWrite(leftMotorPWM, speed);
  }
  if(motor == 1){
    digitalWrite(rightMotor1, inPin1);
    digitalWrite(rightMotor2, inPin2);
    analogWrite(rightMotorPWM, speed);
  }  
}

void loop() 
{
  // put your main code here, to run repeatedly:
  //QTRA Sensor Calls
   unsigned int position = qtra.readLine(sensorValues);
  
  // print the sensor values as numbers from 0 to 1000, where 0 means maximum reflectance and
  // 1000 means minimum reflectance, followed by the line position
  for (unsigned char i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println(position);
  //if(push==HIGH)
  
   Move(); 
  
  
  delay(250);
}

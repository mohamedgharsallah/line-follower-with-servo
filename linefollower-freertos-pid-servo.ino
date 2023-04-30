TaskHandle_t Task1;
TaskHandle_t Task2;

#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"
#include <QTRSensors.h> //Make sure to install the library
#include <Servo.h> 

Servo myservo;

QTRSensors qtr;

int servospeed=0;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

//VALUE FOR MOTOR
float Kp = 10; 
              
float Ki = 0.000; 
              
float Kd = 0;

int P;
int I;
int D;

int ENB =13 ;
int ENA = 32;
int IN1 = 33;
int IN2 = 22;
int IN3 =13;
int IN4 =12;





//VALUE FOR SERVO
float Kps = 0.16; 
              
float Kis = 0.000; 
              
float Kds = 0.0;


//servo contrainte
int  maxservo = 1850;
int minservo = 1100;
int servobase = 1500;





/*************************************************************************
* Motor speed variables (choose between 0 - no speed, and 255 - maximum speed)
*************************************************************************/
int maxspeeda = 65000;
int maxspeedb = 65000;
int basespeeda = 25000;
int basespeedb = 25000;
int lastError = 0;







void calibration() {
  
  for (uint16_t i = 0; i < 150; i++)
  {
    qtr.calibrate();
  }

}

int PID_valuemotor() {
   qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){15,21,4,16,17,27,18,19}, SensorCount);


uint16_t position = qtr.readLineBlack(sensorValues); //read the current position


int error = 3500 - position; //3500 is the ideal position (the centre)

  P = error;
  I = I + error;
  D = error - lastError;
  lastError = error;
  int motorspeed = P*Kp + I*Ki + D*Kd;
  return motorspeed;
}


int PID_valueservo() {
   qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){15,21,4,16,17,27,18,19}, SensorCount);

uint16_t position = qtr.readLineBlack(sensorValues); //read the current position
int error = 3500 - position; //3500 is the ideal position (the centre)

  P = error;
  I = I + error;
  D = error - lastError;
  lastError = error;
  int servo = P*Kps + I*Kis + D*Kds;
  return servo;
}



  


void setup() {
  Serial.begin(9600);



  calibration();
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);


 ledcSetup(0, 50, 16);
 ledcSetup(1, 50, 16);
 
 ledcAttachPin(ENB, 1);
 ledcAttachPin(ENA, 0);


  xTaskCreatePinnedToCore(Task1code,"Task1",110000,NULL,1,&Task1,0);                         
  
 
  xTaskCreatePinnedToCore(Task2code,"Task2",110000,NULL,1,&Task2,1);          
    
}

void Task1code( void * parameter ){
  
  
 myservo.attach(23);

 myservo.writeMicroseconds(1500); 
  
  Serial.print("Task1 is running on core ");
 Serial.println(xPortGetCoreID());

  for(;;){

   
  int servospeed = servobase -  PID_valueservo();
  
if (servospeed > maxservo) {
   servospeed=maxservo;
  }

  if (servospeed< minservo) {
   servospeed=minservo;
  }

// Serial.println(position);

 
myservo.writeMicroseconds(servospeed); 
    
   
  }
  
 
}

void Task2code( void * parameter ){
   

  
  Serial.print("Task2 is running on core ");
  Serial.println(xPortGetCoreID());

  for(;;){
 
//Serial.println(motorspeed);


  int motorspeeda = basespeeda + PID_valuemotor() ;
  int motorspeedb = basespeedb - PID_valuemotor() ;


//motor
  if (motorspeeda > maxspeeda) {
    motorspeeda = maxspeeda;
  }
  if (motorspeedb > maxspeedb) {
    motorspeedb = maxspeedb;
  }
  if (motorspeeda < 0) {
    motorspeeda = 0;
  }
  if (motorspeedb < 0) {
    motorspeedb = 0;
 
   
  }

digitalWrite(IN1,1);
digitalWrite(IN2,0);
digitalWrite(IN3,1);
digitalWrite(IN4,0);
ledcWrite(0,motorspeeda);
ledcWrite(1,motorspeedb);
}
}

void loop() {
  TIMERG0.wdt_wprotect=TIMG_WDT_WKEY_VALUE;
TIMERG0.wdt_feed=1;
TIMERG0.wdt_wprotect=0;
  delay(1);

}

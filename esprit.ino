#include <QTRSensors.h>

QTRSensors qtr;
const uint8_t SensorCount =8;
uint16_t sensorValues[SensorCount];


float Kp = 0.120  ; //related to the proportional control term rep;0.177               //change the value by trial-and-error (ex: 0.07).
float Ki = 0.001  ; //related to the integral control term exact; 0.0009
              //change the value by trial-and-error (ex: 0.0008).
float Kd = 1 ;//related to the derivative control term oss; 
              //change the value by trial-and-error (ex: 0.6).1
              
int P;
int I;
int D;
//configuratoin du vittese 
int lastError = 0;
const uint8_t maxspeeda = 250;
const uint8_t maxspeedb =250;
const uint8_t basespeeda =130;
const uint8_t basespeedb = 130;

//PONT H (L298N)
#define IN1 7
#define ENA 9
#define IN2 8
#define IN4 4
#define ENB 3
#define IN3 2
//declaratoin des inpout du QTR
#define a7 A8 
#define a6 A9
#define a5 A10
#define a4 A11
#define a3 A14
#define a2 A15
#define a1 A12
#define a0 A13
#define U0 digitalRead(a0)
#define U1 digitalRead(a1)
#define U2 digitalRead(a2)
#define U3 digitalRead(a3)
#define U4 digitalRead(a4)
#define U5 digitalRead(a5)
#define U6 digitalRead(a6)
#define U7 digitalRead(a7)
//pushbuton
#define push 22
#define U digitalRead(push )
//buzzz
#define buzz 5
//led de controle 
#define ledl 23  
#define ledr 24 
//tcrt 
#define ur digitalRead(25)
#define ul digitalRead(26)
void setup() {
 qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){a0, a1, a2, a3, a4, a5, a6, a7}, SensorCount);
  Serial.begin(9600);
  qtr.setEmitterPin(10);
  pinMode(push, INPUT);
  pinMode(25, INPUT);
  pinMode(24, INPUT);
  pinMode(buzz, OUTPUT);
pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ledr, OUTPUT);
  pinMode(ledl, OUTPUT);
calibration();
   for (int i=0;i<200;i++){
 analogWrite(buzz,2*i) ;
delay(10); }
digitalWrite(buzz,0) ;
   do{}while (U==0);
  analogWrite(buzz,250);
  delay(1000);
  digitalWrite(buzz,0);

forward();

 
}

void loop() {
  // put your main code here, to run repeatedly:

PID_controlB();
}
void calibration() {
  for (uint16_t i = 0; i < 400; i++)
  {
  qtr.calibrate();
  }
}

void forward_brake(int posa, int posb) {
  //set the appropriate values for aphase and bphase so that the robot goes straight
  if(posa>0){
    digitalWrite(IN4, LOW);  
      digitalWrite(IN3, HIGH);
       analogWrite(ENB, posa);
       }
        if(posb>0){
        digitalWrite(IN2,HIGH );                 
        digitalWrite(IN1, LOW);
        analogWrite(ENA, posb);
}
if(posa<0){
    digitalWrite(IN4, HIGH);  
      digitalWrite(IN3, LOW);
       analogWrite(ENB, posa);
       }
        if(posb<0){
        digitalWrite(IN2,LOW  );                 
        digitalWrite(IN1, HIGH);
        analogWrite(ENA, posb);
}}

/*************************************************************************
* Function Name: PID_control
**********************************************************************
* Summary: 
* This is the function of the PID control system. The distinguishing 
* feature of the PID controller is the ability to use the three control 
* terms of proportional, integral and derivative influence on the controller 
* output to apply accurate and optimal control. This correction is applied to
* the speed of the motors, which should be in range of the interval [0, max_speed],
* max_speed <= 255. 
* 
* Parameters:
* none
* 
* Returns:
*  none
*************************************************************************/
void PID_controlB() {
  uint16_t position = qtr.readLineBlack(sensorValues); 
  int error = 3500 - position; 

  P = error;
  I = I + error;
  D = error - lastError;
  lastError = error;
  int motorspeed = P*Kp + I*Ki + D*Kd; 
  
  int motorspeeda = basespeeda + motorspeed;
  int motorspeedb = basespeedb - motorspeed;
  
  if (motorspeeda > maxspeeda) {
    motorspeeda = maxspeeda;
  }
  if (motorspeedb > maxspeedb) {
    motorspeedb = maxspeedb;
  }
  if (motorspeeda < -77) {
    motorspeeda = -77;
  }
  if (motorspeedb < -77) {
    motorspeedb = -77;
  }  
  forward_brake(motorspeeda, motorspeedb);
 // Serial.println(motorspeeda);Serial.print('\t');Serial.println(motorspeedb);
 }
  /*if (motorspeeda<motorspeedb){digitalWrite(ledr,HIGH);digitalWrite(ledl,LOW);
    }
     else if (motorspeeda>motorspeedb){digitalWrite(ledl,HIGH);digitalWrite(ledr,LOW);
    }
    else{digitalWrite(ledr,LOW);digitalWrite(ledl,LOW);}*/
  

  void forward()
{
  digitalWrite(IN4, LOW);
  digitalWrite(IN3, HIGH);
  analogWrite(ENB, 150);

  digitalWrite(IN2, HIGH  );
  digitalWrite(IN1, LOW);
  analogWrite(ENA, 150);
 
}

void backward()
{

  digitalWrite(IN4, HIGH );
  digitalWrite(IN3,  LOW);
  analogWrite(ENB, 63);

  digitalWrite(IN2, LOW );
  digitalWrite(IN1, HIGH );
  analogWrite(ENA, 60);

}

void left()
{
  digitalWrite(IN4, LOW);
  digitalWrite(IN3, HIGH);
  analogWrite(ENB, 120);

  digitalWrite(IN2, LOW);
  digitalWrite(IN1, HIGH);
  analogWrite(ENA, 120);
  

}

void right()
{
  digitalWrite(IN4, HIGH);
  digitalWrite(IN3, LOW);
  analogWrite(ENB, 120);

  digitalWrite(IN2, HIGH);
  digitalWrite(IN1, LOW);
  analogWrite(ENA, 120);
  
}

void off()
{
  digitalWrite(ENB, LOW);
  digitalWrite(ENA, LOW);
}

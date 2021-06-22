//===================================================================================  Declaring   ==============================================================================

#include <servo.h>
#include <Wire.h>
#include <Time.h>
#include <DS1307RTC.h>
#include <SPI.h>
#include <MFRC522.h>
#include <SoftwareSerial.h>
#include "RTClib.h"




//servo
Servo myservo1;
Servo myservo2;
Servo myservo3;
Servo myservo4;
int pos = 0;



//ir
int IRH = 10;
int IRO = 11;

//RTC
RTC_DS1307 rtc;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

//Buzzer
int buz 6;

//line follower
#define in1 25
#define in2 24
#define in3 23
#define in4 22
#define enA 30
#define enB 31
int M1_Speed = 100; // speed of motor 
int M2_Speed = 100; // speed of motor 2
int LeftRotationSpeed = 250;  // Left Rotation Speed
int RightRotationSpeed = 250; // Right Rotation Speed

//LED
int Rled 8;
int Gled 7;

//miscellaneous

int rand;



//===================================================================================  Void Setup  ==============================================================================


void setup() 
{
  //Line FOllower  
  pinMode(in1,OUTPUT);
  pinMode(in2,OUTPUT);
  pinMode(in3,OUTPUT);
  pinMode(in4,OUTPUT);
  pinMode(enA,OUTPUT);
  pinMode(enB,OUTPUT);
  pinMode(A0, INPUT); // initialize Left sensor as an input
  pinMode(A1, INPUT); // initialize Right sensor as an input

   //RTC
  //  while (!Serial); // for Leonardo/Micro/Zero
  Serial.begin(9600);
  if (! rtc.begin())
  {
    Serial.println("Couldn't find RTC");
    while (1);
  }

  if (! rtc.isrunning())
  {
    Serial.println("RTC is NOT running!");
    // following line sets the RTC to the date & time this sketch was compiled
//     rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    rtc.adjust(DateTime(2021, 6, 22, 10, 24, 15));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  } 



  //IR Sensor
  pinMode (IRH, INPUT); // sensor pin INPUT for hand detection
  pinMode (IRO, INPUT); // sensor pin INPUT for object detection 

  //Servo Motors
  myservo.attach(2);
  myservo.attach(3);
  myservo.attach(4);
  myservo.attach(12);
  
  
  //Buzzer
  digitalWrite(buz, OUTPUT);


  //LED
  digitalWrite(Rled, OUTPUT);
  digitalWrite(Gled, OUTPUT);


//===================================================================================  Void Loop  ===============================================================================


void loop()
{
  linefollower();
}



//===================================================================================  Functions  ===============================================================================

//=================================================================================  Line Follower  =============================================================================



void linefollower() //line follower
{
  int LEFT_SENSOR = digitalRead(A0);
  int RIGHT_SENSOR = digitalRead(A1);
  if(RIGHT_SENSOR==0 && LEFT_SENSOR==0) 
  {
    forward(); //FORWARD
  }
  else if(RIGHT_SENSOR==0 && LEFT_SENSOR==1) 
  {
    right(); //Move Right
  }
  else if(RIGHT_SENSOR==1 && LEFT_SENSOR==0) 
  {
    left(); //Move Left
  }
  else if(RIGHT_SENSOR==1 && LEFT_SENSOR==1) 
  {
   Stop();  //STOP
   right();
   delay(5000);
   stop();
   despensing(); // despensing or RIFD have to read RFID and choose
   delay(10000);
   left();
   delay(5000);
   stop();
   forward();
  }
}  
 
void forward()
{
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enA, M1_Speed);
  analogWrite(enB, M2_Speed);
}
void backward()
{
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enA, M1_Speed);
  analogWrite(enB, M2_Speed);
}
void right()
{
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enA, LeftRotationSpeed);
  analogWrite(enB, RightRotationSpeed);
}
void left()
{
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enA, LeftRotationSpeed);
  analogWrite(enB, RightRotationSpeed);
}
void Stop()
{
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}




//==================================================================================  DESPENSING  ===============================================================================

void despensing() //Despensing system
{
//   Servo motor;
//   int p=9;        //digital pin connected
//   void setup()
//   {
//     motor.attach(9);
//     motor.write(180);
//   }
  
//     int statusSensor = digitalRead(IRSensor); // reads the IR sensor input
  
//   if (statusSensor == 0)
//      despensing();
  
//   void loop(){
//     for(p=0;p<180;p++)
//     {
//       motor.write(p);
//       delay(10);
//     }
//     for(p=180;p>=1;p--)
//     {
//       motor.write(p);
//       delay(10);
//     }

  rand = random(3);
  switch(rand)
  {
    case 0: 
      motor1();
      motor2();
      motor3();
      break;
    
    case 1:
      motor2();
      motor1();
      motor3();
      break;
    
    case 2:
      motor3();
      motor2();
      motor1();
      break;
  }
  
  delay(2000);
  motor4();
}

void motor1()
{
  for (pos = 0; pos <= 180; pos += 1)
  { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(3);                       // waits 15 ms for the servo to reach the position
  }
  for (pos = 180; pos >= 0; pos -= 1) 
  { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(3);                       // waits 15 ms for the servo to reach the position
  }
}

void motor2()
{
    for (pos = 0; pos <= 180; pos += 1)
  { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15 ms for the servo to reach the position
  }
  for (pos = 180; pos >= 0; pos -= 1) 
  { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15 ms for the servo to reach the position
  }
}

void motor3()
{
    for (pos = 0; pos <= 180; pos += 1)
  { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15 ms for the servo to reach the position
  }
  for (pos = 180; pos >= 0; pos -= 1) 
  { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15 ms for the servo to reach the position
  }
}

void motor4()//main dispeinsing
{
    for (pos = 0; pos <= 180; pos += 1)
  { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15 ms for the servo to reach the position
  }
  for (pos = 180; pos >= 0; pos -= 1) 
  { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15 ms for the servo to reach the position
  }
}

 

//=====================================================================================  RTC  ===================================================================================

void time()
{
  DateTime now = rtc.now();
  int date_time[5]={now.day(), now.month(), now.year(), now.hour(), now.minute()};
  Serial.print(date_time);
  delay(3000);
}


//===================================================================================  Buzzers  =================================================================================

void buzzReady()
{
  digitalWrite(buz, HIGH);
  delay(2000);
  digitalWrite(buz, LOW);
  delay(2000);
  digitalWrite(buz, HIGH);
  delay(2000);
  digitalWrite(buz, LOW);  
}


void buzzDone()
{
  digitalWrite(buz, HIGH);
  delay(1000);
  digitalWrite(buz, LOW);
  delay(1000);
  digitalWrite(buz, HIGH);
  delay(1000);
  digitalWrite(buz, LOW);
}

//=====================================================================================  LED  ===================================================================================

void red_led()
{
  digitalWrite(Rled, HIGH);
}


void green_led()
{
  digitalWrite(Gled, HIGH);
}


//==================================================================================  IR Sensor  ================================================================================

void irsensor() //despensing gate way
{
  int statusSensor = digitalRead(IRSensor); // reads the IR sensor input
  
  if (statusSensor == 0)
     despensing();
}

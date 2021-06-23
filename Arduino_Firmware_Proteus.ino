//===================================================================================  Declaring   ==============================================================================

#include <Servo.h>
#include <Wire.h>
#include <time.h>
//#include <ds1307rtc.h>
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
int IRH = 2;
int IRO = 3;

//RTC
//RTC_DS1307 rtc;
//char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

//Buzzer
int buz = 24;

//line follower
#define in1 30
#define in2 31
#define in3 32
#define in4 33
#define enA A2
#define enB A3
int M1_Speed = 250; // speed of motor 
int M2_Speed = 250; // speed of motor 2
int LeftRotationSpeed = 500;  // Left Rotation Speed
int RightRotationSpeed = 500; // Right Rotation Speed

//LED
int Rled = 23;
int Gled = 22;

//miscellaneous

int num;



//===================================================================================  Void Setup  ==============================================================================


void setup() 
{
  Serial.begin(9600);
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
//  Serial.begin(9600);
//  if (! rtc.begin())
//  {
//    Serial.printlnln("Couldn't find RTC");
//    while (1);
//  }
//
//  if (! rtc.isrunning())
//  {
//    Serial.printlnln("RTC is NOT running!");
//    // following line sets the RTC to the date & time this sketch was compiled
////     rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
//    rtc.adjust(DateTime(2021, 6, 22, 10, 24, 15));
//    // This line sets the RTC with an explicit date & time, for example to set
//    // January 21, 2014 at 3am you would call:
//    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
//  } 
//


  //IR Sensor
  pinMode (IRH, INPUT); // sensor pin INPUT for hand detection
  pinMode (IRO, INPUT); // sensor pin INPUT for object detection 

  //Servo Motors
  myservo1.attach(25);
  myservo2.attach(26);
  myservo3.attach(27);
  myservo4.attach(28);
  
  
  //Buzzer
  pinMode(buz, OUTPUT);


  //LED
  pinMode(Rled, OUTPUT);
  pinMode(Gled, OUTPUT);
}

//===================================================================================  Void Loop  ===============================================================================


void loop()
{
  linefollower();
}



//===================================================================================  Functions  ===============================================================================

//=================================================================================  Line Follower  =============================================================================



//void linefollower() //line follower
//{
//  Serial.println("Starting to move");
//  int LEFT_SENSOR = digitalRead(A0);
//  int RIGHT_SENSOR = digitalRead(A1);
////  if(RIGHT_SENSOR==0 && LEFT_SENSOR==0) 
////  {
////    forward(); //FORWARD
////  }
////  else if(RIGHT_SENSOR==0 && LEFT_SENSOR==1) 
////  {
////    right(); //Move Right
////  }
////  else if(RIGHT_SENSOR==1 && LEFT_SENSOR==0) 
////  {
////    left(); //Move Left
////  }
////  else if(RIGHT_SENSOR==1 && LEFT_SENSOR==1) 
////  {
//
//   forward();
//   delay(10000);
//   right();
//   delay(5000);
//   forward();
//   delay(3000);
//   left();
//   delay(2000);
//   Stop();  //STOP
//   right();
//   delay(5000);
//   Stop();
//   despensing(); // despensing or RIFD have to read RFID and choose
//   delay(10000);
//   left();
//   delay(5000);
//   Stop();
//   forward();
////  }
//}  

void linefollower() //line follower
{
  int LEFT_SENSOR = digitalRead(A1);
  int RIGHT_SENSOR = digitalRead(A0);
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
   delay(500);
   Stop();
   despensing(); // despensing or RIFD have to read RFID and choose
   delay(500);
   left();
   delay(500);
   Stop();
   forward();
  }
} 
 
void forward()
{
  Serial.println("Line follower Moving Forward");
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enA, M1_Speed);
  analogWrite(enB, M2_Speed);
}
void backward()
{
  Serial.println("Line follower Moving Backwards");
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enA, M1_Speed);
  analogWrite(enB, M2_Speed);
}
void right()
{
  Serial.println("Line follower Turning Right");
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enA, LeftRotationSpeed);
  analogWrite(enB, RightRotationSpeed);
}
void left()
{
  Serial.println("Line follower Turning Left");
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enA, LeftRotationSpeed);
  analogWrite(enB, RightRotationSpeed);
}
void Stop()
{
  Serial.println("Line follower Has Stopped Moving");
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}




//==================================================================================  DESPENSING  ===============================================================================

void despensing() //Despensing system
{
  num = random(0, 3);
  switch(num)
  {
    case 0: 
      Serial.println("Despensing patient 1 Medicines");
      motor1();
      motor2();
      motor3();
      break;
    
    case 1:
      Serial.println("Despensing patient 2 Medicines");
      motor2();
      motor1();
      motor3();
      break;
    
    case 2:
      Serial.println("Despensing patient 3 Medicines");
      motor3();
      motor2();
      motor1();
      break;
  }
  
  delay(500);
  irsensordespensor();
}

void motor1()
{
  Serial.println("Motor 1 turning");
  for (pos = 0; pos <= 180; pos += 1)
  { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo1.write(pos);              // tell servo to go to position in variable 'pos'
    delay(3);                       // waits 15 ms for the servo to reach the position
  }
  for (pos = 180; pos >= 0; pos -= 1) 
  { // goes from 180 degrees to 0 degrees
    myservo1.write(pos);              // tell servo to go to position in variable 'pos'
    delay(3);                       // waits 15 ms for the servo to reach the position
  }
}

void motor2()
{
  Serial.println("Motor 2 turning");
    for (pos = 0; pos <= 180; pos += 1)
  { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo2.write(pos);              // tell servo to go to position in variable 'pos'
    delay(3);                       // waits 15 ms for the servo to reach the position
  }
  for (pos = 180; pos >= 0; pos -= 1) 
  { // goes from 180 degrees to 0 degrees
    myservo2.write(pos);              // tell servo to go to position in variable 'pos'
    delay(3);                       // waits 15 ms for the servo to reach the position
  }
}

void motor3()
{
  Serial.println("Motor 3 turning");
    for (pos = 0; pos <= 180; pos += 1)
  { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo3.write(pos);              // tell servo to go to position in variable 'pos'
    delay(3);                       // waits 15 ms for the servo to reach the position
  }
  for (pos = 180; pos >= 0; pos -= 1) 
  { // goes from 180 degrees to 0 degrees
    myservo3.write(pos);              // tell servo to go to position in variable 'pos'
    delay(3);                       // waits 15 ms for the servo to reach the position
  }
}

void motor4()//main dispeinsing
{
  Serial.println("Motor 4 turning");
    for (pos = 0; pos <= 180; pos += 1)
  { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo4.write(pos);              // tell servo to go to position in variable 'pos'
    delay(3);                       // waits 15 ms for the servo to reach the position
  }
  for (pos = 180; pos >= 0; pos -= 1) 
  { // goes from 180 degrees to 0 degrees
    myservo4.write(pos);              // tell servo to go to position in variable 'pos'
    delay(3);                       // waits 15 ms for the servo to reach the position
  }
}

 

//=====================================================================================  RTC  ===================================================================================

//void time()
//{
//  DateTime now = rtc.now();
//  int date_time[5]={now.day(), now.month(), now.year(), now.hour(), now.minute()};
//  Serial.println(date_time);
//  delay(500);
//}


//===================================================================================  Buzzers  =================================================================================

void buzzReady()
{
  Serial.println("Buzzing for medicine reminder");
  digitalWrite(buz, HIGH);
  delay(500);
  digitalWrite(buz, LOW);
  delay(500);
  digitalWrite(buz, HIGH);
  delay(500);
  digitalWrite(buz, LOW);  
}


void buzzDone()
{
  Serial.println("Buzzing for moving on");
  digitalWrite(buz, HIGH);
  delay(500);
  digitalWrite(buz, LOW);
  delay(500);
  digitalWrite(buz, HIGH);
  delay(1000);
  digitalWrite(buz, LOW);
}

//=====================================================================================  LED  ===================================================================================

void red_led()
{
  Serial.println("Red Led Glowing");
  digitalWrite(Rled, HIGH);
}


void green_led()
{
  Serial.println("Green Led Glwoing");
  digitalWrite(Gled, HIGH);
}


//==================================================================================  IR Sensor  ================================================================================

void irsensordespensor() //despensing gate way
{
  Serial.println("Medicines Dispensing");
//  Serial.println("");
  buzzReady();
  red_led();
  delay(500);
  int statusSensor = digitalRead(IRH); // reads the IR sensor input  
  if (statusSensor == 0)
     motor4();
  else
    delay(3000);
    Serial.println("The pills are ready please collect them");
  buzzDone();
  digitalWrite(Rled, LOW);
  Serial.println("Red Led Stopped Glowing");
  green_led();
  delay(500);
  digitalWrite(Gled, LOW);
  Serial.println("Green Led Stopped Glowing");
}

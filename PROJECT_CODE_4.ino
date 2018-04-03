
#include <Servo.h>
#include <EEPROM.h>
#include <uSTimer2.h>
#include <CharliePlexM.h>
#include <Wire.h>


Servo servo_RightMotor;
Servo servo_LeftMotor;
Servo servo_ArmMotor;
Servo servo_BridgeMotor;//180 for open/closing drawbridge
Servo servo_ClampMotor;//180 for closing the claw
Servo servo_PushMotor;//for tilting the pyramid


//#define DEBUG_MOTORS

//#define DEBUG_ULTRASONIC

//#define DEBUG_MOTOR_CALIBRATION

#define NOFIELD 505L
#define TOMILLIGAUSS 1953L  // For A1301: 2.5mV = 1Gauss, and 1024 analog steps = 5V, so 1 step = 1953mG
// #define TOMILLIGAUSS 3756L  // For A1302: 1.3mV = 1Gauss, and 1024 analog steps = 5V, so 1 step = 3756mG



//port pin constants
//echo is output
const int ci_Ultrasonic_Ping_Front = 2;   //input plug
const int ci_Ultrasonic_Front_Data = 3;   //output plug
const int ci_Ultrasonic_Ping_Side_Front = 4;   //input plug/trigger
const int ci_Ultrasonic_Side_Front_Data = 5;   //output plug/echo
const int ci_Ultrasonic_Ping_Side_Back = 6;   //input plug/trigger
const int ci_Ultrasonic_Side_Back_Data = 7;   //output plug/echo make it 7

const int ci_Right_Motor = 8;
const int ci_Left_Motor = 9;
const int ci_Arm_Motor = 10;//clamp arm
const int ci_Bridge_Motor = 11;//180
const int ci_Clamp_Motor = 12;//180
const int txrx = 13;
const int ci_Push_Motor = 15;


//constants

// EEPROM addresses




const int ci_Left_Motor_Stop = 1500;        // 200 for brake mode; 1500 for stop
const int ci_Right_Motor_Stop = 1500;
const int ci_Push_Motor_Stop = 1500;        // 200 for brake mode; 1500 for stop
const int ci_Arm_Motor_Stop = 1500;
const int ci_Clamp_Motor_Open = 0;   
const int ci_Bridge_Motor_Down = 50; // Experiment to determine appropriate value
const int ci_Clamp_Motor_Closed = 150;        //  "
const int ci_Bridge_Motor_Up = 140;      //  "
const int ci_Display_Time = 500;


//variables

unsigned long ul_Echo_Time_Front;
unsigned long ul_Echo_Time_Side_Front;
unsigned long ul_Echo_Time_Side_Back;

unsigned int ui_Motors_Speed = 1900;        // Default run speed
unsigned int ui_Left_Motor_Speed;
unsigned int ui_Right_Motor_Speed;
unsigned int ui_Left_Motor_Speed_Reverse;
unsigned int ui_Right_Motor_Speed_Reverse;



unsigned long ul_3_Second_timer = 0;
unsigned long ul_Display_Time;
unsigned long Turn_Timer=0;
unsigned long Straight_Timer=0;
unsigned long Spin_Timer=0;


long gauss;
unsigned long halleffectcount;
unsigned long stayout;

unsigned int  ui_Robot_State_Index = 0;
bool Turn_Time_Up=false;
bool Straight_Time_Up = false;
bool Spin_Time_Up = false;
bool bt_3_S_Time_Up = false;
bool bt_Cal_Initialized = false;


void setup() {
  Wire.begin();        // Wire library required for I2CEncoder library
  Serial.begin(9600);

  // set up ultrasonics
  pinMode(ci_Ultrasonic_Ping_Front, OUTPUT);
  pinMode(ci_Ultrasonic_Front_Data, INPUT);
  pinMode(ci_Ultrasonic_Ping_Side_Front, OUTPUT);
  pinMode(ci_Ultrasonic_Side_Front_Data, INPUT);
  pinMode(ci_Ultrasonic_Ping_Side_Back, OUTPUT);
  pinMode(ci_Ultrasonic_Side_Back_Data, INPUT);

  //setup txrx

  pinMode(txrx,INPUT);
  
  // set up full revolution motors
  pinMode(ci_Right_Motor, OUTPUT);
  servo_RightMotor.attach(ci_Right_Motor);
  pinMode(ci_Left_Motor, OUTPUT);
  servo_LeftMotor.attach(ci_Left_Motor);
  pinMode(ci_Arm_Motor, OUTPUT);
  servo_ArmMotor.attach(ci_Arm_Motor);
  pinMode(ci_Push_Motor, OUTPUT);
  servo_PushMotor.attach(ci_Push_Motor);
  
  // set up 180 degree motors
  pinMode(ci_Bridge_Motor, OUTPUT);
  servo_BridgeMotor.attach(ci_Bridge_Motor);
  pinMode(ci_Clamp_Motor, OUTPUT);
  servo_ClampMotor.attach(ci_Clamp_Motor);


}

void Ping_Front()//for front ultrasonic
{
  //Ping Ultrasonic
  //Send the Ultrasonic Range Finder a 10 microsecond pulse per tech spec
  digitalWrite(ci_Ultrasonic_Ping_Front, HIGH);
  delayMicroseconds(10);  //The 10 microsecond pause where the pulse in "high"
  digitalWrite(ci_Ultrasonic_Ping_Front, LOW);
  //use command pulseIn to listen to Ultrasonic_Data pin to record the
  //time that it takes from when the Pin goes HIGH until it goes LOW
  ul_Echo_Time_Front = pulseIn(ci_Ultrasonic_Front_Data, HIGH, 10000);

  // Print Sensor Readings
  
#ifdef DEBUG_ULTRASONIC
  Serial.print("Time (microseconds): ");
  Serial.print(ul_Echo_Time_Front, DEC);
  Serial.print(", Inches: ");
  Serial.print(ul_Echo_Time_Front / 148); //divide time by 148 to get distance in inches
  Serial.print(", cm: ");
  Serial.println(ul_Echo_Time_Front / 58); //divide time by 58 to get distance in cm
#endif

}

void Ping_Side_Front()//for front side ultrasonic
{
  //Ping Ultrasonic
  //Send the Ultrasonic Range Finder a 10 microsecond pulse per tech spec
  digitalWrite(ci_Ultrasonic_Ping_Side_Front, HIGH);
  delayMicroseconds(10);  //The 10 microsecond pause where the pulse in "high"
  digitalWrite(ci_Ultrasonic_Ping_Side_Front, LOW);
  //use command pulseIn to listen to Ultrasonic_Data pin to record the
  //time that it takes from when the Pin goes HIGH until it goes LOW
  ul_Echo_Time_Side_Front = pulseIn(ci_Ultrasonic_Side_Front_Data, HIGH, 10000);

  // Print Sensor Readings
#ifdef DEBUG_ULTRASONIC
  Serial.print("Time (microseconds): ");
  Serial.print(ul_Echo_Time_Side_Front, DEC);
  Serial.print(", Inches: ");
  Serial.print(ul_Echo_Time_Side_Front / 148); //divide time by 148 to get distance in inches
  Serial.print(", cm: ");
  Serial.println(ul_Echo_Time_Side_Front / 58); //divide time by 58 to get distance in cm
#endif
}

void Ping_Side_Back()//for Back side ultrasonic
{
  //Ping Ultrasonic
  //Send the Ultrasonic Range Finder a 10 microsecond pulse per tech spec
  digitalWrite(ci_Ultrasonic_Ping_Side_Back, HIGH);
  delayMicroseconds(10);  //The 10 microsecond pause where the pulse in "high"
  digitalWrite(ci_Ultrasonic_Ping_Side_Back, LOW);
  //use command pulseIn to listen to Ultrasonic_Data pin to record the
  //time that it takes from when the Pin goes HIGH until it goes LOW
  ul_Echo_Time_Side_Back = pulseIn(ci_Ultrasonic_Side_Back_Data, HIGH, 10000);

  // Print Sensor Readings
  
  
#ifdef DEBUG_ULTRASONIC
  Serial.print("Time (microseconds): ");
  Serial.print(ul_Echo_Time_Side_Back, DEC);
  Serial.print(", Inches: ");
  Serial.print(ul_Echo_Time_Side_Back / 148); //divide time by 148 to get distance in inches
  Serial.print(", cm: ");
  Serial.println(ul_Echo_Time_Side_Back / 58); //divide time by 58 to get distance in cm
#endif

}

void DoMeasurement()
{
// measure magnetic field
  int raw = analogRead(0);   // Range : 0..1024

//  Uncomment this to get a raw reading for calibration of no-field point
  Serial.print("Raw reading: ");
  Serial.println(raw);

  long compensated = raw - NOFIELD;                 // adjust relative to no applied field 
  gauss = compensated * TOMILLIGAUSS / 1000;   // adjust scale to Gauss

  Serial.print(gauss);
  Serial.print(" Gauss ");

  if (gauss > 0)     Serial.println("(South pole)");
  else if(gauss < 0) Serial.println("(North pole)");
  else               Serial.println();
}


void loop()
{
  if ((millis() - ul_3_Second_timer) > 3000)
  {
    bt_3_S_Time_Up = true;
  }

  
  switch (ui_Robot_State_Index)
  {
    case 0:    //Robot stopped
      {
        
        Ping_Front();
        Ping_Side_Front();
        Ping_Side_Back();
        
        servo_LeftMotor.writeMicroseconds(ci_Left_Motor_Stop);
        servo_RightMotor.writeMicroseconds(ci_Right_Motor_Stop);
     
        servo_PushMotor.writeMicroseconds(ci_Push_Motor_Stop);
        servo_ArmMotor.writeMicroseconds(ci_Arm_Motor_Stop);
        servo_ClampMotor.write(ci_Clamp_Motor_Open);
        servo_BridgeMotor.write(ci_Bridge_Motor_Up);
     
        Serial.print("Mode:0");
        ui_Robot_State_Index=0;
        
        if (bt_3_S_Time_Up)
        {
        ui_Robot_State_Index=1;
        }
         
        break;
      }

    case 1:    //Robot Run after 3 seconds
      {
        
Serial.print("Mode:1");
DoMeasurement();



Ping_Front();
Ping_Side_Front();
Ping_Side_Back();

//FIND PROPER LENGTHS FOR CM FROM WALL*****
 

if(ul_Echo_Time_Side_Front/58<=4)//when it gets stuck
{
   servo_RightMotor.writeMicroseconds(ci_Right_Motor_Stop);
  servo_LeftMotor.writeMicroseconds(1200);
  
}
else
{
if((ul_Echo_Time_Front/58>15)&&(ul_Echo_Time_Side_Front/58<=8)&&(ul_Echo_Time_Side_Back/58<=8)&&(ul_Echo_Time_Side_Front/58>=6)&&(ul_Echo_Time_Side_Back/58>=6))//if not at wall and sides are between 7 and 9 away, go straight 
{
  
   servo_LeftMotor.writeMicroseconds(1900);//go straight 
   servo_RightMotor.writeMicroseconds(1900);
   
}
if((ul_Echo_Time_Side_Front/58>8)&&(ul_Echo_Time_Side_Back/58>8)&&(ul_Echo_Time_Front/58>15))//if both greater then 9 right
{
  servo_LeftMotor.writeMicroseconds(1900);
   servo_RightMotor.writeMicroseconds(1700);
   
}
if((ul_Echo_Time_Side_Front/58<6)&&(ul_Echo_Time_Side_Back/58<6)&&(ul_Echo_Time_Front/58>15))//if both less then 7 left
{
  servo_LeftMotor.writeMicroseconds(1700);
   servo_RightMotor.writeMicroseconds(1900);

}
if((ul_Echo_Time_Front/58>15)&&(ul_Echo_Time_Side_Front/58>8)&&(ul_Echo_Time_Side_Back/58<6))//if front greater then 8 and back less then 8 right
{
servo_LeftMotor.writeMicroseconds(1900);
   servo_RightMotor.writeMicroseconds(1700);

}
if((ul_Echo_Time_Front/58>15)&&(ul_Echo_Time_Side_Front/58<6)&&(ul_Echo_Time_Side_Back/58>8))//if back is greater and front is less left
{
servo_LeftMotor.writeMicroseconds(1700);
   servo_RightMotor.writeMicroseconds(1900);

  
}
if((ul_Echo_Time_Front/58<15)&&(ul_Echo_Time_Side_Front/58>2))//corners
{
  
 servo_RightMotor.writeMicroseconds(2200);
 servo_LeftMotor.writeMicroseconds(1200);

}
if((ul_Echo_Time_Front/58<15)&&(((ul_Echo_Time_Side_Front/58)-(ul_Echo_Time_Side_Back/58))>3))//for corners 
{
  
 servo_RightMotor.writeMicroseconds(2200);
 servo_LeftMotor.writeMicroseconds(1200);


}

}

if (((gauss >80) || (gauss < 40))&& stayout == 0){
           halleffectcount++;
 }
 
 if (((gauss >80) || (gauss < 40))&& halleffectcount > 8){
  
           servo_RightMotor.writeMicroseconds(ci_Right_Motor_Stop);
           servo_LeftMotor.writeMicroseconds(ci_Left_Motor_Stop);
           stayout = 1;
           servo_ClampMotor.write(ci_Clamp_Motor_Closed);
           Serial.println("GOTEM");
           delay(1000);
           servo_PushMotor.writeMicroseconds(1260);
           Serial.println("started");
           delay(2300);
           Serial.println("waited");
           servo_PushMotor.writeMicroseconds(ci_Push_Motor_Stop);
           Serial.println("stoped");
         
           ui_Robot_State_Index = 2;
           Serial.println("mode changed");
         
           halleffectcount = 0;
           
               
 }

#ifdef DEBUG_MOTORS
          Serial.print("Motors enabled: ");
          Serial.print(bt_Motors_Enabled);
          Serial.print(", Default: ");
          Serial.print(ui_Motors_Speed);
          Serial.print(", Left = ");
          Serial.print(ui_Left_Motor_Speed);
          Serial.print(", Right = ");
          Serial.println(ui_Right_Motor_Speed);
#endif
     //  ul_3_Second_timer=millis();
        
        break;
      }
    case 2:    //go to corner 
      {
        Ping_Front();
        Serial.println("Mode:2");
        if((ul_Echo_Time_Front/58<=15)&&(ul_Echo_Time_Front/58>1))
        {
          servo_LeftMotor.writeMicroseconds(ci_Left_Motor_Stop);
        servo_RightMotor.writeMicroseconds(ci_Right_Motor_Stop);
        ul_3_Second_timer=millis();
        Turn_Timer=millis();
        ui_Robot_State_Index=3;
        }
        else
        {
          servo_LeftMotor.writeMicroseconds(1900);//go straight 
         servo_RightMotor.writeMicroseconds(1900);
        }
        break;
      }
       case 3:
       {
        Serial.println("Mode:3");
        Ping_Front();
          
  if ((millis() - Turn_Timer) > 2000)//timer for turning
  {
    Turn_Time_Up = true;
  }
  if ((millis() - Straight_Timer) > 3000)//timer for going straight 
  {
    Straight_Time_Up = true;
  }
   if ((millis() - Spin_Timer) > 3000)//timer for going straight 
  {
    Spin_Time_Up = true;
  }
  
         
        if(digitalRead(13)==HIGH)
        {
          Serial.println("FOUND PYRAMID");
        servo_LeftMotor.writeMicroseconds(ci_Left_Motor_Stop);
        servo_RightMotor.writeMicroseconds(ci_Right_Motor_Stop);
        ui_Robot_State_Index=4;
        }
        //searching code starts 
        if(Turn_Time_Up==false)//turn until turn timer is up
        {
          Serial.println("Small Turn");
        servo_LeftMotor.writeMicroseconds(1300);
        servo_RightMotor.writeMicroseconds(1800);
        Straight_Timer = millis();//reset straight timer
        Spin_Timer = millis();
        Straight_Time_Up=false;
      
        }
        else
        {
        if(Straight_Time_Up==false)//straight until straight timer up 
        {
           servo_LeftMotor.writeMicroseconds(1700);//go straight 
           servo_RightMotor.writeMicroseconds(1700);
           Serial.println("FORWARD");
          Spin_Timer=millis();
           
        }
         if((Straight_Time_Up)&&(!(Spin_Time_Up)))    //were going to need to slow down the speed and increase the time to read properly 
        {
          servo_LeftMotor.writeMicroseconds(1300);
           servo_RightMotor.writeMicroseconds(1700);
           Serial.println("BIG TURN");
        }
        
        if(Spin_Time_Up)//if spin time up, reset straight timer 
        {
          Straight_Timer = millis();
          Serial.println("LOOP");
          Spin_Time_Up=false;
        }
        
        }
        break;
      }
  
      case 4:
    {
      Serial.println("Mode:4");
Ping_Front();
if((digitalRead(13)==HIGH)&&(ul_Echo_Time_Front/58<=2))//if it senses pyramid
{
  
   servo_LeftMotor.writeMicroseconds(ci_Left_Motor_Stop);//go straight 
   servo_RightMotor.writeMicroseconds(ci_Right_Motor_Stop);
   servo_BridgeMotor.write(ci_Bridge_Motor_Down);
   servo_ArmMotor.writeMicroseconds(1600);
   delay(2000);
    servo_ArmMotor.writeMicroseconds(ci_Arm_Motor_Stop);
    servo_ClampMotor.write(ci_Clamp_Motor_Open);
delay(1000);
    servo_ArmMotor.writeMicroseconds(1400);
    delay(2000);
    ui_Robot_State_Index=5;
   }
   
if((digitalRead(13)==HIGH)&&(ul_Echo_Time_Front/58>2))//getting in reach      this will not work w/o encoders (need to sweep)
{
   servo_LeftMotor.writeMicroseconds(1900);//go straight 
   servo_RightMotor.writeMicroseconds(1900);
}

    
        break;
      }
  


   case 5:    //finished 
      {
 
        Serial.print("Mode:5");
        servo_LeftMotor.writeMicroseconds(ci_Left_Motor_Stop);
        servo_RightMotor.writeMicroseconds(ci_Right_Motor_Stop);
     
        servo_PushMotor.writeMicroseconds(ci_Push_Motor_Stop);
        servo_ArmMotor.writeMicroseconds(ci_Arm_Motor_Stop);
        servo_ClampMotor.write(ci_Clamp_Motor_Open);
        servo_BridgeMotor.write(ci_Bridge_Motor_Down);
      

        break;
      }
  }

  if ((millis() - ul_Display_Time) > ci_Display_Time)
  {
    ul_Display_Time = millis();
  }
}


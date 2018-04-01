
#include <Servo.h>
#include <EEPROM.h>
#include <uSTimer2.h>
#include <CharliePlexM.h>
#include <Wire.h>
#include <I2CEncoder.h>

Servo servo_RightMotor;
Servo servo_LeftMotor;
Servo servo_ArmMotor;
Servo servo_BridgeMotor;//180 for open/closing drawbridge
Servo servo_ClampMotor;//180 for closing the claw
Servo servo_PushMotor;//for tilting the pyramid

I2CEncoder encoder_RightMotor;
I2CEncoder encoder_LeftMotor;


//#define DEBUG_MOTORS

//#define DEBUG_ENCODERS
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
const int ci_Push_Motor = 13;

const int ci_Light_Sensor_Back = A0;
const int ci_Light_Sensor_Front = A1;
const int ci_Light_Sensor_left = A2;
const int ci_Light_Sensor_right = A4;


//constants

// EEPROM addresses

const int ci_Left_Motor_Offset_Address_L = 12;
const int ci_Left_Motor_Offset_Address_H = 13;
const int ci_Right_Motor_Offset_Address_L = 14;
const int ci_Right_Motor_Offset_Address_H = 15;



const int ci_Left_Motor_Stop = 1500;        // 200 for brake mode; 1500 for stop
const int ci_Right_Motor_Stop = 1500;
const int ci_Push_Motor_Stop = 1500;        // 200 for brake mode; 1500 for stop
const int ci_Arm_Motor_Stop = 1500;
const int ci_Clamp_Motor_Open = 0;   
const int ci_Bridge_Motor_Down = 50; // Experiment to determine appropriate value
const int ci_Clamp_Motor_Closed = 145;        //  "
const int ci_Bridge_Motor_Up = 140;      //  "
const int ci_Display_Time = 500;

const int ci_Motor_Calibration_Cycles = 3;
const int ci_Motor_Calibration_Time = 5000;

//variables
byte b_LowByte;
byte b_HighByte;
unsigned long ul_Echo_Time_Front;
unsigned long ul_Echo_Time_Side_Front;
unsigned long ul_Echo_Time_Side_Back;

unsigned int ui_Motors_Speed = 1900;        // Default run speed
unsigned int ui_Left_Motor_Speed;
unsigned int ui_Right_Motor_Speed;
unsigned int ui_Left_Motor_Speed_Reverse;
unsigned int ui_Right_Motor_Speed_Reverse;
long l_Left_Motor_Position;
long l_Right_Motor_Position;


unsigned long ul_3_Second_timer = 0;
unsigned long ul_Display_Time;
unsigned long ul_Calibration_Time;
unsigned long ui_Left_Motor_Offset;
unsigned long ui_Right_Motor_Offset;

unsigned int ui_Cal_Count;
unsigned int ui_Cal_Cycle;
long gauss;
unsigned long halleffectcount;
unsigned long stayout;

unsigned int  ui_Robot_State_Index = 0;
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

  // set up encoders. Must be initialized in order that they are chained together,
  // starting with the encoder directly connected to the Arduino. See I2CEncoder docs
  // for more information
  encoder_LeftMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_LeftMotor.setReversed(false);  // adjust for positive count when moving forward
  encoder_RightMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_RightMotor.setReversed(true);  // adjust for positive count when moving forward


  b_LowByte = EEPROM.read(ci_Left_Motor_Offset_Address_L);
  b_HighByte = EEPROM.read(ci_Left_Motor_Offset_Address_H);
  ui_Left_Motor_Offset = word(b_HighByte, b_LowByte);
  b_LowByte = EEPROM.read(ci_Right_Motor_Offset_Address_L);
  b_HighByte = EEPROM.read(ci_Right_Motor_Offset_Address_H);
  ui_Right_Motor_Offset = word(b_HighByte, b_LowByte);
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
        encoder_LeftMotor.zero();
        encoder_RightMotor.zero();
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

#ifdef DEBUG_ENCODERS
          l_Left_Motor_Position = encoder_LeftMotor.getRawPosition();
          l_Right_Motor_Position = encoder_RightMotor.getRawPosition();

          Serial.print("Encoders L: ");
          Serial.print(l_Left_Motor_Position);
          Serial.print(", R: ");
          Serial.println(l_Right_Motor_Position);
#endif

          // set motor speeds
          ui_Left_Motor_Speed = constrain(ui_Motors_Speed + ui_Left_Motor_Offset, 1900, 2000);
          ui_Right_Motor_Speed = constrain(ui_Motors_Speed + ui_Right_Motor_Offset, 1900, 2000);
          ui_Left_Motor_Speed_Reverse = constrain(ui_Motors_Speed + ui_Left_Motor_Offset, 1350, 1250);
          ui_Right_Motor_Speed_Reverse = constrain(ui_Motors_Speed + ui_Right_Motor_Offset, 1350, 1250);

Ping_Front();
Ping_Side_Front();
Ping_Side_Back();

//FIND PROPER LENGTHS FOR CM FROM WALL*****
 

   
if((ul_Echo_Time_Front/58>20)&&(ul_Echo_Time_Side_Front/58<=9)&&(ul_Echo_Time_Side_Back/58<=9)&&(ul_Echo_Time_Side_Front/58>=7)&&(ul_Echo_Time_Side_Back/58>=7))//if not at wall and sides are between 7 and 9 away, go straight 
{
  
   servo_LeftMotor.writeMicroseconds(1900);//go straight 
   servo_RightMotor.writeMicroseconds(1900);
}
if((ul_Echo_Time_Side_Front/58>9)&&(ul_Echo_Time_Side_Back/58>9)&&(ul_Echo_Time_Front/58>20))//if both greater then 9 right
{
  servo_LeftMotor.writeMicroseconds(1900);
   servo_RightMotor.writeMicroseconds(1700);
}
if((ul_Echo_Time_Side_Front/58<7)&&(ul_Echo_Time_Side_Back/58<7)&&(ul_Echo_Time_Front/58>20))//if both less then 7 left
{
  servo_LeftMotor.writeMicroseconds(1700);
   servo_RightMotor.writeMicroseconds(1900);
}
if((ul_Echo_Time_Front/58>20)&&(ul_Echo_Time_Side_Front/58>8)&&(ul_Echo_Time_Side_Back/58<6))//if front greater then 8 and back less then 8 right
{
servo_LeftMotor.writeMicroseconds(1900);
   servo_RightMotor.writeMicroseconds(1700);
}
if((ul_Echo_Time_Front/58>20)&&(ul_Echo_Time_Side_Front/58<6)&&(ul_Echo_Time_Side_Back/58>8))//if back is greater and front is less left
{
servo_LeftMotor.writeMicroseconds(1700);
   servo_RightMotor.writeMicroseconds(1900);
}
if((ul_Echo_Time_Front/58<30)&&((ul_Echo_Time_Side_Front/58)-(ul_Echo_Time_Side_Back/58)>3))//for courners 
{
  
 servo_RightMotor.writeMicroseconds(2200);
 servo_LeftMotor.writeMicroseconds(1200);

}
if((ul_Echo_Time_Front/58<20))//courners
{
  
 servo_RightMotor.writeMicroseconds(2200);
 servo_LeftMotor.writeMicroseconds(1200);

}
if((ul_Echo_Time_Side_Front/58)<2 && (ul_Echo_Time_Side_Back/58)>4)//for when it gets stuck 
{
  
 servo_RightMotor.writeMicroseconds(ci_Right_Motor_Stop);
 servo_LeftMotor.writeMicroseconds(1150);

}

if (((gauss >150) || (gauss < 0))&& stayout == 0){
           halleffectcount++;
 }
 
 if (((gauss >150) || (gauss < 0))&& halleffectcount > 8){
  
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
           servo_ClampMotor.write(ci_Clamp_Motor_Open);  
           Serial.println("opened");
           ui_Robot_State_Index = 0;
           Serial.println("mode changed");
           delay(1000);
           servo_PushMotor.writeMicroseconds(1730);
           Serial.println("started");
           delay(2000);
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
       ul_3_Second_timer=millis();
        
        break;
      }
    case 2:    //testing code for now 
      {
if(bt_3_S_Time_Up)
{
     /*  
   servo_BridgeMotor.write(ci_Bridge_Motor_Down);
   servo_PushMotor.writeMicroseconds(1700);
   servo_ArmMotor.writeMicroseconds(1600);
   delay(4000);
   servo_PushMotor.writeMicroseconds(ci_Push_Motor_Stop);
   servo_ArmMotor.writeMicroseconds(ci_Arm_Motor_Stop);
   delay(1000);
   servo_PushMotor.writeMicroseconds(1300);
   servo_ArmMotor.writeMicroseconds(1400);
   delay(4000);
   servo_PushMotor.writeMicroseconds(ci_Push_Motor_Stop);
   servo_ArmMotor.writeMicroseconds(ci_Arm_Motor_Stop);
   delay(1000);
   */
    }
        break;
      }

     case 3:    //Calibrate motor straightness after 3 seconds.
      {
       
          if (!bt_Cal_Initialized)
          {
            bt_Cal_Initialized = true;
            encoder_LeftMotor.zero();
            encoder_RightMotor.zero();
            ul_Calibration_Time = millis();
            servo_LeftMotor.writeMicroseconds(ui_Motors_Speed);
            servo_RightMotor.writeMicroseconds(ui_Motors_Speed);
          }
          else if ((millis() - ul_Calibration_Time) > ci_Motor_Calibration_Time)
          {
            servo_LeftMotor.writeMicroseconds(ci_Left_Motor_Stop);
            servo_RightMotor.writeMicroseconds(ci_Right_Motor_Stop);
            l_Left_Motor_Position = encoder_LeftMotor.getRawPosition();
            l_Right_Motor_Position = encoder_RightMotor.getRawPosition();
            if (l_Left_Motor_Position > l_Right_Motor_Position)
            {
              // May have to update this if different calibration time is used
              ui_Right_Motor_Offset = 0;
              ui_Left_Motor_Offset = (l_Left_Motor_Position - l_Right_Motor_Position) / 4;
            }
            else
            {
              // May have to update this if different calibration time is used
              ui_Right_Motor_Offset = (l_Right_Motor_Position - l_Left_Motor_Position) / 4;
              ui_Left_Motor_Offset = 0;
            }

#ifdef DEBUG_MOTOR_CALIBRATION
            Serial.print("Motor Offsets: Left = ");
            Serial.print(ui_Left_Motor_Offset);
            Serial.print(", Right = ");
            Serial.println(ui_Right_Motor_Offset);
#endif
            EEPROM.write(ci_Right_Motor_Offset_Address_L, lowByte(ui_Right_Motor_Offset));
            EEPROM.write(ci_Right_Motor_Offset_Address_H, highByte(ui_Right_Motor_Offset));
            EEPROM.write(ci_Left_Motor_Offset_Address_L, lowByte(ui_Left_Motor_Offset));
            EEPROM.write(ci_Left_Motor_Offset_Address_H, highByte(ui_Left_Motor_Offset));

            ui_Robot_State_Index = 0;    // go back to Mode 0
          }
#ifdef DEBUG_MOTOR_CALIBRATION
          Serial.print("Encoders L: ");
          Serial.print(encoder_LeftMotor.getRawPosition());
          Serial.print(", R: ");
          Serial.println(encoder_RightMotor.getRawPosition());
#endif
         ui_Robot_State_Index=0;
        
        break;
      }
  }

  if ((millis() - ul_Display_Time) > ci_Display_Time)
  {
    ul_Display_Time = millis();
  }
}


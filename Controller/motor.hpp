#ifndef INCLUDE_MOTOR_HPP_
#define INCLUDE_MOTOR_HPP_

#include <Arduino.h>
#include <PCA9685.h>
#include <PCF8574.h>

#define MOTOR_1_DIRECTION     -1 //If the direction is reversed, change 1 to -1
#define MOTOR_2_DIRECTION     -1 //If the direction is reversed, change 1 to -1
#define MOTOR_3_DIRECTION     -1 //If the direction is reversed, change 1 to -1
#define MOTOR_4_DIRECTION     -1 //If the direction is reversed, change 1 to -1

void PCA9685_Close_Com_Address(void);//Close the PCA9685 public address

/////////////////////PCA9685 drive area//////////////////////////////////////
void PCA9685_Setup(void);              //servo initialization
void Servo_1_Angle(float angle);       //Set the rotation parameters of servo 1, and the parameters are 0-180 degrees
void Servo_2_Angle(float angle);       //Set the rotation parameters of servo 2, and the parameters are 0-180 degrees
void Servo_Sweep(int servo_id, int angle_start, int angle_end, int speed);//Servo sweep function;
void Motor_Move(int m1_speed, int m2_speed, int m3_speed, int m4_speed);  //A function to control the car motor

//////////////////////Buzzer drive area////////////////////////////////////
#define PIN_BUZZER 2                    //Define the pins for the ESP32 control buzzer
#define BUZZER_CHN 0                    //Define the PWM channel for ESP32
#define BUZZER_FREQUENCY 2000           //Define the resonant frequency of the buzzer 

void Buzzer_Setup(void);                //Buzzer initialization
void Buzzer_Alert(int beat, int rebeat);//Buzzer alarm function

////////////////////Battery drive area/////////////////////////////////////
int Get_Battery_Voltage_ADC(void);     //Gets the battery ADC value
float Get_Battery_Voltage(void);       //Get the battery voltage value
void Set_Battery_Coefficient(float coefficient);//Set the partial pressure coefficient

////////////////////Photosensitive drive area//////////////////////////////
void Photosensitive_Setup(void);       //Photosensitive initialization
int Get_Photosensitive(void);          //Gets the photosensitive resistance value

/////////////////////Ultrasonic drive area/////////////////////////////////
void Ultrasonic_Setup(void);           //Ultrasonic initialization
float Get_Sonar(void);                 //Obtain ultrasonic distance data

/////////////////////PCF8574 drive area//////////////////////////////
extern unsigned char sensorValue[4];
void Track_Setup(void);               //Trace module initialization
void Track_Read(void);                //Tracking module reading


//////////////////////Emotion drive area////////////////////////////////

void Emotion_Setup();                            //Initialize
void eyesRotate(int delay_ms);                   //Turn the eyes-1
void eyesBlink(int delay_ms);                    //Wink the eyes-2
void eyesSmile(int delay_ms);                    //Smile-3
void eyesCry(int delay_ms);                      //Cry-4
void eyesBlink1(int delay_ms);                   //Wink the eyes-5
void showArrow(int arrow_direction,int delay_ms);//Arrow-6
void wheel(int mode,int delay_ms);               //wheel-7
void carMove(int mode,int delay_ms);             //car-8
void expressingLove(void);                       //expressing love-9
void saveWater(int delay_ms);                    //save water-10







#endif
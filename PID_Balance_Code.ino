/* PID balance code with ping pong ball and distance sensor sharp 2y0a21
 *  by ELECTRONOOBS: https://www.youtube.com/channel/UCjiVhIvGmRZixSzupD0sS9Q
 *  Tutorial: http://electronoobs.com/eng_arduino_tut100.php
 *  Code: http://electronoobs.com/eng_arduino_tut100_code1.php
 *  Scheamtic: http://electronoobs.com/eng_arduino_tut100_sch1.php 
 *  3D parts: http://electronoobs.com/eng_arduino_tut100_stl1.php   
 */
#include <Wire.h>
#include <Servo.h>

#include "Adafruit_VL53L0X.h"

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

///////////////////////Inputs/outputs///////////////////////
int Analog_in = A0;
Servo myservo;  // create servo object to control a servo, later attatched to D9
///////////////////////////////////////////////////////



////////////////////////Variables///////////////////////
float distance = 0.0;
float time;        //Variables for time control
float distance_previous_error, distance_error;
int period = 120;  //Refresh rate period of the loop is 120ms
///////////////////////////////////////////////////////


///////////////////PID constants///////////////////////

float kp_orig=8; //Mine was 8
float ki_orig=0.2; //Mine was 0.2
float kd_orig=5000; //Mine was 3100

float kp=kp_orig; //Mine was 8
float ki=ki_orig; //Mine was 0.2
float kd=kd_orig; //Mine was 3100
float distance_setpoint = 36;         // 36 Should be the distance from sensor to the middle of the bar in mm

float PID_p, PID_i, PID_d, PID_total;
///////////////////////////////////////////////////////

boolean debug = true;
int num_consecutive_times_close_to_center = 0;
int center_servo_angle = 125;

void setup() {
  //analogReference(EXTERNAL);
  Serial.begin(115200);

  // wait until serial port opens for native USB devices
  while (! Serial) {
    delay(1);
  }
  
  Serial.println("Adafruit VL53L0X test");
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1);
  }
  // power 
  Serial.println(F("VL53L0X API Simple Ranging example\n\n")); 

  
  myservo.attach(9); // attaches the servo on pin 9 to the servo object
  myservo.write(0);
  delay(1000);
  myservo.write(170);
  delay(1000);
  pinMode(Analog_in,INPUT);  
  time = millis();
}

void loop() {
  while (true) {
    if (millis() > time+period)
    {
      time = millis();
      VL53L0X_RangingMeasurementData_t measure;
  
      lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
    
      if (measure.RangeStatus != 4) 
      { // phase failures have incorrect data
        distance = measure.RangeMilliMeter / 10;
        if (distance > 55)
        {
          distance = 55;
        }
      } 
      else 
      {
        continue;
      }
      
      if (debug) {Serial.print("Distance = "); Serial.println(distance);}
      
      distance_error = distance_setpoint - distance - 4;

      PID_p = kp * distance_error;
      float dist_diference = distance_error - distance_previous_error;     
        
      if(-3 < distance_error && distance_error < 3)
      {
        PID_i = PID_i + (ki * distance_error);
      }
      else
      {
        PID_i = 0;
      }

      PID_d = kd*((distance_error - distance_previous_error)/period);

      if(abs(distance_error) < 5 && abs(PID_d) < 175)
      {
        num_consecutive_times_close_to_center += 1;
        if(num_consecutive_times_close_to_center >= 3)
        {
          Serial.println("Stop!");
          myservo.write(center_servo_angle);
          break;
        }
        else
        {
          float factor = .7;
          kp *= factor;
          ki *= factor;
          kd *= factor;
        }
      }
      else
      {
        num_consecutive_times_close_to_center = 0;
        kp=kp_orig;
        ki=ki_orig;
        kd=kd_orig;
      }
      
    
      PID_total = PID_p + PID_i + PID_d;
      PID_total = map(PID_total, -240, 240, 0, 100);
    
      if(PID_total < 10){PID_total = 10;}
      if(PID_total > 90) {PID_total = 90;}

      int final_pid_total = 180-PID_total;
  
      myservo.write(final_pid_total);
      
      distance_previous_error = distance_error;
      if (debug) {delay(50);}
    }
    
  }
  
}

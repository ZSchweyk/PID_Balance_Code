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
int Read = 0;
float distance = 0.0;
float elapsedTime, time, timePrev;        //Variables for time control
float distance_previous_error, distance_error;
int period = 120;  //Refresh rate period of the loop is 50ms
///////////////////////////////////////////////////////


///////////////////PID constants///////////////////////
float kp=8; //Mine was 8
float ki=0.2; //Mine was 0.2
float kd=5000; //Mine was 3100
float distance_setpoint = 36;         //Should be the distance from sensor to the middle of the bar in mm
float PID_p, PID_i, PID_d, PID_total;
///////////////////////////////////////////////////////

boolean debug = true;
int num_consecutive_times_close_to_center = 0;

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




  
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  myservo.write(0);
  delay(1000);
  myservo.write(170); //Put the servco at angle 125, so the balance is in the middle
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

      
      
      if (debug) {Serial.print("\t\t\tDistance Error = "); Serial.println(distance_error);}
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

      if (debug) {Serial.print("PID_d = "); Serial.println(PID_d);}

      if(abs(distance_error) < 6 && abs(PID_d) < 175)
      {
        num_consecutive_times_close_to_center += 1;
        if(num_consecutive_times_close_to_center >= 3)
        {
          Serial.println("Stop!");
          myservo.write(125);
          return;
        }
      }
      else
      {
        num_consecutive_times_close_to_center = 0;
      }

      
    
      PID_total = PID_p + PID_i + PID_d;  
      PID_total = map(PID_total, -150, 150, 0, 100);
    
      if(PID_total < 10){PID_total = 10;}
      if(PID_total > 90) {PID_total = 90;}

      int final_pid_total = 180-PID_total;
  
      myservo.write(final_pid_total);
      
      if (debug) {Serial.print("Final PID Total = "); Serial.println(final_pid_total);}
      distance_previous_error = distance_error;
      if (debug) {delay(50);}
    }
    
  }
  
}


float get_dist(int n)
{
  long sum=0;
  for(int i=0;i<n;i++)
  {
    sum=sum+analogRead(Analog_in);
  }  
  //Serial.print("One Time Reading: "); Serial.println(analogRead(Analog_in));
  float adc=sum/n;
  //float volts = analogRead(adc)*0.0048828125;  // value from sensor * (5/1024)
  //float volts = sum*0.003222656;  // value from sensor * (3.3/1024) EXTERNAL analog refference

  float distance_cm = 17569.7 * pow(adc, -1.2062);
  //float distance_cm = 13*pow(volts, -1); 
  Serial.print("Distance: "); Serial.println(distance_cm);
  return(distance_cm - 7);
}

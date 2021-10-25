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
float distance_setpoint = 35;         //Should be the distance from sensor to the middle of the bar in mm
float PID_p, PID_i, PID_d, PID_total;
///////////////////////////////////////////////////////

boolean debug = false;


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
      //                       {time, p, i, d, distance}
      time = millis();
      VL53L0X_RangingMeasurementData_t measure;
  
      //Serial.print(time); Serial.print(", ");
      //Serial.print("Reading a measurement... ");
      lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
    
      if (measure.RangeStatus != 4) 
      { // phase failures have incorrect data
//        distance = 0;
//        int n = 100;
//        for(int i = 0; i < n; i++)
//        {
//          distance += measure.RangeMilliMeter / 10;
//        }
//        distance /= n;
        distance = measure.RangeMilliMeter / 10;
        if (distance > 55) {
          distance = 55;
        }
        
        //Serial.print("Distance (cm): "); Serial.println(distance);
      } 
      else 
      {
        continue;
      }
      
      if (debug) {Serial.print("Distance = "); Serial.println(distance);}
      
      distance_error = distance_setpoint - distance;   
      if (debug) {Serial.print("\t\t\tDistance Error = "); Serial.println(distance_error);}
      PID_p = kp * distance_error;
      //Serial.print("P = "); Serial.println(PID_p);
      //Serial.print(PID_p); Serial.print(", ");
      float dist_diference = distance_error - distance_previous_error;     
      
      //Serial.print("Distance Error: "); Serial.println(distance_error);
      //Serial.print("Distance Previous Error: "); Serial.println(distance_previous_error);
      //Serial.print("Difference: "); Serial.println(distance_error - distance_previous_error);
      //delay(1500);
        
      if(-3 < distance_error && distance_error < 3)
      {
        PID_i = PID_i + (ki * distance_error);
      }
      else
      {
        PID_i = 0;
      }
      //Serial.print(PID_i); Serial.print(", ");

      PID_d = kd*((distance_error - distance_previous_error)/period);

      //Serial.print(PID_d); Serial.print(", ");

      //Serial.println(distance);
    
      PID_total = PID_p + PID_i + PID_d;  
      //Serial.print("\t\t\t\tPID_Total = "); Serial.println(PID_total);
      //PID_total = PID_d;  
      //Serial.print("PID_p: "); Serial.println(PID_p);
      //Serial.print("PID_i: "); Serial.println(PID_i);
      //Serial.print("PID_d: "); Serial.println(PID_d);
      PID_total = map(PID_total, -150, 150, 0, 90);
    
      if(PID_total < 10){PID_total = 10;}
      if(PID_total > 80) {PID_total = 80;}

      int final_pid_total = 180-PID_total;
      //final_pid_total = map(final_pid_total, 85, 145, 0, 240);
  
      //Serial.print("\t\t\t\t\t\tPID_total: "); Serial.println(final_pid_total);
      //Serial.print("Final PID Total = "); Serial.println(final_pid_total);
      myservo.write(final_pid_total);
      
      if (debug) {Serial.print("Final PID Total = "); Serial.println(final_pid_total);}
      distance_previous_error = distance_error;
      if (debug) {delay(50);}
    }

//    int start_pos = 40;
//    int end_pos = 180;
//    int increment = 5;
//
//    for(int i = start_pos; i < end_pos + 1; i += increment)
//    {
//      delay(1000);
//      Serial.print("Position = "); Serial.println(i);
//      myservo.write(i);
//    }
//    for(int i = end_pos; i > start_pos - 1; i -= increment)
//    {
//      delay(1000);
//      Serial.print("Position = "); Serial.println(i);
//      myservo.write(i);
//    }
    
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

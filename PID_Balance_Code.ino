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

float kp_orig=8;
float ki_orig=0.2;
float kd_orig=5000;

float kp=kp_orig;
float ki=ki_orig;
float kd=kd_orig;
float distance_setpoint = 36; // 36 Should be the distance from sensor to the middle of the bar in cm. Be sure to account for the 4-5 cm length of tape next to the Lidar sensor.

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
  myservo.write(0); // write 0 to the servo. This initializes the 0 position of the servo. Without this line, the servo will not know the position of any other write value, so think of this as a reference for other writes.
  delay(1000);
  myservo.write(170); // for similar reasons stated above, write 170 (the maximum possible value that I chose to write to the servo in loop())
  delay(1000);
  pinMode(Analog_in,INPUT);  
  time = millis();
}

void loop() {
  while (true) {
    if (millis() > time+period)
    {
      time = millis(); // sets time for the next loop iteration
      VL53L0X_RangingMeasurementData_t measure; // create an object to retrieve the Lidar's readings
  
      lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
    
      if (measure.RangeStatus != 4) // test if the reading from the Lidar is in range. 
      { // phase failures have incorrect data
        distance = measure.RangeMilliMeter / 10; // assign distance to the Lidar's reading (in cm)
        if (distance > 55) // if the Lidar reads a value greater than 55 cm, set distance = 55 as that is the length of the seesaw.
        {
          distance = 55;
        }
      }
      else // If the Lidar reading is not in range, move on to the next loop iteration.
      {
        continue;
      }
      
      if (debug) {Serial.print("Distance = "); Serial.println(distance);} // print the distance of the ball if debug = True
      
      distance_error = distance_setpoint - distance - 4; // find the distance error from the ball to the middle of the seesaw. I added "- 4" to account to the tape I added next to the sensor.

      PID_p = kp * distance_error; // initialize PID_p with a scaled up value of the distance error.

      // keep track of the distance covered when the ball is within 3 cm of the center.
      if(abs(distance_error) < 3)
      {
        PID_i = PID_i + (ki * distance_error);
      }
      else
      {
        PID_i = 0;
      }

      
      PID_d = kd*((distance_error - distance_previous_error)/period); // find the derivative of the ball's position (its instantaneous speed in cm/millisecond) and scale it up with the kd constant.

      // The following code makes the servo stop adjusting when the ball has relatively low speed and is close to the center 
      if(abs(distance_error) < 5 && abs(PID_d) < 175) // test if the magnitude of the distance error is < 5 AND the magnitude of the scaled speed is < 175. Feel free to change the boundaries of these tests in order to change the accuracy of where the ball ballances. Note: decreasing the minimum distance_error will make the servo take longer to stabilize. It is a trade-off for speed and accuracy.
      {
        num_consecutive_times_close_to_center += 1; // increment a consecutive counter by one
        if(num_consecutive_times_close_to_center >= 3) // test if the consecutive counter is at least 3. This ensures that the ball is more or less stable in the middle for a while. I found that 3 is the most efficient minimum number of consecutive times to check if the ball is close to the center, but feel free to change this too.
        {
          Serial.println("Stop!");
          myservo.write(center_servo_angle); // set the position of the servo such that the seesaw is horizontal to the ground (middle position).
          break; // break out of the "while True" loop. The loop() function will start up again.
        }
        else // if the consecutive counter is not >= 3, scale the kp, ki and kd constants down by 30%. Again, feel free to change this factor to whatever you see fit. I implemented this so that the servo doesn't make drastic movements when the ball is close to the center.
        {
          float factor = .7;
          kp *= factor;
          ki *= factor;
          kd *= factor;
        }
      }
      else // if the ball is not within 5 cm of the center AND it is moving relatively fast ...
      {
        num_consecutive_times_close_to_center = 0; // reset the consecutive loop counter
        // set the kp, ki and kd constants back to their original values.
        kp=kp_orig;
        ki=ki_orig;
        kd=kd_orig;
      }
      
    
      PID_total = PID_p + PID_i + PID_d; // take the sum of the p, i and d values to prepare for writing to the servo
      PID_total = map(PID_total, -240, 240, 0, 100); // map the value of PID_total from 0 to 100. This assumes that the range of PID_total is [-240, 240].

      // If for any reason PID_total falls within a range not within [-240, 240] as defined above, it will be mapped to a value outside of the range [0, 100].
      // In this case, limit the minimum of the mapped PID_total to 10 and the maximum to 90.
      if(PID_total < 10){PID_total = 10;}
      if(PID_total > 90) {PID_total = 90;}

      // Because the arm that connects the servo to the seesaw is on the opposite side of the Lidar sensor, invert the value of PID_total.
      // The servo only writes angles that are between 0 and 180.
      // Assuming that the center_servo_angle is around 125 to 130, the maximum angle of the seesaw will be approximately the same on both sides. In other words, the seesaw's maximum tilting angle on the left will be approximately equal to the right.
      int final_pid_total = 180-PID_total;
  
      myservo.write(final_pid_total); // write final_pid_total to the servo
      
      distance_previous_error = distance_error; // set distance_previous_error to distance_error for the next loop iteration.
      if (debug) {delay(50);} // wait/sleep for 50 milliseconds. I found that the servo does not have a difficult time responding without this wait, so set debug = False to skip this line if you'd like.
    }
    
  }
  
}

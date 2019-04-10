#include "Arduino.h"
#include "Math.h"


int aggressiveness = 2; //maybe changeable with potentiometer or on display?


/*
Simple torque distribution based on steering angle, throttle position brake application and geometry of vehicle

  Wheels
          Wheelbase Front track width - df
          Wheelbase Back track width - dr

  Speed
          Right motor - rm
          Left motor - lm
  
  Accelerations
          Yaw Rate - turning 
          Roll rate - sway bar 
          Pitch rate - nose diving
          Forward,backward - ax
          Left,right - ay
          Up,down
  
  Throttle (desired)
          ThrDes
  
  Turning
          Radius - R
          Inner turning angle -ita
          Outer turning angle - ota
          Steering angle - delta
          (Steer angle minus slip angle, see picture - delta)
          
  Mass - m
  
  centre of mass - cm
  
  Body side slip angle (rate) - beta(dot)
  
  Yaw rate - gamma
  
  Side slip angle front and rear - alpha1,2,3,4
  alphafl
  alphafr
  alpharl
  alpharr
  
  length to front - lf
  length to rear - lr

  
*/



void setup() {

  pinMode(throttle, OUTPUT);
  pinMode(steering, OUTPUT);
  pinMode(BRAKE, OUTPUT);
  
}

void loop() {
  
  //read brake sensor
  
  int breakValue = analogRead(BREAK);
  if (brakeValue >= 0){
  
  leftTorque = 0;
  rightTorque = 0;
  
  }
  
  
   //read desired throttle
  
  int accelerator_value = read_accelerator_value();  // Analog value from 0 to 860
  (accelerator_value >860) ? m_speed = 860 : 1; 
  float percent_torque = map(accelerator_value,0,860,0,100); // Converts analog to motor values (NM) || 100NM = 1000 in Code
  
  float left_torque = torqVec(percent_torque, 0);
  float right_torque = torqVec(percent_torque, 1);
  
}

float torqVec(float percent_torque, bool side){ // side left 0, side right 1
  
  
  // steering angle
  
  float steeringValue = read_steering_value(); //Right turn (+) Left turn (-) 
  
  if (side)
  {
  
  // Obtain individual torques
  
  leftChange = percent_torque + (steeringValue/100) * (percent_torque/aggressiveness);
  int leftTorque = (int)min(percent_torque, leftChange);
  
  }
  
  elseif (!side)
  {
      
  rightChange = percent_torque - (steeringValue/100) * (percent_torque/aggressiveness);
  int rightTorque = (int)min(percent_torque, rightChange);
  
  }
    
  
}



int read_accelerator_value()
{
  int avg_sensor = 0;
  int sensorValue_1 = analogRead(accelerator_1);
  int sensorValue_2 = analogRead(accelerator_2);
  float errorcheck = (abs(sensorValue_1 - sensorValue_2)) / sensorValue_2); // Percent error
  if (errorcheck <= 0.05) // if the error is less than 5%
  {
    avg_sensor = (sensorValue_1 + sensorValue_2) / 2;
  }
  else
  {
    println("Error accelerator reading");
  }
  return avg_sensor;
}


int read_steering_value()
{
  int sensorValue = analogRead(steering);
 
  float sensorCorrection = map(sensorValue, 0, 1023, -100, 100);
 
  return sensorCorrection;
}

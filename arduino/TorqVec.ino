#include "Arduino.h"
#include "Math.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
  
Adafruit_BNO055 bno = Adafruit_BNO055(55);

/*

Input - Throttle position, all other sensors


Setpoint - Desired Throttle, desired torque


Ouput - Throttle commands



  Wheels

          Wheelbase Front track width - df
          Wheelbase Back track width - dr

  Steering

          steer - sr

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

  roll stiffness front and rear - rhof, rhor

  

*/




float PID_error = 0;
float previous_error = 0;


unsigned float elapsedTime, Time, timePrev;
float PID_value = 0;


void setup() {

  //micros will take 11 hours to overflow, millis() takes 49 days
  //will use float and millis in order to keep precision and avoid overflow
  //use unsigned so that the range is from 0 to x, without negative values
  Time = millis();

  
  //vy = 
  //vx = CAN motor speed * radius from shaft to wheel

  v = sqrt(vx^2+vy^2);

  beta = atan(vy/vx);

  alpha1 = alpha2 = beta + (lf*gamma)/(v) - delta;

  alpha3 = alpha4 = beta - (lr*gammay)/(v);

  gammades = (v*delta)/(lf+lr);


  //yawrate error calculations
  gammaE = gammaDes - gamma;

  //Minimize this stability (S) equation
  S = fabs(gammaDes - gamma) - fabs(betaabs - beta);


  //Forces on wheels
  Fzfl = 1/2 *lr/l * m * g - rhof*ay*m*hg/df - ax*m*hg/l;

  Fzfr = 1/2 *lr/l * m * g + rhof*ay*m*hg/df - ax*m*hg/l;

  Fzrl = 1/2 *lf/l * m * g - rhor*ay*m*hg/dr + ax*m*hg/l;

  Fzrr = 1/2 *lf/l * m * g + rhor*ay*m*hg/dr + ax*m*hg/l;


  //experimental factor Kt
  Kstablerr = 4*Kt*Fzrr/m;

  Kstablerl = 4*Kt*Fzrl/m;

  Kstablefr = 4*Kt*Fzfr/m;

  Kstablefl = 4*Kt*Fzfl/m;


  //Oversteering case

  Kunstablerr = 1 + Kpprr * gammaE;

  Kunstablerl = 1 - Kpprl * gammaE;

  Kunstablefr = 1 + Kppfr * gammaE;

  Kunstablefl = 1 - Kppfl * gammaE;




}

void loop() {
  
  //Get time interval
  timePrev = Time;
  Time = millis();

  elapsedTime = (Time-timePrev);

  if (elapsedTime >= 10){ //run the pid normally with accelerometer updates
    
    
  
  
  
  //read desired throttle
  
  
  //read brake sensor
  
  
  //read steering angle
  
  
  //Obtain radius, inner and outer turning angle



        //Accelerometer data  
  
  //Orientation
  
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  
  yaw = euler.z();
  
  pitch = euler.y();
  
  roll = euler.x();
  
  //Rates
  
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  
  x = accel.x();
  
  y = accel.y();
  
  z = accel.z();
  
  
  } else { //run the algorithm without accessing the accelerometer data (too slow because only 100 HZ
    
    
    
  }


  

}

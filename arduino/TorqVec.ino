#include "Arduino.h"
#include "Math.h"

/*
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
float elapsedTime, Time, timePrev;
float PID_value = 0;


void setup() {
  // put your setup code here, to run once:

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
  // put your main code here, to run repeatedly:


  timePrev = Time;
  Time = millis();

  elapsedTime = (Time-timePrev)/1000;

  //Ackerman geometry


  // Lateral acceleration
  Ay = V ^ 2 / R;

  //Equations of motion


  

}

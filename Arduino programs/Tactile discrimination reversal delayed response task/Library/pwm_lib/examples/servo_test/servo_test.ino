/**
 ** pwm_lib library
 ** Copyright (C) 2015, 2016
 **
 **   Antonio C. Domínguez Brito <adominguez@iusiani.ulpgc.es>
 **     División de Robótica y Oceanografía Computacional <www.roc.siani.es>
 **     and Departamento de Informática y Sistemas <www.dis.ulpgc.es>
 **     Universidad de Las Palmas de Gran  Canaria (ULPGC) <www.ulpgc.es>
 **  
 ** This file is part of the pwm_lib library.
 ** The pwm_lib library is free software: you can redistribute it and/or modify
 ** it under  the  terms of  the GNU  General  Public  License  as  published  by
 ** the  Free Software Foundation, either  version  3  of  the  License,  or  any
 ** later version.
 ** 
 ** The  pwm_lib library is distributed in the hope that  it  will  be  useful,
 ** but   WITHOUT   ANY WARRANTY;   without   even   the  implied   warranty   of
 ** MERCHANTABILITY or FITNESS FOR A PARTICULAR  PURPOSE.  See  the  GNU  General
 ** Public License for more details.
 ** 
 ** You should have received a copy  (COPYING file) of  the  GNU  General  Public
 ** License along with the pwm_lib library.
 ** If not, see: <http://www.gnu.org/licenses/>.
 **/
/*
 * File: servo_test.ino 
 * Description: This is a basic example illustrating the use of li-
 * brary pwm_lib and its servo objects.
 * Date: December 20th, 2015
 * Author: Antonio C. Dominguez-Brito <adominguez@iusiani.ulpgc.es>
 * ROC-SIANI - Universidad de Las Palmas de Gran Canaria - Spain
 */

#include "pwm_lib.h"
#include "tc_lib.h"

using namespace arduino_due::pwm_lib;

#define ANGLES 8
uint32_t angles[ANGLES]= // degrees
{
  0,
  45, 
  90,
  135,
  180,
  135,
  90,
  45
};
uint32_t angle=0;
uint32_t last_angle;

#define PWM_PERIOD 2000000 // hundredth of usecs (1e-8 secs)

#define CAPTURE_TIME_WINDOW 40000 // usecs
#define ANGLE_CHANGE_TIME 5000 // msecs

servo<pwm_pin::PWML0_PB16> servo_pwm_pinDAC1; // PB16 is DUE's pin DAC1

// To measure PWM signals generated by the previous servo object, we will use
// a capture object of tc_lib library as "oscilloscope" probe, concretely: 
// capture_tc0.
// IMPORTANT: Take into account that for TC0 (TC0 and channel 0) the TIOA0 is
// PB25, which is pin 2 for Arduino DUE, so  the capture pin in  this example
// is pin 2. For the correspondence between all TIOA inputs for the different 
// TC modules, you should consult uC Atmel ATSAM3X8E datasheet in section "36. 
// Timer Counter (TC)"), and the Arduino pin mapping for the DUE.
// Mind that to meausure the servo output in this example you should connect 
// pin DAC1 to capture_tc0's pin 2.
capture_tc0_declaration();
auto& capture_pin2=capture_tc0;

void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);

  // capture_pin2 initialization
  capture_pin2.config(CAPTURE_TIME_WINDOW);

  servo_pwm_pinDAC1.start(
    PWM_PERIOD, // pwm servo period
    100000, // 1e-8 secs. (1 msecs.), minimum duty value
    200000, // 1e-8 secs. (2 msecs.), maximum duty value
    0, // minimum angle, corresponding minimum servo angle
    180, // maximum angle, corresponding minimum servo angle
    angles[angle] // initial servo angle 
  );
  last_angle=millis();
  Serial.println("********************************************************");
  Serial.print("angle "); Serial.print(angle); 
  Serial.print(": "); Serial.print(angles[angle]); 
  Serial.println(" dgs.");
  Serial.println("********************************************************");
}

void loop() {
  // put your main code here,to run repeatedly:

  delay(ANGLE_CHANGE_TIME);

  uint32_t status,duty,period;
  status=capture_pin2.get_duty_and_period(duty,period);

  Serial.println("********************************************************");  
  Serial.print("[PIN DAC1 -> PIN 2] duty: "); 
  Serial.print(
    static_cast<double>(duty)/
    static_cast<double>(capture_pin2.ticks_per_usec()),
    3
  );
  Serial.print(" usecs. period: ");
  Serial.print(
    static_cast<double>(period)/
    static_cast<double>(capture_pin2.ticks_per_usec()),
    3
  );
  Serial.println(" usecs.");

  if(millis()-last_angle>ANGLE_CHANGE_TIME)
  {
    angle=(angle+1)&0x07;
    servo_pwm_pinDAC1.set_angle(angles[angle]);
    last_angle=millis();
    Serial.println("********************************************************");
    Serial.print("angle "); Serial.print(angle); 
    Serial.print(": "); Serial.print(angles[angle]); 
    Serial.println(" dgs.");
    Serial.println("********************************************************");
  }
}


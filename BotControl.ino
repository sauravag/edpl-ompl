/*********************************************************************
* Software License Agreement (BSD License)
*
* Copyright (c) 2015, Texas A&M University
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above
* copyright notice, this list of conditions and the following
* disclaimer in the documentation and/or other materials provided
* with the distribution.
* * Neither the name of the Texas A&M University nor the names of its
* contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Shiva Gopalan */


//To check for Arduino version, and include the corresponding header file
#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <geometry_msgs/Twist.h>

//ROS node handle
ros::NodeHandle nh;

Servo servoLeft;
Servo servoRight;

// This function generates PWM signals for motors given the velocity command
void generatePWMFromVel(double v, double omega, double &pwmLeft, double &pwmRight)
{
  // a_L and a_R are the linear paramaters for velocity
  double a_L = 6900;
  double b_L = -37.63;
  double c_L = 428.3;
  double d_L = 1503;
  
  double a_R = -6741;
  double b_R = -12.26;
  double c_R = -392.2;
  double d_R = 1503;
  
  double a_omega = 0.04688;
  double b_omega = 0.01969;
  
  double v_L, v_R;
  
  if(omega==0)
  {
    v_L = v_R = v;
  }
  else
  {
    double v_turn = a_omega*omega + b_omega ;
    // right rurn
    if(omega>0)
    {
      v_L = v_turn;
      v_R = -v_turn;
    }
    else // left turn
    {
      v_L = -v_turn;
      v_R = v_turn;
    }
  }
 
  
  /**
  The equation for conversions a cubic polynomial
  */
  pwmLeft = v_L*v_L*v_L*a_L + v_L*v_L*b_L + v_L*c_L + d_L;
  
  pwmRight = v_R*v_R*v_R*a_R + v_R*v_R*b_R + v_R*c_R + d_R;
  
}


//Servo callback function
void servo_cb( const geometry_msgs::Twist& cmd_msg){
    
  double v = cmd_msg.linear.x; // forward velocity
  double omega = cmd_msg.angular.z; // yaw rate
  
  double pwmLeft, pwmRight;
  
  generatePWMFromVel(v, omega, pwmLeft, pwmRight);
  
  servoLeft.write(pwmLeft);
  servoRight.write(pwmRight);
 
}

//ROS subscriver listening to the topic cmd_vel
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", servo_cb);

void setup()
{
  //Set pins 13 and 12 as servo outputs
  pinMode(13, OUTPUT);
  pinMode(12, OUTPUT);
  
  nh.initNode();
  
  nh.subscribe(sub);
  
  servoLeft.attach(12); //attach it to pin 13
  servoRight.attach(13);
  
 //servo.detach();
}

void loop()
{
  nh.spinOnce();
  delay(1); // pauses the code for 1 millisecond
}

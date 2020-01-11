#include <ros.h>
#include <ros/time.h> 
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <TimerOne.h>
#include "Variables.h"

// this function is called when the line nh.spinOnce() receives the message cmd_vel
void handle_cmd(const geometry_msgs::Twist& cmd_msg){
  linearSpeed = cmd_msg.linear.x;
  angularSpeed = cmd_msg.angular.z;
}

unsigned long lastTime = 0;

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &handle_cmd);      // see function handle_cmd

void PID();
void Position();
void Velocity();
void Motor();
void Publish();
void CMD_VEL();

void setup()
{    
    PCMSK1 = B00001111;   // enable PCINT8, PCINT9, PCINT10, PCINT11, interrupt for encoders and
    PCIFR = B00000000;    // clear all interrupt flags
    PCICR = B00000010;    // enable PCIE1 group
  
    pinMode(rightForward, OUTPUT);
    pinMode(rightBackward, OUTPUT);
    pinMode(leftForward, OUTPUT);
    pinMode(leftBackward, OUTPUT);
    pinMode(encoderPin1, INPUT_PULLUP); 
    pinMode(encoderPin2, INPUT_PULLUP);
    pinMode(encoderPin3, INPUT_PULLUP); 
    pinMode(encoderPin4, INPUT_PULLUP);
  
    digitalWrite(encoderPin1, HIGH); //turn pullup resistor on
    digitalWrite(encoderPin2, HIGH); //turn pullup resistor on
    digitalWrite(encoderPin3, HIGH); //turn pullup resistor on
    digitalWrite(encoderPin4, HIGH); //turn pullup resistor on
    
    nh.initNode();
    nh.getHardware()->setBaud(57600);
    nh.subscribe(sub);
    nh.advertise(pub_pos);
    nh.advertise(pub_vel);
    //nh.advertise(pub_cur_vel);
    //nh.advertise(pub_puls_L);
    //nh.advertise(pub_puls_R);
    int lastTime = millis();
}

void loop()
{   
  nh.spinOnce();                            // check for subscriber messages in ROS
  if(millis() - lastTime >= 25)             // make sure loop takes at least 50ms
  {
    lastTime = millis();                    
    if(nh.connected())                      // check for connection between the Arduino and ROS
    {
      CMD_VEL();
      Velocity();
      Position(); 
      PID();
      Motor();
      Publish();
    }
    else{                                   // if connection is lost, set the most important values to zero
      linearSpeed = 0;
      angularSpeed = 0;
      
      digitalWrite(rightForward, LOW);
      digitalWrite(rightBackward, LOW);
      digitalWrite(leftForward, LOW);
      digitalWrite(leftBackward, LOW);
    }
  }
}

// this function publishes every value generated in this Arduino file to ROS 
// this is done because the serial monitor of Arduino does not work properly when connected with ROS
void Publish()
{
    vel_msg.linear.x = vx;
    vel_msg.angular.z = vth;
    
    //cur_vel_msg.linear.x = vx * 40;
    //cur_vel_msg.angular.z = vth * 40;
    
    pos_msg.linear.x = x;
    pos_msg.linear.y = y;
    pos_msg.angular.z = th;
    
    //pulses_left.data = LeftWheel.pulsesPerSecond;
    //pulses_right.data = RightWheel.pulsesPerSecond;

    pub_pos.publish(&pos_msg);
    //pub_cur_vel.publish(&cur_vel_msg);
    pub_vel.publish(&vel_msg);
    //pub_puls_L.publish(&pulses_left);
    //pub_puls_R.publish(&pulses_right);
    
    nh.spinOnce();
}

// this function calculates the speed at which the left and right wheel have to rotate to accomadate the linearSpeed and angularSpeed we receive from ROS
void CMD_VEL()
{
    LeftWheel.radiantsPerSecond = ((2 * linearSpeed) - (angularSpeed * baseLine)) / (2 * wheelRadius);
    LeftWheel.speedCommand = (LeftWheel.radiantsPerSecond  * wheelRadius) / meterPerPuls;                 // this is the result we feed into the PID controller
    
    RightWheel.radiantsPerSecond  = ((2 * linearSpeed) + (angularSpeed * baseLine)) / (2 * wheelRadius);
    RightWheel.speedCommand = (RightWheel.radiantsPerSecond  * wheelRadius) / meterPerPuls;               // this is the result we feed into the PID controller
}

// this function calculates the current speed of the robot
void Velocity()
{
    // calculate pulses per second from the encoders
    LeftWheel.pulsesPerSecond = LeftWheel.encoderValue * 40;                // the sample time is 25ms so that's why we multiply with 40
    RightWheel.pulsesPerSecond = RightWheel.encoderValue * 40;
    
    // calculate current speed in meters per sample (25 ms)
    LeftWheel.linearVelocity = (LeftWheel.encoderValue * meterPerPuls);
    RightWheel.linearVelocity = (RightWheel.encoderValue * meterPerPuls);
    
    // reset encoder count
    LeftWheel.encoderValue = 0;
    RightWheel.encoderValue = 0;

    // calculate origin speed of the robot per sample (25 ms)
    vx = (RightWheel.linearVelocity + LeftWheel.linearVelocity) / 2;
    vy = 0.0;
    vth = ((RightWheel.linearVelocity - LeftWheel.linearVelocity) / (baseLine));
}

// this function calculates the position of the robot
// we need this for our odom frame
void Position()
{ 
    //  calculate distance travelled
    double delta_x = vx * cos(th);
    double delta_y = vx * sin(th);
    double delta_th = vth;
 
    // calculate current position
    x += delta_x;
    y += delta_y;
    th += delta_th;   
}

// this function controls the speed of the robot based on the current error
void PID()
{
    // calculate current error
    LeftWheel.error = LeftWheel.speedCommand - LeftWheel.pulsesPerSecond;
    RightWheel.error = RightWheel.speedCommand - RightWheel.pulsesPerSecond;
    
    // calculate integral error
    LeftWheel.somError = LeftWheel.somError + LeftWheel.error;
    RightWheel.somError = RightWheel.somError + RightWheel.error;

    // calculate differential error
    LeftWheel.deltaError = LeftWheel.error - LeftWheel.previousError;
    RightWheel.deltaError = RightWheel.error - RightWheel.previousError;

    // set previous error to current error
    LeftWheel.previousError = LeftWheel.error;
    RightWheel.previousError = RightWheel.error;

    // calculate output needed to reach the speed command
    LeftWheel.totalOutput = (Kp * LeftWheel.error) + (Ki * LeftWheel.somError) + (Kd * LeftWheel.deltaError);
    RightWheel.totalOutput = (Kp * RightWheel.error) + (Ki * RightWheel.somError) + (Kd * RightWheel.deltaError);

    if(LeftWheel.totalOutput > 26500)   // limiter set, otherwise the output will get to large and the robot will display undefined behavior
    {
      LeftWheel.totalOutput = 26500;    // max speed of the robot
    }

    if(RightWheel.totalOutput > 26500)
    {
      RightWheel.totalOutput = 26500;
    }
}

// this function uses the calculated totalOutput in PID to drive the wheels
void Motor()
{
    if(LeftWheel.totalOutput < 0)                                                   // for driving backward
    {
        LeftWheel.totalOutput = LeftWheel.totalOutput * -1;
        digitalWrite(leftForward, LOW);                                             // set forward output LOW otherwise, forward and backward will both be active and the robot will not move
        analogWrite(leftBackward, (int) (LeftWheel.totalOutput / (27.04 * 4)));     // the totalOutput is divided by (27.04 * 4) because the robot can only accept values from 0 to 255
    }else                                                                           // for driving forward
    {
        digitalWrite(leftBackward, LOW);
        analogWrite(leftForward, (int) (LeftWheel.totalOutput / (27.04 * 4)));
    }
    
    if(RightWheel.totalOutput < 0)                                                  // for driving backward
    {
        RightWheel.totalOutput = RightWheel.totalOutput * -1;
        digitalWrite(rightForward, LOW);
        analogWrite(rightBackward, (int) (RightWheel.totalOutput / (27.04 * 4)));
    }else                                                                           // for driving forward
    {
        digitalWrite(rightBackward, LOW);
        analogWrite(rightForward, (int) (RightWheel.totalOutput / (27.04 * 4)));
    }
}

// Interrupt for the encoder on the left and right wheel
ISR(PCINT1_vect)
{
    int MSB_L = digitalRead(encoderPin3);
    int LSB_L = digitalRead(encoderPin4);
    int MSB_R = digitalRead(encoderPin1);
    int LSB_R = digitalRead(encoderPin2);

    int encoded_L = (MSB_L << 1) |LSB_L;
    int sum_L = (lastEncodedB << 2) | encoded_L;
    
    int encoded_R = (MSB_R << 1) |LSB_R;
    int sum_R = (lastEncodedA << 2) | encoded_R;

    if(sum_L == 0b1101 || sum_L == 0b0100 || sum_L == 0b0010 || sum_L == 0b1011) LeftWheel.encoderValue ++;
    if(sum_L == 0b1110 || sum_L == 0b0111 || sum_L == 0b0001 || sum_L == 0b1000) LeftWheel.encoderValue --;
    
    if(sum_R == 0b1101 || sum_R == 0b0100 || sum_R == 0b0010 || sum_R == 0b1011) RightWheel.encoderValue ++;
    if(sum_R == 0b1110 || sum_R == 0b0111 || sum_R == 0b0001 || sum_R == 0b1000) RightWheel.encoderValue --;

    lastEncodedB = encoded_L;
    lastEncodedA = encoded_R;
}


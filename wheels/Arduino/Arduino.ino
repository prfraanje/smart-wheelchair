#include <ros.h>
#include <ros/time.h> 
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
//#include <TimerOne.h>
#include "Variables.h"

void handle_cmd(const geometry_msgs::Twist& cmd_msg){
  linearSpeed = cmd_msg.linear.x;
  angularSpeed = cmd_msg.angular.z;
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &handle_cmd);

void Odom();
void PID();
void Velocity();
void Motor();
void Publish();
void testSoft();
void updateEncoder();

void setup()
{    
    PCMSK1 = B00000011; //enable PCINT8, PCINT9
    PCIFR = B00000000; // clear all interrupt flags
    PCICR = B00000010; // enable PCIE1 group
  
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
  
    attachInterrupt(0, updateEncoder, CHANGE); 
    attachInterrupt(1, updateEncoder, CHANGE);
    
    //Timer1.initialize(10000);
    //Timer1.attachInterrupt(testSoft); 
    nh.initNode();
    nh.getHardware()->setBaud(57600);
    nh.subscribe(sub);
    nh.advertise(pub_odom);
    nh.advertise(pub_vel);

}

void loop()
{
    nh.spinOnce();
    Velocity(); 
    PID();
    Motor();
    Odom();
    Publish();
    delay(10);
}

// void testSoft()
// {
//     Velocity();
//     PID();
//     Motor();
// }

void Odom()
{
   //compute odometry in a typical way given the velocities of the robot
   double delta_x = (vx * cos(th) - vy * sin(th)) * 0.01;
   double delta_y = (vx * sin(th) + vy * cos(th)) * 0.01;
   double delta_th = vth * 0.01;

   x += delta_x;
   y += delta_y;
   th += delta_th; 
   
   
}

void Publish()
{
    odom_msg.header.stamp = nh.now();
    odom_msg.vector.x = x;
    odom_msg.vector.y = y;
    odom_msg.vector.z = th;
    
    vel_msg.vector.x = vx;
    vel_msg.vector.z = vth;

    pub_odom.publish(&odom_msg);
    pub_vel.publish(&vel_msg);
}

void Velocity()
{
    LeftWheel.pulsesPerSecond = LeftWheel.encoderValue * 100;
    RightWheel.pulsesPerSecond = RightWheel.encoderValue * 100;

    LeftWheel.encoderValue = 0;
    RightWheel.encoderValue = 0;

    LeftWheel.linearVelocity = LeftWheel.pulsesPerSecond * meterPerPuls;
    RightWheel.linearVelocity = RightWheel.pulsesPerSecond * meterPerPuls;

    vx = (RightWheel.linearVelocity + LeftWheel.linearVelocity) / 2;
    vy = 0.0;
    vth = (RightWheel.linearVelocity - LeftWheel.linearVelocity) / baseLine;
    
    Serial.print("VX : ");
    Serial.println(vx);
    
    Serial.print("VTH : ");
    Serial.println(vth);
}

void PID()
{
    LeftWheel.error = LeftWheel.speedCommand - LeftWheel.pulsesPerSecond;
    RightWheel.error = RightWheel.speedCommand - RightWheel.pulsesPerSecond;

    LeftWheel.somError = LeftWheel.somError + LeftWheel.error;
    RightWheel.somError = RightWheel.somError + RightWheel.error;

    LeftWheel.deltaError = LeftWheel.error - LeftWheel.previousError;
    RightWheel.deltaError = RightWheel.error - RightWheel.previousError;

    LeftWheel.previousError = LeftWheel.error;
    RightWheel.previousError = RightWheel.error;

    LeftWheel.totalOutput = (Kp * LeftWheel.error) + (Ki * LeftWheel.somError) + (Kd * LeftWheel.deltaError);
    RightWheel.totalOutput = (Kp * RightWheel.error) + (Ki * RightWheel.somError) + (Kd * RightWheel.deltaError);

    if(LeftWheel.totalOutput > 26500)
    {
      LeftWheel.totalOutput = 26500;
    }

    if(RightWheel.totalOutput > 26500)
    {
      RightWheel.totalOutput = 26500;
    }
}

void Motor()
{
    LeftWheel.speedCommand = ((2 * linearSpeed) + (angularSpeed * baseLine)) / (2 * wheelRadius);
    LeftWheel.speedCommand = pulsesPerRotation * (LeftWheel.speedCommand / (2 * M_PI));

    RightWheel.speedCommand = ((2 * linearSpeed) - (angularSpeed * baseLine)) / (2 * wheelRadius);
    RightWheel.speedCommand = pulsesPerRotation * (RightWheel.speedCommand / (2 * M_PI));


    if(LeftWheel.totalOutput < 0)
    {
        LeftWheel.totalOutput = LeftWheel.totalOutput * -1;
        digitalWrite(leftForward, LOW);
        analogWrite(leftBackward, LeftWheel.totalOutput / (27.04 * 4));
    }else
    {
        digitalWrite(leftBackward, LOW);
        analogWrite(leftForward, LeftWheel.totalOutput / (27.04 * 4));
    }
    
    if(RightWheel.totalOutput < 0)
    {
        RightWheel.totalOutput = RightWheel.totalOutput * -1;
        digitalWrite(rightForward, LOW);
        analogWrite(rightBackward, RightWheel.totalOutput / (27.04 * 4));
    }else
    {
        digitalWrite(rightBackward, LOW);
        analogWrite(rightForward, RightWheel.totalOutput / (27.04 * 4));
    }
}

// Interrupt for the encoder on the right wheel
void updateEncoder()
{
    int MSB = digitalRead(encoderPin1);
    int LSB = digitalRead(encoderPin2);

    int encoded = (MSB << 1) |LSB;
    int sum = (lastEncodedA << 2) | encoded;

    if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) RightWheel.encoderValue ++;
    if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) RightWheel.encoderValue --;

    lastEncodedA = encoded;
}

// Interrupt for the encoder on the left wheel
ISR(PCINT1_vect)
{
    int MSB = digitalRead(encoderPin3);
    int LSB = digitalRead(encoderPin4);

    int encoded = (MSB << 1) |LSB;
    int sum = (lastEncodedB << 2) | encoded;

    if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) LeftWheel.encoderValue ++;
    if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) LeftWheel.encoderValue --;

    lastEncodedB = encoded;
}


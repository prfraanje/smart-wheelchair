#include <ros.h>
#include <ros/time.h> 
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
//#include <TimerOne.h>
#include "Variables.h"

void handle_cmd(const geometry_msgs::Twist& cmd_msg){
  linearSpeed = cmd_msg.linear.x;
  angularSpeed = cmd_msg.angular.z;
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &handle_cmd);

void PID();
void Odom();
void Velocity();
void Motor();
void Publish();
void testSoft();
void updateEncoder();

void setup()
{    
    PCMSK1 = B00001111; //enable PCINT8, PCINT9, PCINT10, PCINT11
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
  
    //attachInterrupt(0, updateEncoder, CHANGE); 
    //attachInterrupt(1, updateEncoder, CHANGE);
    
    //Timer1.initialize(10000);
    //Timer1.attachInterrupt(testSoft); 
    nh.initNode();
    nh.getHardware()->setBaud(57600);
    nh.subscribe(sub);
    nh.advertise(pub_pos);
    nh.advertise(pub_vel);
    nh.advertise(pub_cur_vel);
    nh.advertise(pub_puls_L);
    nh.advertise(pub_puls_R);
}

void loop()
{
    //int time_t1 = millis();
    nh.spinOnce();
    Velocity();
    Odom(); 
    PID();
    Motor();
    Publish();
    //int time_t2 = millis() - time_t1;
    //Serial.println(time_t2);
    delay(10);
}

// void testSoft()
// {
      //nh.spinOnce();
      //Velocity();
      //Odom(); 
      //PID();
      //Motor();
// }

void Publish()
{
    vel_msg.linear.x = vx;
    vel_msg.angular.z = vth;
    
    cur_vel_msg.linear.x = vx * 100;
    cur_vel_msg.angular.z = vth * 100;
    
    pos_msg.linear.x = x;
    pos_msg.linear.y = 0.0;
    pos_msg.angular.z = th;
    
    pulses_left.data = LeftWheel.pulsesPerSecond;
    pulses_right.data = RightWheel.pulsesPerSecond;

    pub_pos.publish(&pos_msg);
    pub_cur_vel.publish(&cur_vel_msg);
    pub_vel.publish(&vel_msg);
    pub_puls_L.publish(&pulses_left);
    pub_puls_R.publish(&pulses_right);
    
}

void Velocity()
{
    LeftWheel.pulsesPerSecond = LeftWheel.encoderValue * 100;
    RightWheel.pulsesPerSecond = RightWheel.encoderValue * 100;

    LeftWheel.linearVelocity = (LeftWheel.encoderValue * meterPerPuls) / wheelRadius;
    RightWheel.linearVelocity = (RightWheel.encoderValue * meterPerPuls) / wheelRadius;
    
    LeftWheel.encoderValue = 0;
    RightWheel.encoderValue = 0;

    vx = (RightWheel.linearVelocity + LeftWheel.linearVelocity) / 2;
    vy = 0.0;
    vth = (RightWheel.linearVelocity - LeftWheel.linearVelocity) * (wheelRadius / baseLine);
}

void Odom()
{
    double delta_x = (vx * cos(th) - vy * sin(th));
    //double delta_y = (vx * sin(th) + vy * cos(th));
    double delta_th = vth;
 
    x += delta_x;
    //y += delta_y;
    th += delta_th;   
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
    LeftWheel.speedCommand = ((2 * (linearSpeed * wheelRadius)) - (angularSpeed * baseLine)) / (2 * wheelRadius);
    LeftWheel.speedCommand = (LeftWheel.speedCommand * wheelRadius) / meterPerPuls;
    

    RightWheel.speedCommand = ((2 * (linearSpeed * wheelRadius)) + (angularSpeed * baseLine)) / (2 * wheelRadius);
    RightWheel.speedCommand = (RightWheel.speedCommand * wheelRadius) / meterPerPuls;

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
//void updateEncoder()
//{
//    int MSB = digitalRead(encoderPin1);
//    int LSB = digitalRead(encoderPin2);
//
//    int encoded = (MSB << 1) |LSB;
//    int sum = (lastEncodedA << 2) | encoded;
//
//    if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) RightWheel.encoderValue ++;
//    if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) RightWheel.encoderValue --;
//
//    lastEncodedA = encoded;
//}

// Interrupt for the encoder on the left wheel
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


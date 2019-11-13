#ifndef VARIABLES_H
#define VARIABLES_H

#define leftBackward  6
#define leftForward 5

#define rightForward   11
#define rightBackward  10

#define M_PI  3.14159265358979323846

int encoderPin1 = 2;
int encoderPin2 = 3;
int encoderPin3 = A0;
int encoderPin4 = A1;

struct Wheel {
    double speedCommand = 0;
    double totalOutput = 0;
    double error = 0;
    double somError = 0;
    double deltaError = 0;
    double previousError = 0;

    double encoderValue = 0;
    double pulsesPerSecond = 0;
    double linearVelocity = 0;
} LeftWheel, RightWheel;

double Kp = 0.15;
double Ki = 0.1;
double Kd = 0.1;

volatile long lastEncodedA = 0;
volatile long lastEncodedB = 0;

double pulsesPerRotation = 27466;
double meterPerPuls = 0.0000343143;
double wheelRadius = 0.15;            // meters
double baseLine = 0.56;               // meters

double linearSpeed = 0;
double angularSpeed = 0;

ros::NodeHandle nh;

geometry_msgs::Vector3Stamped odom_msg;
geometry_msgs::Vector3Stamped vel_msg;

ros::Publisher pub_odom("odom_xy", &odom_msg);
ros::Publisher pub_vel("current_vel_xyz", &vel_msg);

double x = 0.0;
double y = 0.0;
double th = 0.0;

double vx = 0;
double vy = 0;
double vth = 0;

#endif /* VARIABLES_H */

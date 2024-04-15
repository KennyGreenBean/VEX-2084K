#pragma once

#include "Calculations/PID.hpp"

class Chassis{
  public:
    Chassis(pros::Motor FrontLeft, pros::Motor FrontRight, pros::Motor MiddleLeft, pros::Motor MiddleRight, pros::Motor BackLeft, pros::Motor BackRight, pros::Imu inertial);

    void ptoSwitch(bool currentToggle);
    void brake();

    void inlineTelOp(int leftY, int rightX);

    void move(int leftSpeed, int rightSpeed);
    void moveDistance(int leftSpeed, int rightSpeed, int distance);
    void stop();
    int getVelocity();
    void resetMotorEncoders();
    double getDriveEncoder();
    void moveSteering(float steering, int speed);
    void moveFowardMotor(double target, int speed, int decelZone);
    void moveForward(double target, int speed);
    void turnInertial(double target);
    void turnInertialAbsolute(double target);
    void turnInertialAbsoluteFast(double target);
    void turnInertialFast(double target);
    void reset();


    // PID Objects
    PID keepStraight = PID(0.4, 0.01, 0.2, 0);
    PID forwardController = PID(0.3, 0.01, 0, 30);
    PID turnController = PID(0.7, 0.001, 5, 30);
    // 0.4, 0.03, 3, 15
    //1.2, 0, 0.1, 30

    

  protected:
    // false = 6m drive, true = 4m drive
    bool pto = false;

    pros::Motor frontLeft;
    pros::Motor frontRight;
    pros::Motor middleLeft;
    pros::Motor middleRight;
    pros::Motor backLeft;
    pros::Motor backRight;

    int frontleftport = 12;
    int frontrightport = 19;
    int middleleftport = 13;
    int middlerightport = 18;

    pros::Imu inertial;
};
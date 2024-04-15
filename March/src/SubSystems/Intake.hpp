#pragma once

#include "api.h"

class Intake{
  public:
    Intake(pros::Motor intake);

    void brake();
    void telOP(bool intake, bool outtake);
    void spinIntake(bool direction);
    void spinIntakeTime(int speed, float time);
    void stop();

  private:
    pros::Motor intake;
};
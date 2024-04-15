#include "Catapult.hpp"
#include "pros/adi.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/optical.h"
#include "pros/rtos.hpp"

Catapult::Catapult(pros::Motor left, pros::Motor right) : left(left), right(right){}

// controls shooting and reloading of catapult

void Catapult::ptoSwitch(bool currentToggle){
	pto = currentToggle;
}

void Catapult::brake(){
	left.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	right.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
}

void Catapult::telOP(bool pressed){
	if(pressed){
		left.move_velocity(200);
		right.move_velocity(200);
	}
	else{
		left.move_velocity(0);
		right.move_velocity(0);
	}
}

void Catapult::spin(){
	left.move_velocity(200);
	right.move_velocity(200);
}

void Catapult::stop(){
	left.move_velocity(0);
	right.move_velocity(0);
}
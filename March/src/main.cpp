#include "main.h"
#include "SubSystems/Chassis.hpp"
#include "SubSystems/Intake.hpp"
#include "SubSystems/Catapult.hpp"

pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::Motor LEFT_MIDDLE(-13, pros::v5::MotorGears::blue);//
pros::Motor LEFT_FRONT(-12, pros::v5::MotorGears::blue);//
pros::Motor LEFT_BACK(-11, pros::v5::MotorGears::blue);//
pros::Motor RIGHT_MIDDLE(18, pros::v5::MotorGears::blue);//
pros::Motor RIGHT_FRONT(19, pros::v5::MotorGears::blue);//
pros::Motor RIGHT_BACK(20, pros::v5::MotorGears::blue);//

pros::Motor catapultRight(-15, pros::v5::MotorGears::green);
pros::Motor catapultLeft(17, pros::v5::MotorGears::green);
pros::Motor intakeMotor(-14, pros::v5::MotorGears::blue);

pros::Imu inertial(1);

pros::adi::DigitalOut hang('A');
pros::adi::DigitalOut frontwings('B');
pros::adi::DigitalOut rightWing('C');
pros::adi::DigitalOut leftWing('D');

Chassis chassis(LEFT_FRONT, RIGHT_FRONT, LEFT_MIDDLE, RIGHT_MIDDLE, LEFT_BACK, RIGHT_BACK, inertial);
Catapult catapult(catapultLeft, catapultRight);
Intake intake(intakeMotor);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

void far6(){
	chassis.reset();

	intake.spinIntake(true);
	pros::delay(500);
	chassis.moveDistance(-300, -300, 700);
	intake.stop();
	chassis.turnInertialAbsolute(150);
	chassis.moveDistance(300, 300, 500);
	rightWing.set_value(true);
	chassis.moveDistance(100, 500,550);
	rightWing.set_value(false);
	chassis.moveDistance(-100, -500,550);
	chassis.turnInertialAbsoluteFast(160);
	frontwings.set_value(true);
	chassis.move(300, 600);
	intake.spinIntake(false);
	pros::delay(1000);
	chassis.move(600, 600);
	pros::delay(500);
	chassis.stop();
	intake.stop();
	frontwings.set_value(false);
	chassis.moveDistance(-200, -200, 300);
	pros::delay(200);
	chassis.turnInertialAbsolute(-0);
	chassis.move(-200, -200);
	pros::delay(300);
	chassis.reset();
	chassis.moveForward(700, 300);
	chassis.turnInertialAbsoluteFast(20);
	intake.spinIntake(true);
	chassis.moveDistance(400, 400, 1300);
	pros::delay(300);
	intake.stop();
	chassis.turnInertialAbsoluteFast(170);
	intake.spinIntake(false);
	chassis.moveDistance(300, 300, 150);
	chassis.turnInertialAbsoluteFast(60);
	intake.spinIntake(true);
	chassis.moveDistance(300, 300, 1000);
	chassis.turnInertialAbsoluteFast(0);
	leftWing.set_value(true);
	rightWing.set_value(true);
	chassis.move(-500, -500);
	pros::delay(1000);
	chassis.stop();
	leftWing.set_value(false);
	rightWing.set_value(false);
	chassis.moveDistance(300, 300, 300);
	intake.stop();
	chassis.turnInertialFast(180);
	intake.spinIntake(false);
	frontwings.set_value(true);
	chassis.move(600, 600);
	pros::delay(800);
	chassis.stop();

}

void farSafe(){
	intake.spinIntake(true);
	chassis.moveDistance(300, 300, 100);
	rightWing.set_value(true);
	chassis.moveDistance(100, 500,500);
	rightWing.set_value(false);
	chassis.moveDistance(-100, -500,500);
	chassis.turnInertialAbsoluteFast(10);
	frontwings.set_value(true);
	chassis.move(400, 600);
	intake.spinIntake(false);
	pros::delay(1000);
	chassis.move(600, 600);
	pros::delay(500);
	chassis.stop();
	intake.stop();
	frontwings.set_value(false);
	chassis.moveDistance(-200, -200, 200);
	pros::delay(200);
	chassis.turnInertialAbsolute(-120);
	chassis.move(-200, -200);
	pros::delay(300);
	chassis.reset();
	chassis.moveForward(700, 300);
	chassis.turnInertialAbsoluteFast(15);
	intake.spinIntake(true);
	chassis.moveDistance(400, 400, 1200);
	pros::delay(300);
	intake.stop();
	chassis.turnInertialAbsoluteFast(170);
	intake.spinIntake(false);
	chassis.moveDistance(300, 300, 150);
	chassis.turnInertialAbsoluteFast(60);
	intake.spinIntake(true);
	chassis.moveDistance(300, 300, 900);
	chassis.turnInertialAbsoluteFast(0);
	leftWing.set_value(true);
	rightWing.set_value(true);
	chassis.move(-500, -500);
	pros::delay(1000);
	chassis.stop();
	leftWing.set_value(false);
	rightWing.set_value(false);
	chassis.moveDistance(300, 300, 300);
	chassis.turnInertialFast(180);
	intake.spinIntake(false);
	chassis.move(600, 600);
	pros::delay(800);
	chassis.stop();
}

void close(){
	chassis.reset();
	frontwings.set_value(true);
	pros::delay(200);
	frontwings.set_value(false);
	intake.spinIntake(true);
	chassis.moveForward(1700, 600);
	chassis.moveForward(1850, -400);
	intake.stop();
	chassis.turnInertialAbsolute(110);
	intake.spinIntake(false);

	chassis.moveDistance(-300, -300, 500);
	chassis.move(-300, -600);
	pros::delay(500);
	chassis.move(-600, -600);
	pros::delay(400);
	chassis.stop();

	chassis.moveDistance(200, 200, 200); 
	chassis.turnInertial(-40);
	chassis.moveDistance(200, 200, 700);
	rightWing.set_value(true);
	chassis.moveDistance(100, 500,500);
	rightWing.set_value(false);
	chassis.moveDistance(-100, -500,500);
	chassis.turnInertialFast(10);
	chassis.moveDistance(220, 400, 800);
	chassis.moveDistance(300, 300, 1500);
	chassis.turnInertialFast(10);
	chassis.moveDistance(-300, -300, 900);
	chassis.moveDistance(-300, -400, 400);
	rightWing.set_value(true);
	chassis.moveDistance(-300, -300, 600);
}

void autonomous() {
	// farSafe();
	far6();
	// close();
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	intake.brake();
	chassis.brake();
	inertial.reset();

	bool hangToggle = false;
	bool frontwingsToggle = false;
	bool rightWingToggle = false;
	bool leftWingToggle = false;

	while(true){

		//Function in chassis class to move according to joystick inputs
		chassis.inlineTelOp(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), 
		master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));

		// function in intake class to spin intake based on controller button inputs
		intake.telOP(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1), 
		master.get_digital(pros::E_CONTROLLER_DIGITAL_R2));

		// function in catapult class to spin catapult motors based on controller inputs
		catapult.telOP(master.get_digital(pros::E_CONTROLLER_DIGITAL_Y));

		// if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)){
		// 	rightWingToggle = !rightWingToggle;
		// 	rightWing.set_value(rightWingToggle);
		// }
		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)){
			frontwingsToggle = !frontwingsToggle;
			frontwings.set_value(frontwingsToggle);
		}
		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)){
			hangToggle = !hangToggle;
			hang.set_value(hangToggle);
		}
		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)){
			leftWingToggle = !leftWingToggle;
			leftWing.set_value(leftWingToggle);
		}
		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)){
			rightWingToggle = !rightWingToggle;
			rightWing.set_value(rightWingToggle);
		}
		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)){
			leftWingToggle = !leftWingToggle;
			leftWing.set_value(leftWingToggle);
			rightWingToggle = !rightWingToggle;
			rightWing.set_value(rightWingToggle);
		}
	}
}
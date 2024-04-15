#include "main.h"
#include "display/lv_core/lv_obj.h"
#include "display/lv_draw/lv_draw.h"
#include "display/lv_misc/lv_color.h"
#include "display/lv_objx/lv_btn.h"
#include "display/lv_objx/lv_btnm.h"
#include "display/lv_objx/lv_img.h"
#include "display/lv_objx/lv_label.h"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include "api.h"
#include "okapi/api.hpp"
 
#include "SubSystems/Chassis.hpp"
#include "SubSystems/Intake.hpp"
#include "SubSystems/Flywheel.hpp"
#include "SubSystems/Catapult.hpp"
#include <cmath>
#include <functional>

#include "pros/apix.h"

using namespace okapi;

pros::Controller master(pros::E_CONTROLLER_MASTER); // Sets controller name to master

// Tells what ports each motor is on, what motor cartridge they have, and whether or not they are reversed
pros::Motor LEFT_FRONT(19, pros::E_MOTOR_GEARSET_06, true);//
pros::Motor LEFT_MIDDLE(9, pros::E_MOTOR_GEARSET_06, true);
pros::Motor LEFT_BACK(8, pros::E_MOTOR_GEARSET_06, true);

pros::Motor RIGHT_FRONT(13, pros::E_MOTOR_GEARSET_06, false);//
pros::Motor RIGHT_MIDDLE(3, pros::E_MOTOR_GEARSET_06, false);//
pros::Motor RIGHT_BACK(1, pros::E_MOTOR_GEARSET_06, false);

pros::Motor shooter(10, pros::E_MOTOR_GEARSET_36, false);//

pros::Motor intakeMotor(15, pros::E_MOTOR_GEARSET_06, true);

pros::IMU inertial(2);

pros::ADIDigitalOut dropdownRight('H');
pros::ADIDigitalOut elevate ('D');
pros::ADIDigitalOut Flaps('A');
pros::ADIDigitalOut balance('B');
pros::ADIDigitalOut dropdownLeft('F');

// class initialization
Chassis chassis = Chassis(LEFT_FRONT, RIGHT_FRONT, LEFT_MIDDLE, RIGHT_MIDDLE, LEFT_BACK, RIGHT_BACK, inertial);
Intake intake = Intake(intakeMotor);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

void initialize() {
}


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

void competition_initialize() {
}


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

void far(){
	intake.brake();
	chassis.brake();
	chassis.reset();

	dropdownRight.set_value(true);
	intake.spinIntake(true);
	pros::delay(500);
	intake.spinIntake(false);
	chassis.moveDistance(250, 600, 500);
	dropdownRight.set_value(false);
	intake.spinIntake(false);
	chassis.move(500, 500);
	pros::delay(800);
	chassis.stop();

	chassis.moveDistance(-200, -200, 500);
	chassis.turnInertialAbsolute(-115);
	intake.spinIntake(true);
	chassis.moveForward(2200, 400);
	pros::delay(500);
	intake.stop();
	chassis.turnInertialAbsolute(130);
	intake.spinIntake(false);
	chassis.moveDistance(200, 200, 300);
	chassis.turnInertialAbsolute(40);
	intake.spinIntake(true);
	chassis.moveDistance(200, 200, 1000);
	pros::delay(300);
	intake.stop();
	chassis.turnInertialAbsoluteFast(160);
	Flaps.set_value(true);
	intake.spinIntake(false);
	chassis.move(400, 400);
	pros::delay(300);
	intake.stop();
}

void far5(){
	intake.brake();
	chassis.brake();
	chassis.reset();

	intake.spinIntake(true);
	pros::delay(500);
	chassis.moveDistance(200, 200, 200);
	chassis.moveDistance(-300, -300, 900);
	chassis.turnInertialAbsolute(140);

	chassis.moveDistance(300, 300, 500);
	dropdownRight.set_value(true);
	intake.spinIntake(false);
	chassis.moveDistance(300, 600, 600);
	dropdownRight.set_value(false);
	chassis.move(500, 500);
	pros::delay(800);
	chassis.stop();

	chassis.moveDistance(-200, -200, 500);
	chassis.turnInertialAbsolute(15);
	intake.spinIntake(true);
	chassis.moveForward(2200, 400);
	pros::delay(500);
	intake.stop();
	chassis.turnInertialAbsoluteFast(150);
	intake.spinIntake(false);
	chassis.moveDistance(200, 200, 300);
	chassis.turnInertialAbsolute(35);
	intake.spinIntake(true);
	chassis.moveDistance(200, 200, 1000);
	pros::delay(300);
	intake.stop();
	chassis.turnInertialAbsolute(165);
	Flaps.set_value(true);
	intake.spinIntake(false);
	chassis.move(400, 400);
}

void close(){
	intake.brake();
	chassis.brake();
	chassis.reset();

	chassis.moveDistance(-200, -200, 360);
	dropdownRight.set_value(true);
	intake.spinIntake(true);
	chassis.moveDistance(100, 600, 500);
	dropdownRight.set_value(false);
	chassis.moveDistance(200, 200, 300);
	chassis.turnInertialAbsolute(-40);
	intake.spinIntake(false);
	chassis.moveDistance(300, 300, 1250);
	// Flaps.set_value(true);
}

void closeElim(){
	intake.brake();
	chassis.brake();
	chassis.reset();
	
	intake.spinIntake(true);
	Flaps.set_value(true);
	pros::delay(50);
	Flaps.set_value(false);
	chassis.moveForward(1800, 600);
	pros::delay(300);
	chassis.turnInertialAbsolute(5);
	chassis.moveForward(1700, -500);
	chassis.turnInertialAbsolute(100);
	intake.spinIntake(false);
	chassis.moveDistance(-300, -300, 800);
	chassis.turnInertial(40);
	chassis.move(-200, -600);
	pros::delay(700);
	chassis.move(-600, -600);
	pros::delay(500);
	chassis.stop();

	chassis.moveDistance(200, 500, 400); 
	chassis.moveDistance(200, 200, 700);
	dropdownRight.set_value(true);
	chassis.moveDistance(100, 500,500);
	dropdownRight.set_value(false);
	chassis.moveDistance(-100, -500,500);
	chassis.turnInertialFast(10);
	chassis.moveDistance(220, 400, 800);
	chassis.moveDistance(300, 300, 1500);
	chassis.moveDistance(-300, -300, 1300);
	chassis.moveDistance(-200, -400, 400);
	dropdownRight.set_value(true);
	chassis.moveDistance(-300, -300, 300);
}

void closeElimExperiment(){
	intake.brake();
	chassis.brake();
	chassis.reset();
	shooter.move_velocity(100);
	pros::delay(700);
	shooter.move_velocity(0);
	chassis.turnInertial(-5);
	intake.spinIntake(true);
	chassis.moveForward(1900, 600);
	pros::delay(300);
	chassis.turnInertialAbsolute(0);
	chassis.moveForward(1800, -500);
	chassis.turnInertialAbsolute(100);
	intake.spinIntake(false);

	chassis.moveDistance(-300, -300, 800);
	chassis.turnInertial(30);
	chassis.move(-200, -600);
	pros::delay(700);
	chassis.move(-600, -600);
	pros::delay(500);
	chassis.stop();

	chassis.moveDistance(200, 500, 400); 
	chassis.moveDistance(200, 200, 700);
	dropdownRight.set_value(true);
	chassis.moveDistance(100, 300,400);
	dropdownRight.set_value(false);
	chassis.moveDistance(-100, -300,400);
	chassis.turnInertialFast(10);
	chassis.moveDistance(220, 400, 800);
	chassis.moveDistance(300, 300, 1700);
	chassis.moveDistance(-300, -300, 2000);
}

void skills(){
	intake.brake();
	chassis.brake();
	chassis.reset();
	intake.spinIntake(true);

	chassis.moveDistance(-200, -200, 500);
	intake.stop();
	chassis.turnInertialAbsolute(-64);
	dropdownRight.set_value(true);
	shooter.move_velocity(200);
	pros::delay(31000);
	dropdownRight.set_value(false);
	shooter.move_velocity(0);
	chassis.turnInertialAbsolute(0);
	chassis.moveDistance(200, 200, 600);
	chassis.turnInertialAbsolute(-38);
	intake.spinIntake(false);
	chassis.moveForward(3150, 500);
	chassis.move(350, 600);
	pros::delay(1000);
	chassis.move(600, 600);
	pros::delay(500);
	chassis.stop();
	chassis.move(-300, -300);
	pros::delay(500);
	chassis.stop();
	chassis.move(600, 350);
	pros::delay(1000);
	chassis.stop();
	chassis.moveDistance(-200, -200, 400);
	chassis.turnInertialAbsolute(-190);
	chassis.move(-200, -200);
	pros::delay(500);
	chassis.stop();
	chassis.reset();
	chassis.moveDistance(200, 250, 300);
	chassis.move(400, 224);
	pros::delay(400);
	Flaps.set_value(true);
	chassis.moveDistance(400, 400, 300);
	chassis.move(400, 200);
	pros::delay(1900);
	chassis.move(600, 600);
	pros::delay(600);
	chassis.stop();
	Flaps.set_value(false);

	chassis.move(-500, -500);
	pros::delay(500);
	chassis.turnInertialAbsolute(180);
	chassis.move(-200, -200);
	pros::delay(2000);
	chassis.moveDistance(200, 200, 100);
	chassis.turnInertialFast(-120);
	intake.spinIntake(true);
	chassis.moveDistance(300, 300, 1300);
	intake.spinIntake(false);
	chassis.turnInertialAbsolute(60);
	Flaps.set_value(true);
	chassis.move(370, 600);
	pros::delay(2000);
	chassis.stop();
	Flaps.set_value(false);
	chassis.moveDistance(-200, -200, 300);
	chassis.turnInertialAbsoluteFast(0);


	chassis.move(-300, -300);
	pros::delay(1000);
	chassis.stop();
	chassis.moveDistance(200, 200, 100);
	chassis.turnInertial(-60);
	Flaps.set_value(true);
	chassis.moveDistance(300, 300, 1300);
	Flaps.set_value(false);
	chassis.moveDistance(200, 400, 300);
	chassis.move(600, 100);
	pros::delay(500);
	chassis.move(600, 200);
	pros::delay(800);
	chassis.move(600, 600);
	pros::delay(1000);
	chassis.stop();
	chassis.move(-300, -300);
	pros::delay(500);
	chassis.stop();
	chassis.move(600, 500);
	pros::delay(1000);
	chassis.stop();
	chassis.move(-300, -300);
	pros::delay(500);
	chassis.stop();
	chassis.move(600, 400);
	pros::delay(1000);
	chassis.stop();
	chassis.move(-600, -600);
}

void skills2(){
	intake.brake();
	chassis.brake();
	chassis.reset();
	intake.spinIntake(true);

	chassis.moveDistance(-200, -200, 500);
	intake.stop();
	chassis.turnInertialAbsolute(-64);
	dropdownRight.set_value(true);
	shooter.move_velocity(200);
	pros::delay(31000);
	dropdownRight.set_value(false);
	shooter.move_velocity(0);
	chassis.turnInertialAbsolute(0);
	chassis.moveDistance(200, 200, 600);
	chassis.turnInertialAbsolute(-38);
	intake.spinIntake(false);
	chassis.moveForward(3150, 500);
	chassis.move(350, 600);
	pros::delay(1000);
	chassis.move(600, 600);
	pros::delay(500);
	chassis.stop();
	chassis.move(-300, -300);
	pros::delay(500);
	chassis.stop();
	chassis.move(600, 350);
	pros::delay(1000);
	chassis.stop();
	chassis.moveDistance(-200, -200, 400);
	chassis.turnInertialAbsolute(-190);
	chassis.move(-200, -200);
	pros::delay(500);
	chassis.stop();
	chassis.reset();
	chassis.moveDistance(250, 250, 300);
	chassis.turnInertialAbsolute(5);
	chassis.moveDistance(300, 300, 1500);
	// chassis.move(400, 224);
	// pros::delay(400);
	chassis.turnInertialAbsolute(120);
	Flaps.set_value(true);
	chassis.move(500, 400);
	pros::delay(1500);
	chassis.move(600, 600);
	pros::delay(600);
	chassis.stop();
	Flaps.set_value(false);

	chassis.move(-500, -500);
	pros::delay(300);
	chassis.turnInertialAbsolute(180);
	chassis.move(-200, -200);
	pros::delay(1500);
	chassis.moveDistance(200, 200, 100);
	chassis.turnInertialFast(-120);
	intake.spinIntake(true);
	chassis.moveDistance(300, 300, 1300);
	intake.spinIntake(false);
	chassis.turnInertialAbsolute(45);
	Flaps.set_value(true);
	chassis.move(400, 600);
	pros::delay(1500);
	chassis.stop();
	Flaps.set_value(false);
	chassis.moveDistance(-200, -200, 300);
	chassis.turnInertialAbsoluteFast(0);


	chassis.move(-300, -300);
	pros::delay(1000);
	chassis.stop();
	chassis.moveDistance(200, 200, 100);
	chassis.turnInertial(-60);
	Flaps.set_value(true);
	chassis.moveDistance(300, 300, 1300);
	Flaps.set_value(false);
	chassis.moveDistance(200, 400, 300);
	chassis.move(600, 100);
	pros::delay(500);
	chassis.move(600, 300);
	pros::delay(800);
	chassis.move(600, 600);
	pros::delay(1000);
	chassis.stop();
	chassis.move(-300, -300);
	pros::delay(500);
	chassis.stop();
}

void autonomous() {
	// chassis.turnInertial(90);
	// chassis.moveDistance(300, 300, 200);
	far5();
	// far();
	// close();
	// closeElim();
	// closeElimExperiment();
	// skills();
	// skills2();
	// chassis.turnInertial(90);
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
	bool dropdownRightToggle = false;
	bool dropdownLeftToggle = false;
	bool elevateToggle = false;
	bool flapsToggle = false;
	bool balanceToggle = false;

	// Loop that repeats forever until the program gets shut down
	while (true) { 
		// // Controls the movement of the drivetrain with the values of both joysticks
		chassis.inlineTelOp(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), 
		master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));

		intake.telOP(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1), 
		master.get_digital(pros::E_CONTROLLER_DIGITAL_R2));

		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
			shooter.move_velocity(100);
		}
		else{
			shooter.move_velocity(0);
		}

		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
			intake.spinIntake(true);
			chassis.moveDistance(-200, -200, 500);
			intake.stop();
			chassis.turnInertialAbsolute(-64);
			dropdownRight.set_value(true);
		}

		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)){
			flapsToggle = !flapsToggle;
			Flaps.set_value(flapsToggle);
		}

		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)){
			elevateToggle = !elevateToggle;
			elevate.set_value(elevateToggle);
		}

		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)){
			dropdownRightToggle = !dropdownRightToggle;
			dropdownRight.set_value(dropdownRightToggle);
		}

		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)){
			dropdownLeftToggle = !dropdownLeftToggle;
			dropdownLeft.set_value(dropdownLeftToggle);
		}

		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)){
			dropdownRightToggle = !dropdownRightToggle;
			dropdownRight.set_value(dropdownRightToggle);
			dropdownLeftToggle = !dropdownLeftToggle;
			dropdownLeft.set_value(dropdownLeftToggle);
		}

		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)){
			balanceToggle = !balanceToggle;
			balance.set_value(balanceToggle);
		}



		// // // Initialize the variable to store the temperature of motors
		// pros::lcd::set_text(0, "TEMPERATURES");
		// std::string temp = std::to_string((LEFT_FRONT.get_temperature() + LEFT_MIDDLE.get_temperature()
		//  + RIGHT_FRONT.get_temperature() + RIGHT_MIDDLE.get_temperature())/4);

		// // // Display the variable on the screen
		// pros::lcd::set_text(1, "Drivetrain: " + temp);

		// // repeat for all motors
		// temp = std::to_string((LEFT_BACK.get_temperature() + RIGHT_BACK.get_temperature())/2);
		// pros::lcd::set_text(2, "11W Catapult: " + temp);
		// temp = std::to_string((cataLeft.get_temperature() + cataRight.get_temperature())/2);
		// pros::lcd::set_text(3, "5.5W Catapult: " + temp);
		// temp = std::to_string(intakeMotor.get_temperature());
		// pros::lcd::set_text(4, "Intake: " + temp);

		
		pros::delay(20);
	}
}

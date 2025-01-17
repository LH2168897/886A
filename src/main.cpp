#include "main.h"

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */

	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::MotorGroup right_drive({5,9,3});    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
	pros::MotorGroup left_drive({-16,-12,-20});  // Creates a motor group with forwards port 5 and reversed ports 4 & 6
	pros::MotorGroup intake({13,10});
	pros::Motor lebron(15);
	pros::adi::Pneumatics clamp('h',false); 
	pros::adi::Pneumatics clamp2('a',false);
	pros::Imu imu(21);
	

void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "nelson is short af");
	imu.reset(); 
	delay(1000); 
	pros::lcd::register_btn1_cb(on_center_button);
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



void powerDrive(int forward, int turn){
	left_drive.move(forward+turn);
	right_drive.move(forward-turn);

}

int speedLimit(int speed, int max_speed){
	if(speed>max_speed){
		return(max_speed);
	} 
	else if(speed<-max_speed){
		return(-max_speed);
	}
	else{
		return(speed);
	}
}

double Inchtoticks(int distance) {
	return distance * 300 / (4 * 3.14159265) *7/3;
}

	
void movep(double Distance, int max_speed){
		int target = Inchtoticks(-1*Distance);
		left_drive.tare_position();
		int error = target - left_drive.get_position();
		int tt = millis();
		double kP = 0.35, kD = 0.5;
		int integral = 0;
		int pasterror;
		int derivative;

		while (millis() - tt < 1000){
			pasterror=error;
			error = target - left_drive.get_position();
			derivative=pasterror-error;
			if (abs(error) > 1){
				int tt = millis();
			}
			left_drive.move(error*kP + derivative*kD);
			right_drive.move(error*kP + derivative*kD);

			powerDrive(speedLimit(error*kP + integral, max_speed), 0);

			delay(20);
		}
		 powerDrive ( 0,0);
		left_drive.move(0);
		right_drive.move(0);
	}


void Turndrive(double degrees){
	imu.tare_rotation();
	double error = degrees - imu.get_rotation();
	int trackingTime = pros::millis();
	int timeout = pros::millis();
	double kP = 1, kD = 5;
	int pasterror;
	int derivative;

	while ((pros:: millis() - trackingTime < 800) && (pros::millis()-timeout < 3000)){
	error = degrees - imu.get_rotation();
		derivative+pasterror-error;
		if (abs(error) > 3){
			trackingTime = pros::millis();
					}
	powerDrive(0, error *kP + derivative*kD);
	}
	powerDrive(0, 0);

}

void blueleftside(){
	movep(-27,70);
	delay(600);
	clamp2.toggle();
	delay(200);
	intake.move(-100);
	delay(200);
	Turndrive(95);
	intake.move(-100);// eating the first ring 
	movep(15,70);
	delay(1400);
	Turndrive(185);
	movep(200,60);
	delay(1500);
	movep(0,0);                                                                                                                                                                                                                                                           
}

void bluerightside(){
	movep(-27,70);
	delay(600);
	clamp2.toggle();
	delay(200);
	intake.move(-100);
	delay(200);
	Turndrive(-95);
	intake.move(-100);// eating the first ring 
	movep(15,70);
	delay(1400);
	Turndrive(-185);
	movep(200,60);
	delay(1500);
	movep(0,0);
}

void redrightside(){
	movep(-27,70);
	delay(600);
	clamp2.toggle();
	delay(200);
	intake.move(-100);
	delay(200);
	Turndrive(-95);
	intake.move(-100);// eating the first ring 
	movep(15,70);
	delay(1400);
	Turndrive(-185);
	movep(200,60);
	delay(1500);
	movep(0,0);
}

void redleftside(){
	movep(-27,70);
	delay(600);
	clamp2.toggle();
	delay(200);
	intake.move(-100);
	delay(200);
	Turndrive(95);
	intake.move(-100);// eating the first ring 
	movep(15,70);
	delay(1400);
	Turndrive(185);
	movep(200,60);
	delay(1500);
	movep(0,0);
}


void back(){
	movep(-30, 90);
	delay(1000);
	clamp.toggle();
	intake.move(127);
	delay(50);
	movep(-4, 90);
}

void skills(){
	intake.move(127);
	delay(300);
	intake.move(0);
}


void autonomous(){
skills();
//bluerightside();
//redrightside();
//bluerightside();
//blueleftside();
//redleftside();
}

//auto 2



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


	int brownStatus = 0;

	while (true) {
		// Drive
		int dir = -1*master.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
		int turn = master.get_analog(ANALOG_RIGHT_X);  // Gets the turn left/right from right joystick
		left_drive.move(dir + turn);                       // Sets left motor voltage
		right_drive.move(dir - turn);
	

		//Arm
		if(master.get_digital(DIGITAL_L2)){
			intake.move(127);
		}
		else if(master.get_digital(DIGITAL_L1)){
			intake.move(-100);
		}
		else{
			intake.move(0);
		}

		if(master.get_digital_new_press(DIGITAL_R1)){
			if (brownStatus != 2)
				brownStatus++;
			/*if (brownStatus == 0) {
				lebron.move_absolute(100,75); //replace with collect thing
				brownStatus++;
			}
			else if (brownStatus == 1) {
				lebron.move_absolute(200,75); // replace with high stake
				brownStatus++;
			}*/
		}
		else if(master.get_digital_new_press(DIGITAL_R2)){
			if (brownStatus != 0)
				brownStatus--;
			/*if (brownStatus == 1) {
				lebron.move_absolute(0,75); //replace start
				brownStatus++;
			}
			else if (brownStatus == 2) {
				lebron.move_absolute(200,75); // replace with high stake
				brownStatus++;
			}*/
		}
		if (brownStatus == 0)
			lebron.move_absolute(0,127); //start height
		else if (brownStatus == 1)
			lebron.move_absolute(495,127); //ring height
		else if (brownStatus == 2)
			lebron.move_absolute(2000,127); //high stake height;

	
		//pnu
    	if(master.get_digital_new_press(DIGITAL_A)){
      		//clamp.toggle();
			clamp2.toggle();
    	}
		//else if(master.get_digital_new_press(DIGITAL_B)){
			//clamp2.toggle();
		//}
		if(master.get_digital_new_press(DIGITAL_B)){
      		clamp.toggle();
			//clamp2.toggle();999998
		}


pros::delay(20);  
 
           

}	

}

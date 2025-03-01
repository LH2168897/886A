#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/motors.h"

pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup right_drive({20,5,12},pros::MotorGearset::blue);    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
pros::MotorGroup left_drive({-16,-10,-9},pros::MotorGearset::blue);  // Creates a motor group with forwards port 5 and reversed ports 4 & 6
pros::Motor secondStage(14);
pros::MotorGroup intake({3,14});
pros::Motor suck(3);
pros::Motor lebron(4);
pros::adi::Pneumatics clamp('h',false); 
pros::adi::Pneumatics doinker('a',false);
pros::Imu imu(18);
pros::Optical optical(19);
// create a v5 rotation sensor on port 1
pros::Rotation vertical_encoder(6);
pros::Rotation horizontal_encoder(-13);

// drivetrain settings
lemlib::Drivetrain drivetrain(
	&left_drive, // left motor group
	&right_drive, // right motor group
	12.625, // 10 inch track width
	lemlib::Omniwheel::NEW_275, // using new 4" omnis
	480, // drivetrain rpm is 360
	2 // horizontal drift is 2 (for now)
);
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_2, -1.5);
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_2, -4.5);


lemlib::OdomSensors sensors(
	&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
	nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
	&horizontal_tracking_wheel, // horizontal tracking wheel 1
	nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
	&imu // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(
	15, // proportional gain (kP)
	0, // integral gain (kI)
	5, // derivative gain (kD)
	3, // anti windup
	1, // small error range, in inches
	100, // small error range timeout, in milliseconds
	3, // large error range, in inches
	500, // large error range timeout, in milliseconds
	20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(
	2, // proportional gain (kP)
	0, // integral gain (kI)
	10, // derivative gain (kD)
	3, // anti windup
	1, // small error range, in degrees
	100, // small error range timeout, in milliseconds
	3, // large error range, in degrees
	500, // large error range timeout, in milliseconds
	0 // maximum acceleration (slew)
);

// create the chassis
lemlib::Chassis chassis(
	drivetrain, // drivetrain settings
	lateral_controller, // lateral PID settings
	angular_controller, // angular PID settings
	sensors // odometry sensors
);
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

void initialize() {
	chassis.calibrate(); // calibrate sensors
	left_drive.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	right_drive.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
    pros::lcd::initialize(); // initialize brain screen

    // print position to brain screen
    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // delay to save resources
            pros::delay(20);
        }
    });
}

void disabled() {}

void competition_initialize() {}


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

	
void movep(double Distance, int max_speed, int timeOut = 1000){
		int target = Inchtoticks(Distance);
		left_drive.tare_position();
		int error = target - left_drive.get_position();
		int tt = millis();
		double kP = 0.35, kD = 1;
		int integral = 0;
		int pasterror;
		int derivative;

		while (millis() - tt < timeOut){
			error = target - left_drive.get_position();
			derivative=pasterror-error;
			if (abs(error) > 1){
				int tt = millis();
			}
			// left_drive.move(error*kP + derivative*kD);
			// right_drive.move(error*kP + derivative*kD);
			
			powerDrive(speedLimit(error*kP - derivative*kD, max_speed), 0);
			
			pasterror=error;
			delay(20);
		}
		 powerDrive ( 0,0);
		left_drive.move(0);
		right_drive.move(0);
	}


void Turndrive(double degrees, int timeLimit = 3000){
	imu.tare_rotation();
	double error = degrees - imu.get_rotation();
	int trackingTime = pros::millis();
	int timeout = pros::millis();
	double kP = 1, kD = 1.5;
	int pasterror;
	int derivative;

	while ((pros:: millis() - trackingTime < 1000) && (pros::millis()-timeout < timeLimit)){
		error = degrees - imu.get_rotation();
		derivative=pasterror-error;
		if (error > 0.5){
			trackingTime = pros::millis();
					}
		powerDrive(0, error *kP - derivative*kD);
		pasterror=error;
		delay(20);
	}
	powerDrive(0, 0);

}

void blueleftside(){
	movep(-27,70);
	delay(600);
	clamp.toggle();
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
	clamp.toggle();
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
	clamp.toggle();
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
	clamp.toggle();
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

void skills(){

	/*! - red right
	//alliance stake
	intake.move(-127);
	delay(1000);
	intake.move(0);*/

	//clamp MG
	movep(10,90,1000);
	Turndrive(-93,1000);
	movep(-18, 55, 1000);
	delay(500);
	clamp.toggle();
	delay(200);

	/*//#1
	intake.move(-100);
	Turndrive(100,1000);
	movep(19,127);
	delay(500);
	//#2
	Turndrive(100,1000);
	movep(18,127);
	delay(500);*/

	/*
	//score
	lebron.move_absolute(1700,127);
	movep(5,100);
	delay(100);
	movep(-5,100);

	lebron.move_absolute(485,127);
	intake.move(-100);
	delay(250);

	//#3+4+5
	Turndrive(135);
	movep(28,110);
	delay(750);
	movep(7,110);
	Turndrive(180);
	movep(15,127);
	//pos corner
	Turndrive(-90);
	movep(-20,127);
	clamp.toggle();
	delay(250);

	//transfer to red left
	movep(15,127);
	intake.move(0);
	Turndrive(150);
	movep(10,127);
	delay(250);
	movep(-90,127);

	*/

	//! - red left (copy)

	/*
	//transfer to blue left + one ring
	movep(15,127);
	Turndrive(-30);
	movep(-10,127);
	delay(250);
	movep(80,127);
	intake.move(-127);
	delay(250);
	movep(10,127);
	intake.move(0);
	movep(60,127);

	//! - blue left
	//push goal
	Turndrive(-90);
	movep(-25,55);
	delay(250);
	clamp.toggle();
	Turndrive(-165);
	movep(-50,127);
	clamp.toggle();
	delay(500);

	//last stack
	intake.move(-127);
	movep(80,127);
	Turndrive(180);
	movep(-20,55);
	clamp.toggle();
	Turndrive(-70);
	movep(30,127);
	Turndrive(-90);
	movep(30,127);
	Turndrive(-90);
	movep(30,127);
	Turndrive(45);
	movep(20,127);
	Turndrive(120);
	movep(-40,127);

	*/
}

void skills2(){ 
	chassis.setPose(0, -61, 0);

	//alliance stake
	secondStage.move(-127);
	delay(1000);
	secondStage.move(0);

	//red right
	chassis.moveToPose(0, -48, 0, 2000, {.forwards = true,  .minSpeed = 60}); //mg clamp
	delay(1000);
	chassis.moveToPose(0, -48, -90, 2000, {.forwards = true,   .minSpeed = 60});
	chassis.waitUntilDone();
	chassis.moveToPose(27, -48, -90,2000, {.forwards = false,  .minSpeed = 60});
	chassis.waitUntilDone();
	clamp.toggle();
	delay(1000);
	intake.move(-100); //ring 1
	delay(1000);
	chassis.moveToPose(24, -47, 0, 2000, {.forwards = true});
	chassis.moveToPose(24, -24, 0, 2000, {.forwards = true, .minSpeed = 80});
	delay(1000); //ring 2
	chassis.moveToPose(24, -24, 90, 2000, {.forwards = true, .minSpeed = 60}); //ring 2
	chassis.moveToPose(53, -24, 90, 2000, {.forwards = true,  .minSpeed = 60});
	delay(1000);
	chassis.moveToPose(51, -24, 18, 2000, {.forwards = true,   .minSpeed = 60});
	chassis.moveToPose(60, 9, 18, 2000, {.forwards = true, .minSpeed = 60}); //ring 3
	delay(1000);
	chassis.moveToPose(49, -10, 18, 1000, {.forwards = false,  .minSpeed = 60});
	delay(1000);
	chassis.turnToHeading(180, 2000);
	delay(1000);
	chassis.moveToPose(49, -60, 180, 3000, {.forwards = true,   .minSpeed = 60});
	chassis.waitUntilDone(); 

	chassis.moveToPose(49, -40, 180, 2000, {.forwards = false,   .minSpeed = 60});
	delay(1000);
	chassis.moveToPose(49, -40, 90, 2000, {.forwards = true,   .minSpeed = 90});
	chassis.moveToPose(63, -45, 90, 2000, {.forwards = true,   .minSpeed = 90});
	delay(2000);
	chassis.moveToPose(63, -45, -30, 2000, {.forwards = true, .minSpeed = 90});

	chassis.moveToPose(70, -60, -30, 2000, {.forwards = false, .maxSpeed = 70}); //drop goal 
	chassis.waitUntilDone();
	delay(1000);
	clamp.toggle();
	intake.move(0);

	chassis.moveToPose(60, -41, -30, 2000);
	delay(1000);
	chassis.moveToPose(60, -41, 90, 2000);
	
	chassis.moveToPose(-18, -45, 90, 4000, {.forwards = false, .maxSpeed = 100}); //mg goal
	chassis.moveToPose(-24, -45, 90, 4000, {.forwards = false, .maxSpeed = 55});
	chassis.waitUntilDone();
	delay(1000);
	clamp.toggle();
	chassis.moveToPose(-19, -41, 0, 2000); //ring 1
	intake.move(-100);
	chassis.moveToPose(-19, -19, 0, 2000);
	delay(1000);
	chassis.moveToPose(-21, -16, -90, 2000); //ring 2
	chassis.moveToPose(-45, -16, -90, 2000);
	chassis.moveToPose(-45, -16, -35, 2000); //ring 3
	chassis.moveToPose(-57, 8, -35, 2000);
	chassis.moveToPose(-45, -16, -35, 2000, {.forwards = false});
	chassis.moveToPose(-45, -16, -180, 2000);
	chassis.moveToPose(-49, -45, -180, 2000);
	chassis.moveToPose(-47, -45, -180, 2000);
	chassis.moveToPose(-49, -45, -90, 2000);
	chassis.moveToPose(-49, -47, -90, 2000);
	chassis.moveToPose(-75, -75, -180, 2000, {.forwards = false});

}

void autonomous(){
	skills2();
	//skills();
	//bluerightside();
	//blueleftside();
	//redrightside();
	//redleftside();
}

void opcontrol() {

	int brownStatus = 0;
	optical.set_led_pwm(100);

	while (true) {
		// Drive
		int dir = master.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
		int turn = master.get_analog(ANALOG_RIGHT_X);  // Gets the turn left/right from right joystick
		left_drive.move(dir + turn);                       // Sets left motor voltage
		right_drive.move(dir - turn);
	
		pros::lcd::set_text(2, std::to_string(optical.get_hue()));

		//Intake
		if(master.get_digital(DIGITAL_L2)){
			intake.move(127);
		}
		else if(master.get_digital(DIGITAL_L1)){
			intake.move(-100);
		}
		else{
			intake.move(0);
		}


		//Arm
		if(master.get_digital_new_press(DIGITAL_R1)){
			if (brownStatus != 2)
				brownStatus++;
		}
		else if(master.get_digital_new_press(DIGITAL_R2)){
			if (brownStatus != 0)
				brownStatus--;
		}
		if (brownStatus == 0)
			lebron.move_absolute(0,127); //start height
		else if (brownStatus == 1)
			lebron.move_absolute(485,127); //ring height
		else if (brownStatus == 2)
			lebron.move_absolute(1700,127); //high stake height;

	
		//pnu
    	if(master.get_digital_new_press(DIGITAL_A)){
			doinker.toggle();
    	}
		if(master.get_digital_new_press(DIGITAL_B)){
      		clamp.toggle();
		}

		pros::delay(20);  
 
	}	

}
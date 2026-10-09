#include "vex.h"

vex::brain Brain;
vex::controller Controller;


Chassis chassis(
    // Left drivetrain motors (left/right is looking from behind the robot)
    mik::motor_group({
        mik::motor(PORT10, false, blue_6_1, "left front motor"), 
        mik::motor(PORT3, true, blue_6_1, "left back motor"),
    }),
    // // Right drivetrain motors
    mik::motor_group({
        mik::motor(PORT6, false, blue_6_1, "right front motor"),
        mik::motor(PORT5, true, blue_6_1, "right back motor"),
    }),
    // mik::motor_group({
    //     mik::motor(PORT1, true, blue_6_1, "left front motor"), 
    //     mik::motor(PORT2, true, blue_6_1, "left back motor"),
    // }),
    // // Right drivetrain motors
    // mik::motor_group({
    //     mik::motor(PORT9, false, blue_6_1, "right front motor"),
    //     mik::motor(PORT10, false, blue_6_1, "right back motor"),
    // }),
	
    PORT0,  // Inertial sensor port
    360,    // Inertial scale (rotation reading after a full 360° turn)
	false,  // Forces inertial sensor to recalibrate until it is within minimum threshold of 0.05 deg for 1 second
	
    2.75,   // Drivetrain wheel diameter (in). Negative flips direction.
    450,    // Drivetrain RPM. Cartridge * gear ratio, (Ex: 600 * (36/48) = 450).

    PORT0,  // Forward tracker port. PORT0 if unused. Accepts "PORT_A"
    2,      // Forward tracker wheel diameter (in). Negative flips direction. Pushing robot forward at 0° should increase Y
    0,      // Forward tracker center distance (in). Positive = right of center, negative = left.

    PORT0,  // Sideways tracker port. PORT0 if unused. Accepts "PORT_A"
    -2,     // Sideways tracker wheel diameter (in). Negative flips direction. Pushing robot right at 0° should increase X
    0,      // Sideways tracker center distance (in). Positive = behind center, negative = in front.

    // Distance sensors mounted on a face of the robot
    mik::distance_reset({
        mik::distance(
			PORT0,		   // Distance sensor port
            rear_sensor,   // "front_sensor", "rear_sensor", "left_sensor", "right_sensor"
            4,             // X offset from tracking center (in). Positive = right of center, negative = left. 
            6              // Y offset from tracking center (in). Positive = in front of center, negative = behind.
        ),
        mik::distance(PORT0, left_sensor, -6, 4),
    })
);

// Add your devices in assembly.h then create them here

/* Creating a motor group in assembly */
// mik::motor_group Assembly::lower_intake_motors({
// 	mik::motor(PORT8, true, green_18_1, "bottom_intake"),
// 	mik::motor(PORT9, false, green_18_1, "middle_intake")
// });


//Cascade Motors
mik::motor Assembly::cascade_motorL1(PORT11, false, blue_6_1, "cascade_motorL1");
mik::motor Assembly::cascade_motorL2(PORT12, true, blue_6_1, "cascade_motorL2");
mik::motor Assembly::cascade_motorR1(PORT19, false, blue_6_1, "cascade_motorR1");
mik::motor Assembly::cascade_motorR2(PORT20, true, blue_6_1, "cascade_motorR2");

// Groups share the motors above (pointer overload) instead of copying them.
// The explicit vector type is required: a bare {&a, &b} is ambiguous with the
// const std::vector<mik::motor>& overload.
mik::motor_group Assembly::cascade_right(std::vector<mik::motor*>{&cascade_motorR1, &cascade_motorR2});
mik::motor_group Assembly::cascade_left(std::vector<mik::motor*>{&cascade_motorL1, &cascade_motorL2});	

/* Creating alternative vex devices in assembly */
mik::piston Assembly::claw(PORT_A, false); 




Assembly assembly;
Constants constants;
vex::competition Competition;

static void loading_screen(bool stop) {
	static vex::task loading_bar;
	
	if (stop) {
		loading_bar.stop();
		Brain.Screen.printAt(184, 220, "                        ");
		return;
	}
	
#ifndef FAST_COMPILE
	Brain.Screen.drawImageFromBuffer((uint8_t*)mikLib_logo, 0, 0, mikLib_logo_size);
#endif

	loading_bar = vex::task([](){
		std::string calibrate = "Calibrating";
#ifndef FAST_COMPILE
		Brain.Screen.setFillColor(mik::loading_text_bg_color.c_str());
		Brain.Screen.setPenColor(mik::loading_text_color.c_str());
#endif
		int count = 0;
		while(1) {
#ifdef FAST_COMPILE
			Brain.Screen.setFont(vex::fontType::mono40);
			Brain.Screen.setPenColor("#999999");
			Brain.Screen.printAt(160, 135, "mik");

			Brain.Screen.setFont(vex::fontType::mono60);
			Brain.Screen.setPenColor("#FFFFFF");
			Brain.Screen.printAt(227, 135, "Lib");
#endif
			Brain.Screen.setFont(vex::fontType::mono20);
			Brain.Screen.printAt(184, 220, calibrate.c_str());
			task::sleep(200);
			calibrate.append(".");
			count++;
			if (count > 4) {
				count = 0;
				calibrate = "Calibrating";
				Brain.Screen.printAt(184, 220, (calibrate + "     ").c_str());
			}
		}
		return 0;
	});
}

static void handle_disconnected_devices() {
#ifndef FAST_COMPILE
	int errors = run_diagnostic();
	if (errors > 0) {
		Controller.rumble(".");
		Controller.Screen.setCursor(1, 1);
		Controller.Screen.print((to_string(errors) + " ERRORS DETECTED").c_str());
		Controller.Screen.setCursor(2, 1);
		Controller.Screen.print("[Config]->[Error Data]");
		task::sleep(500);
		Controller.Screen.clearScreen();
	}
#endif
}

void init(init_options options) {
	// Disable user control during initialization to prevent inputs
	disable_user_control(false);
	
	// Start loading screen
	loading_screen(false);

	// Check disconnected devices
	if (options.check_disconnected_devices) handle_disconnected_devices();
	
	// Load auton selector
	if (options.enable_controller_selector) UI_controller_auton_selector();

	// Setup motors
#ifndef FAST_COMPILE
	motors_scr->init_motors();
#endif

	// Calibrate inertial
	chassis.calibrate_inertial();

	// Stop loading screen
	loading_screen(true);
}

static bool user_control_disabled = false;

void disable_user_control(bool stop_all_motors_) {
	user_control_disabled = true;
	if (stop_all_motors_) {
		stop_all_motors(vex::brakeType::hold);
		set_brake_all_motors(vex::brakeType::coast);
		stop_all_motors(vex::brakeType::coast);
	}
}

void enable_user_control(void) {
  	user_control_disabled = false;
}

bool control_disabled(void) {
	if (Competition.isDriverControl() && (Competition.isFieldControl() || Competition.isCompetitionSwitch()) && user_control_disabled) {
		auton_scr->disable_controller_overlay();
		return false;
	};
  	return user_control_disabled;
}

void stop_all_motors(vex::brakeType mode) {
	for (auto motor : mik::motor_registry()) {
		motor->stop(mode);
	}
}

void set_brake_all_motors(vex::brakeType mode) {  
	for (auto motor : mik::motor_registry()) {
		motor->setBrake(mode);
	}
}


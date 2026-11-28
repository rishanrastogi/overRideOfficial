#include "vex.h"

void default_constants() {
    // Each controller constant in the form of throttle, turn (deadband, min_output, curve_gain), desaturate_bias.
    chassis.set_control_constants(5, 10, 1.019, 5, 10, 1.019, 0.5);

    // Cheesy drive constants in the form of (turn_nonlinearity, neg_inertia_scalar, sensitivity, slew).
    chassis.set_cheesy_drive_constants(0.5, 3.0, 0.5, 0.1);

    // Each constant set is in the form of (maxVoltage, kP, kI, kD, startI, slew).
    chassis.set_drive_constants(12, 2.120, 0.040, 13.750, 3.000, 2);
    chassis.set_heading_constants(10, 0.500, 0.010, 4.800, 0, 0);
    chassis.set_turn_constants(12, 0.500, 0.020, 3.130, 15, 0);
    chassis.set_swing_constants(12, 0.400, 0.010, 2.100, 15, 0);

    // Each exit condition set is in the form of (settle_error, settle_time, timeout).
    chassis.set_turn_exit_conditions(1, 200, 3000);
    chassis.set_drive_exit_conditions(2, 200, 5000);
    chassis.set_swing_exit_conditions(1, 200, 3000);

    constants.boomerang_lead = .5;
    constants.boomerang_drift = 2;
}

/**
 * @brief Turns to a pin and drives at it fast, slowing down once the clamp distance sensor sees it, then waits for auto-clamp.
 * @param X_position, Y_position Pin location to turn toward.
 * @param max_distance Farthest it will drive (in) if the sensor never sees the pin.
 * @param slow_mm Sensor reading (mm) where it switches from fast to slow.
 * @param slow_voltage Drive voltage for the slow approach.
 * @param clamp_timeout_ms Passed to wait_for_clamp(): forces the grab after this long.
 */
static void drive_to_pin(float X_position, float Y_position, float max_distance = 24, float slow_mm = 150, float slow_voltage = 2.5, int clamp_timeout_ms = 300) {
    chassis.turn_to_point(X_position, Y_position);
    assembly.arm_auto_clamp(); // turned away from the last drop: ready to grab the pin

    auto pin_within = [](float max_mm) {
        return Assembly::clamp_distance.isObjectDetected() && Assembly::clamp_distance.objectDistance(vex::distanceUnits::mm) <= max_mm;
    };

    // Fast until the sensor sees the pin
    float start = chassis.get_forward_tracker_position();
    chassis.drive_distance(max_distance, {.max_voltage =4, .wait = false});
    while (chassis.is_in_motion() && !pin_within(slow_mm)) {
        vex::task::sleep(10);
    }
    float traveled = chassis.get_forward_tracker_position() - start;
    chassis.cancel_motion();

    // Slow for the rest, until auto-clamp grabs it
    float remaining = max_distance - traveled;
    if (remaining > 0 && Assembly::claw_is_open()) {
        chassis.drive_distance(remaining, {.max_voltage = slow_voltage, .wait = false});
        while (chassis.is_in_motion() && Assembly::claw_is_open()) {
            vex::task::sleep(10);
        }
        chassis.cancel_motion();
    }
    assembly.wait_for_clamp(clamp_timeout_ms); // forces the grab if auto-clamp didn't
}

/* When creating autons with odometry, use a path planner, https://path.jerryio.com/.
If your robot is not lining up with the points, double check you set up your odometry correctly */

std::string blue_left_winpoint(bool calibrate, auto_variation var, bool get_name) {
    if (get_name) return "blue left winpoint";

    // Mirror autons here

    if (calibrate) {
        // Run startup sequence that you want in pre-auton, do before field controller plug in
        // chassis.set_coordinates(0, 0, 0);

        //toggle
        chassis.drive_distance(8);
        chassis.drive_distance(-10);
        chassis.drive_distance(10);
        chassis.drive_distance(-20);

        return "";
    }    

    // Place start of autonoumous here
    
    return "";
}

//wall pin nearside auton
std::string blue_left_two_goalpin(bool calibrate, auto_variation var, bool get_name) {
    if (get_name) return "su: left on roller touching";

    if (calibrate) {
        chassis.set_coordinates(0, 0, 0);

        return "";
    }
    assembly.toggle.setVelocity(100,percent);
    assembly.toggle.spinFor(2.25,rev);
    assembly.toggle.setVelocity(100,percent);
    assembly.flip_toggle_to_blue();

    chassis.drive_distance(7);

    assembly.sixbar_left.spinFor(fwd, 0.25, rev, 6, volt, false);
    assembly.sixbar_right.spinFor(reverse, 0.25, rev, 6, volt, false);

    chassis.left_swing_to_angle(90, {.timeout = 300}); //swing into cup
    chassis.drive_distance(5.25, {.timeout = 525});

    assembly.sixbar_left.spinFor(reverse, 0.30, rev, 3.5, volt, false);
    assembly.sixbar_right.spinFor(fwd, 0.30, rev, 3.5, volt);
    wait(0.1,seconds);
    assembly.claw.open();
    wait(0.1,seconds);

    chassis.drive_distance(-7,{.heading=180});
    
    
    chassis.turn_to_angle(135);
    chassis.drive_distance(18.75);
    chassis.left_swing_to_angle(180,{.timeout = 500}); //get to side pin
    assembly.claw.close();
    chassis.drive_distance(1,{.timeout = 100});
    assembly.claw.close();
    wait(0.15,seconds);
    
    assembly.sixbar_left.spinFor(fwd, 0.45, rev, 6, volt, false);
    assembly.sixbar_right.spinFor(reverse, 0.45, rev, 6, volt);
    chassis.turn_to_angle(100,{.timeout = 500});
    chassis.drive_distance(-16,{.heading=180});
    chassis.turn_to_angle(90);
    chassis.drive_distance(10, {.timeout = 500}); //align with goal
    
    assembly.sixbar_left.spinFor(reverse, 0.35, rev, 3.5, volt, false);
    assembly.sixbar_right.spinFor(fwd, 0.35, rev, 3.5, volt);
    wait(0.15,seconds);
    assembly.claw.open();
    wait(0.1,seconds); //drop stack on

    chassis.drive_distance(-6, {.heading=90});
    chassis.turn_to_angle(241);
    chassis.drive_distance(29.5,{.heading = 241});
    chassis.right_swing_to_angle(180,{.timeout = 450, .min_voltage = 12});
    chassis.drive_distance(1.5,{.timeout = 100});
    assembly.claw.close();
    wait(0.15,seconds);
    assembly.sixbar_left.spinFor(fwd, 0.4, rev, 7, volt, false);
    assembly.sixbar_right.spinFor(reverse, 0.4, rev, 7, volt, false);
    chassis.drive_distance(1,{.heading = 180});
    chassis.turn_to_angle(0); 

    chassis.drive_distance(15,{.timeout=300});
    assembly.sixbar_left.spinFor(reverse, 0.35, rev, 4, volt, false);
    assembly.sixbar_right.spinFor(fwd, 0.35, rev, 4, volt);
    wait(0.15,seconds);
    assembly.claw.open(); // final
    
    return "";
}

//standing pin far side code
std::string blue_left_two_goalpin_standingpin(bool calibrate, auto_variation var, bool get_name){
    
    assembly.claw_grab(); // hold the preload (claw starts physically open at power-on)
    assembly.toggle.setVelocity(100,percent);
    assembly.flip_toggle_to_red();

    assembly.sixbar_left.spinFor(fwd, 0.10, rev, 6, volt, false);
    assembly.sixbar_right.spinFor(reverse, 0.10, rev, 6, volt, false);

    chassis.drive_to_point(0.125, 8.348);
    assembly.sixbar_left.spinFor(fwd, 0.15, rev, 6, volt, false);
    assembly.sixbar_right.spinFor(reverse, 0.15, rev, 6, volt, false);
    chassis.left_swing_to_angle(90, {.timeout = 650}); //swing into cup
    chassis.drive_distance(7.5, {.max_voltage = 12, .timeout = 750});
    assembly.sixbar_left.spinFor(reverse, 0.30, rev, 3.5, volt, false);
    assembly.sixbar_right.spinFor(fwd, 0.30, rev, 3.5, volt);
    wait(0.1,seconds);
    assembly.claw_release(); // drop preload on cup. Auto-clamp won't re-grab it: re-arms once we back off 120 mm
    wait(0.1,seconds);

    
    chassis.drive_distance(-6,{.heading = 90});
    chassis.turn_to_point(19.625, 38.348);
    assembly.arm_auto_clamp();
    chassis.drive_distance(19);
    chassis.drive_distance(5,{.max_voltage = 2.5});
    assembly.wait_for_clamp(300); 
    // auto-clamp grabs the first freestanding one; forces the grab after 300 ms
    // drive_to_pin(-27.5, -28.75,250,2.5); // fast at the first freestanding one, slow once the sensor sees it, then auto-clamp
    // chassis.drive_to_point(-26.5, -25.5, {.max_voltage = 8}); // get to first freestanding one

    assembly.sixbar_left.spinFor(fwd, 0.45, rev, 6, volt, false);
    assembly.sixbar_right.spinFor(reverse, 0.45, rev, 6, volt, false);
    chassis.turn_to_point(19.625, 24.098);
    chassis.drive_distance(15,{.max_voltage = 8, .timeout = 1000});

    assembly.sixbar_left.spinFor(reverse, 0.30, rev, 3, volt, false);
    assembly.sixbar_right.spinFor(fwd, 0.30, rev, 3, volt);
    wait(0.1,seconds);
    assembly.claw_release(); // drop first stack on cup. Auto-clamp won't re-grab it: re-arms once we back off 120 mm
    wait(0.1,seconds);

    chassis.drive_distance(-12,{.heading=90});
    chassis.turn_to_point(32.795, 20.668);
    assembly.arm_auto_clamp(); // turned away from the dropped stack: ready to grab the next one
    chassis.drive_to_point(32.795, 20.668);
    chassis.drive_distance(5,{.max_voltage = 4});
    assembly.wait_for_clamp(300); // auto-clamp grabs the second freestanding one; forces the grab after 300 ms

    assembly.sixbar_left.spinFor(fwd, 0.45, rev, 6, volt, false);
    assembly.sixbar_right.spinFor(reverse, 0.45, rev, 6, volt, false);
    chassis.turn_to_point(20.315, 13.368);
    chassis.drive_distance(15,{.max_voltage = 8, .timeout = 1000});
    assembly.sixbar_left.spinFor(reverse, 0.30, rev, 3.5, volt, false);
    assembly.sixbar_right.spinFor(fwd, 0.30, rev, 3.5, volt);
    wait(0.1,seconds);
    assembly.claw_release(); // drop second stack on cup. Auto-clamp won't re-grab it: re-arms once we back off 120 mm
    wait(0.2,seconds);

    chassis.drive_distance(-10,{.min_voltage = 12});

    return "";
}


//wall pin far side code - this the one max wants to run
std::string red_right_two_goalpin(bool calibrate, auto_variation var, bool get_name) {
    if (get_name) return "su: left on roller touching";

    if (calibrate) {
        chassis.set_coordinates(-66.013, -8.701, 0);
        assembly.claw_grab();
        return "";
    }
    // assembly.toggle.setVelocity(100,percent);
    // assembly.toggle.spinFor(2.25,rev);
    assembly.toggle.setVelocity(100,percent);
    assembly.flip_toggle_to_red();
    assembly.sixbar_left.spinFor(fwd, 0.20, rev, 6, volt, false);
    assembly.sixbar_right.spinFor(reverse, 0.20, rev, 6, volt, false);
    chassis.drive_distance(7.25);

    chassis.right_swing_to_angle(270, {.timeout = 800}); //swing into cup
    chassis.drive_distance(7, {.timeout = 600});

    chassis.drive_distance(-1);
    assembly.sixbar_left.spinFor(reverse, 0.25, rev, 3.5, volt, false);
    assembly.sixbar_right.spinFor(fwd, 0.25, rev, 3.5, volt);
    assembly.claw_release(); // drop preload on cup
    wait(0.2,seconds);
    chassis.drive_distance(-2);

    chassis.drive_distance(-7.5,{.heading=180});
    chassis.turn_to_angle(230, {.timeout = 450});
    assembly.arm_auto_clamp(); // armed before swinging into the wall
    chassis.drive_distance(18, {.timeout = 800});
    chassis.right_swing_to_angle(180,{.timeout = 775}); //get to side pin
    chassis.drive_distance(1,{.max_voltage = 4, .timeout = 100});
    assembly.wait_for_clamp(200); // auto-clamp grabs the pin; forces the grab after 300 ms

    assembly.sixbar_left.spinFor(fwd, 0.45, rev, 6, volt, false);
    assembly.sixbar_right.spinFor(reverse, 0.45, rev, 6, volt, false);
    chassis.turn_to_angle(260,{.timeout = 500});
    chassis.drive_distance(-17,{.heading = 180});
    chassis.turn_to_angle(270, {.timeout = 600});
    chassis.drive_distance(9.4, {.timeout = 500}); //align with goal

    assembly.sixbar_left.spinFor(reverse, 0.35, rev, 4, volt, false);
    assembly.sixbar_right.spinFor(fwd, 0.35, rev, 4, volt);
    assembly.claw_release();
    chassis.drive_distance(-4, {.heading=270, .timeout = 500});
    chassis.drive_distance(-29.5, {.min_voltage = 12, .heading = 180, .timeout = 1300});

    chassis.drive_distance(27, {.min_voltage = 8, .heading = 168});
    assembly.arm_auto_clamp(); // armed before swinging into the wall
    chassis.right_swing_to_angle(130, {.timeout = 500});
    chassis.drive_distance(6);
    chassis.left_swing_to_angle(180, {.timeout = 500});
    chassis.drive_distance(3, {.max_voltage = 4, .timeout = 300});
    assembly.wait_for_clamp(200); // auto-clamp grabs the pin; forces the grab after 300 ms

    assembly.sixbar_left.spinFor(fwd, 0.45, rev, 7, volt, false);
    assembly.sixbar_right.spinFor(reverse, 0.45, rev, 7, volt, false);
    chassis.drive_distance(-1, {.min_voltage = 12, .timeout = 100});
    chassis.turn_to_angle(0, {.min_voltage = 12, .timeout = 1000});
    chassis.drive_distance(100, {.min_voltage = 12, .wait = false, .timeout = 1000});
    wait(0.2, seconds);
    assembly.sixbar_left.spinFor(reverse, 0.5, rev, 4, volt, false);
    assembly.sixbar_right.spinFor(fwd, 0.5, rev, 4, volt);
    assembly.claw_release();


    // chassis.drive_distance(-6, {.heading=90});
    // chassis.turn_to_angle(241);
    // chassis.drive_distance(29.5,{.heading = 241});
    // chassis.right_swing_to_angle(180,{.timeout = 450, .min_voltage = 12});
    // chassis.drive_distance(1.5,{.timeout = 100});
    // assembly.claw.close();
    // wait(0.15,seconds);
    // assembly.sixbar_left.spinFor(fwd, 0.4, rev, 7, volt, false);
    // assembly.sixbar_right.spinFor(reverse, 0.4, rev, 7, volt, false);
    // chassis.drive_distance(1,{.heading = 180});
    // chassis.turn_to_angle(0); 

    // chassis.drive_distance(15,{.timeout=300});
    // assembly.sixbar_left.spinFor(reverse, 0.35, rev, 4, volt, false);
    // assembly.sixbar_right.spinFor(fwd, 0.35, rev, 4, volt);
    // wait(0.15,seconds);
    // assembly.claw.open(); // final

    return "";
}


//wall pin near side code
std::string red_left_two_goalpin(bool calibrate, auto_variation var, bool get_name) {
    if (get_name) return "su: left on roller touching";

    if (calibrate) {
        chassis.set_coordinates(-66.013, -8.701, 0);

        return "";
    }

    assembly.toggle.setVelocity(100,percent);
    assembly.flip_toggle_to_red();

    chassis.drive_distance(7);

    assembly.sixbar_left.spinFor(fwd, 0.25, rev, 6, volt, false);
    assembly.sixbar_right.spinFor(reverse, 0.25, rev, 6, volt, false);

    chassis.left_swing_to_angle(90); //swing into cup
    chassis.drive_distance(5.25, {.timeout = 525});

    assembly.sixbar_left.spinFor(reverse, 0.30, rev, 3.5, volt, false);
    assembly.sixbar_right.spinFor(fwd, 0.30, rev, 3.5, volt);
    wait(0.1,seconds);
    assembly.claw.open();
    wait(0.1,seconds);

    chassis.drive_distance(-7,{.heading=180});
    chassis.turn_to_angle(135);
    chassis.drive_distance(18.75);
    chassis.left_swing_to_angle(180,{.timeout = 500}); //get to side pin
    chassis.drive_distance(1,{.timeout = 100});
    assembly.claw.close();
    wait(0.15,seconds);
    
    assembly.sixbar_left.spinFor(fwd, 0.45, rev, 6, volt, false);
    assembly.sixbar_right.spinFor(reverse, 0.45, rev, 6, volt);
    chassis.turn_to_angle(100,{.timeout = 500});
    chassis.drive_distance(-16,{.heading=180});
    chassis.turn_to_angle(90);
    chassis.drive_distance(10, {.timeout = 500}); //align with goal
    
    assembly.sixbar_left.spinFor(reverse, 0.35, rev, 3.5, volt, false);
    assembly.sixbar_right.spinFor(fwd, 0.35, rev, 3.5, volt);
    wait(0.15,seconds);
    assembly.claw.open();
    wait(0.1,seconds); //drop stack on

    chassis.drive_distance(-6, {.heading=90});
    chassis.turn_to_angle(241);
    chassis.drive_distance(29.5,{.heading = 241});
    chassis.right_swing_to_angle(180,{.timeout = 450, .min_voltage = 12});
    chassis.drive_distance(1.5,{.timeout = 100});
    assembly.claw.close();
    wait(0.15,seconds);
    assembly.sixbar_left.spinFor(fwd, 0.4, rev, 7, volt, false);
    assembly.sixbar_right.spinFor(reverse, 0.4, rev, 7, volt, false);
    chassis.drive_distance(1,{.heading = 180});
    chassis.turn_to_angle(0); 

    chassis.drive_distance(15,{.timeout=300});
    assembly.sixbar_left.spinFor(reverse, 0.35, rev, 4, volt, false);
    assembly.sixbar_right.spinFor(fwd, 0.35, rev, 4, volt);
    wait(0.15,seconds);
    assembly.claw.open(); // final

    return "";
}


std::string blue_left_elim(bool calibrate, auto_variation var, bool get_name) {
    if (get_name) return "blue left elim";

    // Mirror autons here

    if (calibrate) {
        // Run startup sequence that you want in pre-auton, do before field controller plug in
        // chassis.set_coordinates(0, 0, 0);

        return "";
    }

    // Place start of autonoumous here

    return "";
}


// PLACEHOLDER: blue right two goalpin auton isn't written yet (does nothing). Needed so the project links.
std::string blue_right_two_goalpin(bool calibrate, auto_variation var, bool get_name) {
    if (get_name) return "blue right two goalpin (empty)";

    if (calibrate) {
        chassis.set_coordinates(-66.013, -8.701, 0);

        return "";
    }

    return "";
}

//old standing pin far side code
// std::string blue_right_two_goalpin(bool calibrate, auto_variation var, bool get_name) {
//     if (get_name) return "su: left on roller touching";

//     if (calibrate) {
//         chassis.set_coordinates(-66.013, -8.701, 0);

//         return "";
//     }
//     assembly.toggle.setVelocity(100,percent);
//     assembly.flip_toggle_to_blue();

//     chassis.drive_distance(6.5);

//     assembly.sixbar_left.spinFor(fwd, 0.25, rev, 6, volt, false);
//     assembly.sixbar_right.spinFor(reverse, 0.25, rev, 6, volt, false);

//     chassis.right_swing_to_angle(270); //swing into cup
//     chassis.drive_distance(7, {.timeout = 700});

//     assembly.sixbar_left.spinFor(reverse, 0.30, rev, 3.5, volt, false);
//     assembly.sixbar_right.spinFor(fwd, 0.30, rev, 3.5, volt);
//     wait(0.1,seconds);
//     assembly.claw.open();
//     wait(0.75,seconds);

//     chassis.drive_distance(-2);
//     wait(0.1,seconds);
//     chassis.drive_distance(1.75);
//     chassis.drive_distance(-8.5,{.heading=180});
//     chassis.turn_to_angle(225);
//     chassis.drive_distance(18.75);
//     chassis.right_swing_to_angle(180,{.timeout = 500}); //get to side pin
//     chassis.drive_distance(1,{.timeout = 100});
//     assembly.claw.close();
//     wait(0.15,seconds);
    
//     assembly.sixbar_left.spinFor(fwd, 0.45, rev, 6, volt, false);
//     assembly.sixbar_right.spinFor(reverse, 0.45, rev, 6, volt);
//     chassis.turn_to_angle(260,{.timeout = 500});
//     chassis.drive_distance(-16,{.heading=180});
//     chassis.turn_to_angle(270);
//     chassis.drive_distance(9.4, {.timeout = 500}); //align with goal
    
//     assembly.sixbar_left.spinFor(reverse, 0.35, rev, 3.5, volt, false);
//     assembly.sixbar_right.spinFor(fwd, 0.35, rev, 3.5, volt);
//     wait(0.15,seconds);
//     assembly.claw.open();
//     wait(0.1,seconds); //drop stack on
//     chassis.drive_distance(-10);

//     // chassis.drive_distance(-6, {.heading=90});
//     // chassis.turn_to_angle(241);
//     // chassis.drive_distance(29.5,{.heading = 241});
//     // chassis.right_swing_to_angle(180,{.timeout = 450, .min_voltage = 12});
//     // chassis.drive_distance(1.5,{.timeout = 100});
//     // assembly.claw.close();
//     // wait(0.15,seconds);
//     // assembly.sixbar_left.spinFor(fwd, 0.4, rev, 7, volt, false);
//     // assembly.sixbar_right.spinFor(reverse, 0.4, rev, 7, volt, false);
//     // chassis.drive_distance(1,{.heading = 180});
//     // chassis.turn_to_angle(0); 

//     // chassis.drive_distance(15,{.timeout=300});
//     // assembly.sixbar_left.spinFor(reverse, 0.35, rev, 4, volt, false);
//     // assembly.sixbar_right.spinFor(fwd, 0.35, rev, 4, volt);
//     // wait(0.15,seconds);
//     // assembly.claw.open(); // final

//     return "";
// }

// Odometry version, start is (0, 0) facing 0
std::string blue_right_winpointV1(bool calibrate, auto_variation var, bool get_name) {
    if (get_name) return "blue_right_winpointV1";

    if (calibrate) {
        chassis.set_coordinates(0, 0, 0);

        return "";
    }

    assembly.toggle.setVelocity(100,percent);
    assembly.toggle.spinFor(1,rev);
    wait(0.1,seconds);
    assembly.toggle.spinFor(1.25,rev);
    chassis.drive_to_point(0, 7, {.timeout = 800});

    assembly.sixbar_left.spinFor(fwd, 0.25, rev, 6, volt, false);
    assembly.sixbar_right.spinFor(reverse, 0.25, rev, 6, volt, false);

    chassis.right_swing_to_angle(270, {.timeout = 1500}); //swing into cup
    chassis.drive_to_point(-11.25, 13, {.max_voltage = 10, .timeout = 1000});

    assembly.sixbar_left.spinFor(reverse, 0.30, rev, 3.5, volt, false);
    assembly.sixbar_right.spinFor(fwd, 0.30, rev, 3.5, volt);
    wait(0.15,seconds);
    assembly.claw.open();
    wait(0.1,seconds);

    chassis.drive_distance(-7);
    chassis.turn_to_point(-17.5, -0.3, {.timeout = 1000});
    chassis.drive_to_point(-17.5, -0.3, {.max_voltage = 7, .timeout = 2000});
    chassis.right_swing_to_angle(180, {.timeout = 500}); //get to side pin
    chassis.drive_distance(1, {.timeout = 100});
    assembly.claw.close();
    wait(0.3,seconds);

    chassis.drive_distance(-1, {.timeout = 100});
    assembly.sixbar_left.spinFor(fwd, 0.45, rev, 6, volt, false);
    assembly.sixbar_right.spinFor(reverse, 0.45, rev, 6, volt);
    chassis.turn_to_point(-3.5, -2.7, {.timeout = 500});
    chassis.drive_to_point(-3.5, -2.7, {.timeout = 1500});
    chassis.turn_to_point(-13.5, -2.7, {.timeout = 800});
    chassis.drive_to_point(-13.5, -2.7, {.timeout = 500}); //align with goal

    // assembly.sixbar_left.spinFor(reverse, 0.35, rev, 3.5, volt, false);
    // assembly.sixbar_right.spinFor(fwd, 0.35, rev, 3.5, volt);
    // wait(0.3,seconds);
    // assembly.claw.open();
    // wait(0.1,seconds); //drop stack on

    // chassis.drive_to_point(-7.5, -2.7, {.direction = vex::directionType::rev, .timeout = 900});
    // chassis.turn_to_point(10.3, -17, {.timeout = 1200});
    // chassis.drive_to_point(10.3, -17, {.timeout = 2500});
    // chassis.left_swing_to_angle(180, {.timeout = 450, .min_voltage = 12});
    // chassis.drive_distance(1.5, {.timeout = 100});
    // assembly.claw.close();
    // wait(0.3,seconds);
    // assembly.sixbar_left.spinFor(fwd, 0.4, rev, 7, volt, false);
    // assembly.sixbar_right.spinFor(reverse, 0.4, rev, 7, volt, false);
    // chassis.drive_distance(1, {.timeout = 300});
    // chassis.turn_to_angle(0, {.timeout = 1200});

    // chassis.drive_distance(15, {.timeout = 300});
    // assembly.sixbar_left.spinFor(reverse, 0.35, rev, 4, volt, false);
    // assembly.sixbar_right.spinFor(fwd, 0.35, rev, 4, volt);
    // assembly.claw.open(); // final

    return "";
}
std::string blue_right_elim(bool calibrate, auto_variation var, bool get_name) {
    if (get_name) return "blue right elim";

    // Mirror autons here

    if (calibrate) {
        // Run startup sequence that you want in pre-auton, do before field controller plug in
        // chassis.set_coordinates(0, 0, 0);

        return "";
    }

    // Place start of autonoumous here

    return "";
}

std::string red_left_winpoint(bool calibrate, auto_variation var, bool get_name) {
    if (get_name) return "red left winpoint";

    // Mirror autons here

    if (calibrate) {
        // Run startup sequence that you want in pre-auton, do before field controller plug in
        // chassis.set_coordinates(0, 0, 0);

        return "";
    }

    // Place start of autonoumous here

    return "";
}
std::string red_left_sawp(bool calibrate, auto_variation var, bool get_name) {
    if (get_name) return "red left sawp";

    // Mirror autons here

    if (calibrate) {
        // Run startup sequence that you want in pre-auton, do before field controller plug in
        // chassis.set_coordinates(0, 0, 0);

        return "";
    }

    // Place start of autonoumous here

    return "";
}
std::string red_left_elim(bool calibrate, auto_variation var, bool get_name) {
    if (get_name) return "red left elim";

    // Mirror autons here

    if (calibrate) {
        // Run startup sequence that you want in pre-auton, do before field controller plug in
        // chassis.set_coordinates(0, 0, 0);

        return "";
    }

    // Place start of autonoumous here

    return "";
}
std::string red_right_winpoint(bool calibrate, auto_variation var, bool get_name) {
    if (get_name) return "red right winpoint";

    // Mirror autons here

    if (calibrate) {
        // Run startup sequence that you want in pre-auton, do before field controller plug in
        // chassis.set_coordinates(0, 0, 0);

        return "";
    }

    // Place start of autonoumous here

    return "";
}
std::string red_right_sawp(bool calibrate, auto_variation var, bool get_name) {
    if (get_name) return "red right sawp";

    // Mirror autons here

    if (calibrate) {
        // Run startup sequence that you want in pre-auton, do before field controller plug in
        // chassis.set_coordinates(0, 0, 0);

        return "";
    }

    // Place start of autonoumous here

    return "";
}
std::string red_right_elim(bool calibrate, auto_variation var, bool get_name) {
    if (get_name) return "red right elim";

    // Mirror autons here

    if (calibrate) {
        // Run startup sequence that you want in pre-auton, do before field controller plug in
        // chassis.set_coordinates(0, 0, 0);

        return "";
    }

    // Place start of autonoumous here

    return "";
}
// Copy of skills with drivetrain, sixbar and toggle only. No claw or auto-clamp.
std::string skills(bool calibrate, auto_variation var, bool get_name) {
    if (get_name) return "skills";

    if (calibrate) {
        chassis.set_coordinates(0,0,0);
        assembly.claw_grab();
        return "";
    }

    assembly.toggle.setVelocity(100,percent);
    assembly.flip_toggle_to_red();
    assembly.sixbar_left.spinFor(fwd, 0.15, rev, 6, volt, false);
    assembly.sixbar_right.spinFor(reverse, 0.15, rev, 6, volt, false);

    chassis.drive_to_point(0.125, 8.348);
    assembly.sixbar_left.spinFor(fwd, 0.15, rev, 6, volt, false);
    assembly.sixbar_right.spinFor(reverse, 0.15, rev, 6, volt, false);
    chassis.left_swing_to_angle(90, {.timeout = 650}); //swing into cup
    chassis.drive_distance(7.5, {.max_voltage = 12, .timeout = 750});
    chassis.drive_distance(-1.5);
    assembly.sixbar_left.spinFor(reverse, 0.35, rev, 3.5, volt, false);
    assembly.sixbar_right.spinFor(fwd, 0.35, rev, 3.5, volt);
    assembly.claw_release();
    wait(0.1,seconds);
    wait(0.1,seconds);

    chassis.drive_distance(-6,{.heading = 90});
    assembly.arm_auto_clamp();
    chassis.turn_to_point(21, 38.348);
    chassis.drive_distance(18);
    chassis.drive_distance(5,{.max_voltage = 2.5});
    assembly.wait_for_clamp(200);

    assembly.sixbar_left.spinFor(fwd, 0.45, rev, 6, volt, false);
    assembly.sixbar_right.spinFor(reverse, 0.45, rev, 6, volt, false);
    chassis.turn_to_point(19.625, 24.098);
    chassis.drive_distance(15,{.max_voltage = 8, .timeout = 1000});
    chassis.drive_distance(-0.75);

    assembly.sixbar_left.spinFor(reverse, 0.55, rev, 3, volt, false);
    assembly.sixbar_right.spinFor(fwd, 0.55, rev, 3, volt);
    assembly.claw_release();
    wait(0.1,seconds);
    wait(0.1,seconds);

    chassis.drive_distance(-12,{.heading=90});
    assembly.arm_auto_clamp();
    chassis.turn_to_point(32.795, 18.668);
    chassis.drive_to_point(32.795, 18.668);
    chassis.drive_distance(5,{.max_voltage = 4});
    assembly.wait_for_clamp(200);

    assembly.sixbar_left.spinFor(fwd, 0.45, rev, 6, volt, false);
    assembly.sixbar_right.spinFor(reverse, 0.45, rev, 6, volt, false);
    chassis.turn_to_point(20.315, 13.368);
    chassis.drive_distance(15,{.max_voltage = 8, .timeout = 1000});
    assembly.sixbar_left.spinFor(reverse, 0.30, rev, 3.5, volt, false);
    assembly.sixbar_right.spinFor(fwd, 0.30, rev, 3.5, volt);
    assembly.claw_release();
    wait(0.1,seconds);
    wait(0.1,seconds);

    chassis.drive_to_point(50, 13.368, {.direction = reverse});
    assembly.arm_auto_clamp();
    assembly.sixbar_left.spinFor(reverse, 0.30, rev, 3.5, volt, false); // lower the arm for the loader
    assembly.sixbar_right.spinFor(fwd, 0.30, rev, 3.5, volt, false);
    chassis.turn_to_angle(175);
    wait(0.1,seconds);
    chassis.drive_distance(14);
    wait(0.1,seconds);
    assembly.wait_for_clamp(200);

    for(int i = 0; i < 3; i++){ // first loader loop
        chassis.set_coordinates(0, 0, chassis.get_absolute_heading()); // RESET THE COORDS ONCE IN THE LOADER, keep current heading

        chassis.drive_to_point(0,12, {.direction = reverse});
        chassis.turn_to_angle(270);
        assembly.sixbar_left.spinFor(fwd, 0.4+(0.1*i), rev, 6, volt, false); // lift the match load
        assembly.sixbar_right.spinFor(reverse, 0.4+(0.1*i), rev, 6, volt, false);
        chassis.drive_distance(20, {.heading = 270});
        chassis.drive_distance(10, {.max_voltage = 6, .heading = 270, .timeout = 1000});

        assembly.sixbar_left.spinFor(reverse, 0.4+(0.1*i), rev, 6, volt, false); // lower the arm back down
        assembly.sixbar_right.spinFor(fwd, 0.4+(0.1*i), rev, 6, volt);
        assembly.claw_release();
        wait(0.2,seconds);
        chassis.drive_to_point(0,12,{.direction = reverse});
        assembly.arm_auto_clamp();
        chassis.turn_to_angle(180);
        chassis.drive_distance(10);
        chassis.drive_distance(4, {.max_voltage = 6, .timeout = 250, .heading = 180});
        assembly.wait_for_clamp(200);
    }

    // for(int i = 0; i < 3; i++){ // second loader loop
    //     chassis.set_coordinates(0, 0, chassis.get_absolute_heading());
    //     chassis.drive_to_point(0,20, {.direction = reverse});
    //     assembly.sixbar_left.spinFor(fwd, 0.4+(0.1*i), rev, 6, volt, false); // lift the match load
    //     assembly.sixbar_right.spinFor(reverse, 0.4+(0.1*i), rev, 6, volt, false);
    //     chassis.turn_to_angle(310);
    //     chassis.drive_distance(8, {.heading = 310});
    //     chassis.drive_distance(3, {.max_voltage = 4, .heading = 310, .timeout = 500});
        
    //     if (i != 2){
    //         assembly.sixbar_left.spinFor(reverse, 0.4+(0.1*i), rev, 6, volt, false); // lower the arm back down
    //         assembly.sixbar_right.spinFor(fwd, 0.4+(0.1*i), rev, 6, volt);
    //         assembly.claw_release();
    //         wait(0.2,seconds);
    //         chassis.drive_distance(-10, {.max_voltage = 6, .heading = 310, .timeout = 1000});
    //         assembly.arm_auto_clamp();
    //         chassis.turn_to_angle(180);
    //         chassis.drive_distance(18);
    //         chassis.drive_distance(6, {.max_voltage = 6, .timeout = 250, .heading = 180});
    //         assembly.wait_for_clamp(200);
    //     }
    // }

    // chassis.drive_distance(-10, {.max_voltage = 6, .heading = 310, .timeout = 1000});
    // chassis.turn_to_angle(0);
    // chassis.drive_to_point(-10, 60);
    // assembly.toggle.spinFor(3, rev);
    
    // DELETE ALL THIS AFTER GRASSROOTS
    chassis.set_coordinates(0, 0, chassis.get_absolute_heading()); // RESET THE COORDS ONCE IN THE LOADER, keep current heading
    chassis.drive_to_point(0,12, {.direction = reverse});
    chassis.turn_to_angle(270);
    assembly.sixbar_left.spinFor(fwd, 0.4+(0.1*4), rev, 6, volt, false); // lift the match load
    assembly.sixbar_right.spinFor(reverse, 0.4+(0.1*4), rev, 6, volt, false);
    chassis.drive_distance(20, {.heading = 270});
    chassis.drive_distance(10, {.max_voltage = 6, .heading = 270, .timeout = 1000});
    assembly.sixbar_left.spinFor(reverse, 0.4+(0.1*4), rev, 6, volt, false); // lower the arm back down
    assembly.sixbar_right.spinFor(fwd, 0.4+(0.1*4), rev, 6, volt);
    assembly.claw_release();
    wait(0.2,seconds);
    chassis.drive_distance(-10);
    chassis.turn_to_angle(315);
    chassis.drive_distance(65);

    return "";
}

// std::string skills(bool calibrate, auto_variation var, bool get_name) {
//     if (get_name) return "skills";

//     // Mirror autons here

// if (calibrate) {
//         // Run startup sequence that you want in pre-auton, do before field controller plug in
//          chassis.set_coordinates(0,0,0);
//         return "";
//     }

//     assembly.claw_grab(); // hold the preload (claw starts physically open at power-on)
//     assembly.toggle.setVelocity(100,percent);
//     assembly.flip_toggle_to_red();

//     assembly.sixbar_left.spinFor(fwd, 0.10, rev, 6, volt, false);
//     assembly.sixbar_right.spinFor(reverse, 0.10, rev, 6, volt, false);

//     chassis.drive_to_point(0.125, 8.348);
//     assembly.sixbar_left.spinFor(fwd, 0.15, rev, 6, volt, false);
//     assembly.sixbar_right.spinFor(reverse, 0.15, rev, 6, volt, false);
//     chassis.left_swing_to_angle(90, {.timeout = 650}); //swing into cup
//     chassis.drive_distance(7.5, {.max_voltage = 12, .timeout = 750});
//     assembly.sixbar_left.spinFor(reverse, 0.30, rev, 3.5, volt, false);
//     assembly.sixbar_right.spinFor(fwd, 0.30, rev, 3.5, volt);
//     wait(0.1,seconds);
//     assembly.claw_release(); // drop preload on cup. Auto-clamp won't re-grab it: re-arms once we back off 120 mm
//     wait(0.1,seconds);

    
//     chassis.drive_distance(-6,{.heading = 90});
//     chassis.turn_to_point(19.625, 38.348);
//     assembly.arm_auto_clamp();
//     chassis.drive_distance(19);
//     chassis.drive_distance(5,{.max_voltage = 2.5});
//     assembly.wait_for_clamp(300); 
//     // auto-clamp grabs the first freestanding one; forces the grab after 300 ms
//     // drive_to_pin(-27.5, -28.75,250,2.5); // fast at the first freestanding one, slow once the sensor sees it, then auto-clamp
//     // chassis.drive_to_point(-26.5, -25.5, {.max_voltage = 8}); // get to first freestanding one

//     assembly.sixbar_left.spinFor(fwd, 0.45, rev, 6, volt, false);
//     assembly.sixbar_right.spinFor(reverse, 0.45, rev, 6, volt, false);
//     chassis.turn_to_point(19.625, 24.098);
//     chassis.drive_distance(15,{.max_voltage = 8, .timeout = 1000});

//     assembly.sixbar_left.spinFor(reverse, 0.30, rev, 3, volt, false);
//     assembly.sixbar_right.spinFor(fwd, 0.30, rev, 3, volt);
//     wait(0.1,seconds);
//     assembly.claw_release(); // drop first stack on cup. Auto-clamp won't re-grab it: re-arms once we back off 120 mm
//     wait(0.1,seconds);

//     chassis.drive_distance(-12,{.heading=90});
//     chassis.turn_to_point(32.795, 20.668);
//     assembly.arm_auto_clamp(); // turned away from the dropped stack: ready to grab the next one
//     chassis.drive_to_point(32.795, 20.668);
//     chassis.drive_distance(5,{.max_voltage = 4});
//     assembly.wait_for_clamp(300); // auto-clamp grabs the second freestanding one; forces the grab after 300 ms

//     assembly.sixbar_left.spinFor(fwd, 0.45, rev, 6, volt, false);
//     assembly.sixbar_right.spinFor(reverse, 0.45, rev, 6, volt, false);
//     chassis.turn_to_point(20.315, 13.368);
//     chassis.drive_distance(15,{.max_voltage = 8, .timeout = 1000});
//     assembly.sixbar_left.spinFor(reverse, 0.30, rev, 3.5, volt, false);
//     assembly.sixbar_right.spinFor(fwd, 0.30, rev, 3.5, volt);
//     wait(0.1,seconds);
//     assembly.claw_release(); // drop second stack on cup. Auto-clamp won't re-grab it: re-arms once we back off 120 mm
//     wait(0.1,seconds);

//     chassis.drive_to_point(50, 13.368, {.direction = reverse});
//     assembly.sixbar_left.spinFor(reverse, 0.30, rev, 3.5, volt, false); // lower the arm for the loader
//     assembly.sixbar_right.spinFor(fwd, 0.30, rev, 3.5, volt, false);
//     chassis.turn_to_angle(175);
//     wait(0.1,seconds);
//     chassis.drive_distance(13);
//     wait(0.1,seconds)
//     assembly.arm_auto_clamp(); // facing away from the dropped stack: ready to grab the match load
//     // chassis.drive_distance(5,{.max_voltage = 8, .timeout = 500});
//     assembly.wait_for_clamp(300); // auto-clamp grabs the match load; forces the grab after 300 ms
    
//     for(int i = 0; i < 3; i++){ // first loader loop
//         chassis.set_coordinates(0, 0, chassis.get_absolute_heading()); // RESET THE COORDS ONCE IN THE LOADER, keep current heading

//         chassis.drive_to_point(0,12, {.direction = reverse});
//         chassis.turn_to_angle(270);
//         assembly.sixbar_left.spinFor(fwd, 0.4+(0.1*i), rev, 6, volt, false); // lift the match load
//         assembly.sixbar_right.spinFor(reverse, 0.4+(0.1*i), rev, 6, volt, false);
//         chassis.drive_distance(20, {.heading = 90});
//         chassis.drive_distance(10, {.max_voltage = 6, .heading = 90, .timeout = 1000});

//         assembly.sixbar_left.spinFor(reverse, 0.4+(0.1*i), rev, 6, volt, false); // lower the arm back down
//         assembly.sixbar_right.spinFor(fwd, 0.4+(0.1*i), rev, 6, volt);
//         wait(0.2,seconds);
//         assembly.claw_release(); // drop the match load
//         wait(0.2,seconds);  
//         chassis.drive_to_point(0,-12,{.direction = reverse});
//         assembly.arm_auto_clamp();// ready to grab the next match load
//         chassis.turn_to_angle(0);
//         chassis.drive_distance(10);
//         chassis.drive_distance(4, {.max_voltage = 6, .timeout = 250, .heading = 0});
//         assembly.wait_for_clamp(300); // auto-clamp grabs the match load; forces the grab after 300 ms

//     }

//     for(int i = 0; i < 3; i++){ // second loader loop
//         chassis.set_coordinates(0, 0, chassis.get_absolute_heading());

//     }

//     return "";
// }

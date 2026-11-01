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
std::string blue_left_two_goalpin(bool calibrate, auto_variation var, bool get_name) {
    if (get_name) return "su: left on roller touching";

    if (calibrate) {
        chassis.set_coordinates(-66.013, -8.701, 0);

        return "";
    }
    // assembly.toggle.setVelocity(100,percent);
    // assembly.toggle.spinFor(2.25,rev);
    assembly.toggle.setVelocity(100,percent);
    assembly.flip_toggle_to_blue();

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

std::string blue_right_two_goalpin(bool calibrate, auto_variation var, bool get_name) {
    if (get_name) return "su: left on roller touching";

    if (calibrate) {
        chassis.set_coordinates(-66.013, -8.701, 0);

        return "";
    }
    assembly.toggle.setVelocity(100,percent);
    assembly.flip_toggle_to_blue();

    chassis.drive_distance(6.5);

    assembly.sixbar_left.spinFor(fwd, 0.25, rev, 6, volt, false);
    assembly.sixbar_right.spinFor(reverse, 0.25, rev, 6, volt, false);

    chassis.right_swing_to_angle(270); //swing into cup
    chassis.drive_distance(7, {.timeout = 700});

    assembly.sixbar_left.spinFor(reverse, 0.30, rev, 3.5, volt, false);
    assembly.sixbar_right.spinFor(fwd, 0.30, rev, 3.5, volt);
    wait(0.1,seconds);
    assembly.claw.open();
    wait(0.75,seconds);

    chassis.drive_distance(-2);
    wait(0.1,seconds);
    chassis.drive_distance(1.75);
    chassis.drive_distance(-8.5,{.heading=180});
    chassis.turn_to_angle(225);
    chassis.drive_distance(18.75);
    chassis.right_swing_to_angle(180,{.timeout = 500}); //get to side pin
    chassis.drive_distance(1,{.timeout = 100});
    assembly.claw.close();
    wait(0.15,seconds);
    
    assembly.sixbar_left.spinFor(fwd, 0.45, rev, 6, volt, false);
    assembly.sixbar_right.spinFor(reverse, 0.45, rev, 6, volt);
    chassis.turn_to_angle(260,{.timeout = 500});
    chassis.drive_distance(-16,{.heading=180});
    chassis.turn_to_angle(270);
    chassis.drive_distance(9.4, {.timeout = 500}); //align with goal
    
    assembly.sixbar_left.spinFor(reverse, 0.35, rev, 3.5, volt, false);
    assembly.sixbar_right.spinFor(fwd, 0.35, rev, 3.5, volt);
    wait(0.15,seconds);
    assembly.claw.open();
    wait(0.1,seconds); //drop stack on
    chassis.drive_distance(-10);

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




// std::string blue_right_two_goalpin(bool calibrate, auto_variation var, bool get_name) {
//     if (get_name) return "su: right on roller touching";

//     if (calibrate) {
//         chassis.set_coordinates(8.118, 59.615 , 180);

//         return "";
//     }


//     assembly.toggle.setVelocity(100,percent);
//     assembly.flip_toggle_to_blue();


//     wait(0.15,seconds);

//     chassis.drive_distance(2, {.timeout = 600});

//     assembly.sixbar_left.spinFor(fwd, 0.2, rev, 6, volt, false);
//     assembly.sixbar_right.spinFor(reverse, 0.2, rev, 6, volt, false);


//     chassis.turn_to_point(22.365, 49.197, {.timeout = 1200});
//     chassis.drive_to_point(22.365, 49.197, {.max_voltage = 10, .timeout = 2200});


//     assembly.sixbar_left.spinFor(reverse, 0.2, rev, 3.5, volt, false);
//     assembly.sixbar_right.spinFor(fwd, 0.2, rev, 3.5, volt, false);
//     wait(0.3,seconds);
//     assembly.claw.open();
//     wait(0.2,seconds); // score the preload

//     chassis.drive_distance(-12, {.timeout = 1500}); 
//     // go to and clamp the first one
//     // chassis.turn_to_point(26.2, 26.25, {.timeout = 1200});
//     // chassis.drive_to_point(26.2, 26.25, {.max_voltage = 4, .timeout = 4500, .wait = false});
//     // chassis.wait_until_within(0.90);
//     // assembly.claw.close();
//     // wait(0.3,seconds);

//     // assembly.sixbar_left.spinFor(fwd, 0.25, rev, 6, volt, false); //lift arm up while moving
//     // assembly.sixbar_right.spinFor(reverse, 0.25, rev, 6, volt, false);
//     // chassis.turn_to_point(25.966, 40.494, {.timeout = 1000}); //turn to goal
//     // chassis.drive_to_point(25.966, 40.494, {.timeout = 2000});

//     // assembly.sixbar_left.spinFor(reverse, 0.35, rev, 3.5, volt, false);
//     // assembly.sixbar_right.spinFor(fwd, 0.35, rev, 3.5, volt, false);
//     // wait(0.3,seconds);
//     // assembly.claw.open();
//     // wait(0.1,seconds);
//     // chassis.drive_distance(-9, {.timeout = 900});

//     // chassis.turn_to_point(48.1, 43.5, {.timeout = 1000}); // go to last cone
//     // chassis.drive_to_point(48.1, 43.5, {.max_voltage = 4, .timeout = 4500, .wait = false});
//     // chassis.wait_until_within(0.90);
//     // assembly.claw.close();
//     // wait(0.1,seconds);
//     // assembly.sixbar_left.spinFor(fwd, 0.45, rev, 6, volt, false);
//     // assembly.sixbar_right.spinFor(reverse, 0.45, rev, 6, volt, false);
//     // chassis.turn_to_point(28.1, 48.012, {.timeout = 1500});
//     // wait(0.1,seconds);

//     // chassis.drive_distance(12, {.timeout = 750});
//     // assembly.sixbar_left.spinFor(reverse, 0.25, rev, 6, volt, false);
//     // assembly.sixbar_right.spinFor(fwd, 0.25, rev, 6, volt);

//     return "";
// }



// std::string red_right_two_goalpin(bool calibrate, auto_variation var, bool get_name) {
//     if (get_name) return "su: right on roller touching";

//     if (calibrate) {
//         chassis.set_coordinates(8.118, 59.615 , 180);

//         return "";
//     }


    
//     assembly.toggle.setVelocity(100,percent);
//     assembly.flip_toggle_to_red();

//     wait(0.15,seconds);

//     chassis.drive_distance(2, {.timeout = 600});

//     assembly.sixbar_left.spinFor(fwd, 0.2, rev, 6, volt, false);
//     assembly.sixbar_right.spinFor(reverse, 0.2, rev, 6, volt, false);


//     chassis.turn_to_point(22.365, 49.197, {.timeout = 1200});
//     chassis.drive_to_point(22.365, 49.197, {.max_voltage = 10, .timeout = 2200});


//     assembly.sixbar_left.spinFor(reverse, 0.2, rev, 3.5, volt, false);
//     assembly.sixbar_right.spinFor(fwd, 0.2, rev, 3.5, volt, false);
//     wait(0.3,seconds);
//     assembly.claw.open();
//     wait(0.2,seconds); // score the preload

//     chassis.drive_distance(-12, {.timeout = 1500}); // go to and clamp the first one
//     chassis.turn_to_point(26.2, 26.25, {.timeout = 1200});
//     chassis.drive_to_point(26.2, 26.25, {.max_voltage = 4, .timeout = 4500, .wait = false});
//     chassis.wait_until_within(0.90);
//     assembly.claw.close();
//     wait(0.3,seconds);

//     assembly.sixbar_left.spinFor(fwd, 0.25, rev, 6, volt, false); //lift arm up while moving
//     assembly.sixbar_right.spinFor(reverse, 0.25, rev, 6, volt, false);
//     chassis.turn_to_point(25.966, 40.494, {.timeout = 1000}); //turn to goal
//     chassis.drive_to_point(25.966, 40.494, {.timeout = 2000});

//     assembly.sixbar_left.spinFor(reverse, 0.35, rev, 3.5, volt, false);
//     assembly.sixbar_right.spinFor(fwd, 0.35, rev, 3.5, volt, false);
//     wait(0.3,seconds);
//     assembly.claw.open();
//     wait(0.1,seconds);
//     chassis.drive_distance(-9, {.timeout = 900});

//     chassis.turn_to_point(48.1, 43.5, {.timeout = 1000}); // go to last cone
//     chassis.drive_to_point(48.1, 43.5, {.max_voltage = 4, .timeout = 4500, .wait = false});
//     chassis.wait_until_within(0.90);
//     assembly.claw.close();
//     wait(0.1,seconds);
//     assembly.sixbar_left.spinFor(fwd, 0.45, rev, 6, volt, false);
//     assembly.sixbar_right.spinFor(reverse, 0.45, rev, 6, volt, false);
//     chassis.turn_to_point(28.1, 48.012, {.timeout = 1500});
//     wait(0.1,seconds);

//     chassis.drive_distance(12, {.timeout = 750});
//     assembly.sixbar_left.spinFor(reverse, 0.25, rev, 6, volt, false);
//     assembly.sixbar_right.spinFor(fwd, 0.25, rev, 6, volt);

//     return "";
// }

std::string red_right_two_goalpin(bool calibrate, auto_variation var, bool get_name) {
    if (get_name) return "su: left on roller touching";

    if (calibrate) {
        chassis.set_coordinates(-66.013, -8.701, 0);

        return "";
    }
    // assembly.toggle.setVelocity(100,percent);
    // assembly.toggle.spinFor(2.25,rev);
    assembly.toggle.setVelocity(100,percent);
    assembly.flip_toggle_to_red();

    chassis.drive_distance(6.5);

    assembly.sixbar_left.spinFor(fwd, 0.25, rev, 6, volt, false);
    assembly.sixbar_right.spinFor(reverse, 0.25, rev, 6, volt, false);

    chassis.right_swing_to_angle(270); //swing into cup
    chassis.drive_distance(7, {.timeout = 700});

    assembly.sixbar_left.spinFor(reverse, 0.30, rev, 3.5, volt, false);
    assembly.sixbar_right.spinFor(fwd, 0.30, rev, 3.5, volt);
    wait(0.1,seconds);
    assembly.claw.open();
    wait(0.75,seconds);

    chassis.drive_distance(-2);
    wait(0.1,seconds);
    chassis.drive_distance(1.75);
    chassis.drive_distance(-8.5,{.heading=180});
    chassis.turn_to_angle(225);
    chassis.drive_distance(18.75);
    chassis.right_swing_to_angle(180,{.timeout = 500}); //get to side pin
    chassis.drive_distance(1,{.timeout = 100});
    assembly.claw.close();
    wait(0.15,seconds);
    
    assembly.sixbar_left.spinFor(fwd, 0.45, rev, 6, volt, false);
    assembly.sixbar_right.spinFor(reverse, 0.45, rev, 6, volt);
    chassis.turn_to_angle(260,{.timeout = 500});
    chassis.drive_distance(-16,{.heading=180});
    chassis.turn_to_angle(270);
    chassis.drive_distance(9.4, {.timeout = 500}); //align with goal
    
    assembly.sixbar_left.spinFor(reverse, 0.35, rev, 3.5, volt, false);
    assembly.sixbar_right.spinFor(fwd, 0.35, rev, 3.5, volt);
    wait(0.15,seconds);
    assembly.claw.open();
    wait(0.1,seconds); //drop stack on
    chassis.drive_distance(-10, {.heading=270});

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
std::string skills(bool calibrate, auto_variation var, bool get_name) {
    if (get_name) return "skills";

    // Mirror autons here

    if (calibrate) {
        // Run startup sequence that you want in pre-auton, do before field controller plug in
         chassis.set_coordinates(-66.013, -8.701, 0);
        return "";
    }

    // Place start of autonoumous here
    assembly.toggle.setVelocity(100,percent);
    assembly.flip_toggle_to_red();

    wait(0.15,seconds);

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
    wait(0.3,seconds);

    chassis.drive_distance(-10);
    wait(0.1,seconds);
    chassis.turn_to_angle(0);
    wait(0.1,seconds);
    chassis.drive_distance(35);


    return "";
}

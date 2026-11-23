#pragma once

#include "vex.h"

using namespace vex;

class Assembly {
public:


    // Cascade Motors
    // static mik::motor cascade_motorL1;
    // static mik::motor cascade_motorL2;
    // static mik::motor cascade_motorR1;
    // static mik::motor cascade_motorR2;
    // static mik::motor_group cascade_left;
    // static mik::motor_group cascade_right;

    // 6B Motors
    static mik::motor sixbar_motorL1;
    static mik::motor sixbar_motorR1;
    static mik::motor sixbar_motorL2;
    static mik::motor sixbar_motorR2;

    static mik::motor_group sixbar_left;
    static mik::motor_group sixbar_right;

    static mik::motor toggle;
    static mik::motor claw;

    // Cascade Rotation Sensors
    static vex::rotation rotation_sensorR;
    static vex::rotation rotation_sensorL;
    static vex::rotation rotation_sensorM;


    static vex::optical optical_sensor;
    static vex::limit limit_switch;

    // Old piston claw, replaced by the claw motor above
    // static mik::piston claw;

    // Distance sensor on the clamp (auto-clamp disabled)
    // static vex::distance clamp_distance;

    // Claw is a motor that always spins: forward = open, reverse = closed.
    // These just change its direction and return right away.
    /** @brief Closes (clamps) the claw. */
    static void claw_grab();
    /** @brief Opens (releases) the claw. */
    static void claw_release();
    /** @brief Opens the claw if it's closed, closes it if it's open. */
    static void claw_toggle();
    /** @returns True if the claw was last commanded open. */
    static bool claw_is_open();
    /** @brief Starts the claw spinning in its current direction (driver control calls this every loop). */
    static void claw_keep_spinning();

//     /**
//      * @brief For autons: waits for auto-clamp to grab, up to timeout_ms.
//      * If nothing triggered it by then, grabs anyway so the auton never misses.
//      * @returns True if auto-clamp grabbed on its own, false if it had to force the grab.
//      */
//     static bool wait_for_clamp(int timeout_ms);

//     /**
//      * @brief For autons: re-arms auto-clamp right now, without waiting for the sensor to see clear.
//      * Call after backing off and turning away from a goal you just dropped.
//      */
//     static void arm_auto_clamp();

//     /** @brief Starts auto-clamp in the background: closes the claw when something reaches the clamp. Used in autons. */
//     void start_auto_clamp();
//     /** @brief Stops auto-clamp. Called when driver control starts. */
//     void stop_auto_clamp();

    // AI Vision: looks for the toggle's color. Colors are tuned in the AI Vision Utility.
    static vex::aivision::colordesc red_toggle;
    static vex::aivision::colordesc blue_toggle;
    static vex::aivision::colordesc yellow_toggle;
    static vex::aivision ai_vision;

    /** @brief For autons: flips the toggle until the camera sees red. */
    void flip_toggle_to_red();
    /** @brief For autons: flips the toggle until the camera sees blue. */
    void flip_toggle_to_blue();
    
    void init();
    void control();

    // void lower_intake_control();
    // void upper_intake_control();
    // void wing_piston_control();
    // void scraper_piston_control();
    // void cascadePre();
    // void cascadeControl();
    void clawControl();
    void sixBarControl();
    void toggleControl();
    
};
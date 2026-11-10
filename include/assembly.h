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

    // Cascade Rotation Sensors
    static vex::rotation rotation_sensorR;
    static vex::rotation rotation_sensorL;
    static vex::rotation rotation_sensorM;


    static vex::optical optical_sensor;
    static vex::limit limit_switch;

    static mik::piston claw;

    // Distance sensor on the clamp
    static vex::distance clamp_distance;

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
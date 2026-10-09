#pragma once

#include "vex.h"

using namespace vex;

class Assembly {
public:


    // Cascade Motors
    static mik::motor cascade_motorL1;
    static mik::motor cascade_motorL2;
    static mik::motor cascade_motorR1;
    static mik::motor cascade_motorR2;
    static mik::motor_group cascade_left;
    static mik::motor_group cascade_right;

    // Cascade Rotation Sensors
    static vex::rotation rotation_sensorR;
    static vex::rotation rotation_sensorL;
    static vex::rotation rotation_sensorM;


    static vex::optical optical_sensor;
    static vex::limit limit_switch;

    static mik::piston claw;
    
    void init();
    void control();

    // void lower_intake_control();
    // void upper_intake_control();
    // void wing_piston_control();
    // void scraper_piston_control();
    // void cascadePre();
    void cascadeControl();
    void clawControl();
    
};
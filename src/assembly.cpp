#include "vex.h"

// The code in this file is example and can be deleted

// This is example code for a push back robot with two 5.5W motors on the lower intake,
// 11W motor on the top intake, a scraper, and wing

// This function is called once at the start of user control
void Assembly::init() {
    // You can declare a task that you want to always run in user control.

    // For example a task that is always checking if the intake is moving forward while being told to,
    // and detected not, then the intake will move in reverse to unjam itself
    sixbar_left.setStopping(vex::brakeType::hold);
    sixbar_right.setStopping(vex::brakeType::hold);
} 

// You want to put this function inside the user control loop in main.
void Assembly::control() {
    static bool initialized = false;

    if (!initialized) {
        init();
        initialized = true;
    }

    // lower_intake_control();
    // upper_intake_control();
    // wing_piston_control();
    // scraper_piston_control();
    // cascadeControl();
    clawControl();
    sixBarControl();
    toggleControl();
}


// // Spins intake forward if L1 is being held, reverse if L2 is being held; stops otherwise
// void Assembly::lower_intake_control() {
//     if (Controller.ButtonL1.pressing()) {
//         lower_intake_motors.spin(fwd, 12, volt);
//     } else if (Controller.ButtonL2.pressing()) {
//         lower_intake_motors.spin(fwd, -12, volt);
//     } else {
//         lower_intake_motors.stop();
//     }
// }

// // Spins intake forward if R2 is being held, reverse if Button Down is being held; stops otherwise
// void Assembly::upper_intake_control() {
//     if (Controller.ButtonR2.pressing()) {
//         upper_intake_motor.spin(fwd, 12, volt);
//     } else if (Controller.ButtonDown.pressing()) {
//         upper_intake_motor.spin(fwd, -12, volt);
//     } else {
//         upper_intake_motor.stop();
//     }
// }

// // Extends piston when button R1 is pressed, releases otherwise 
// void Assembly::wing_piston_control() {
//     if (Controller.ButtonR1.pressing()) {
//         wing_piston.open();
//     } else {
//         wing_piston.close();
//     }
// }

// // Extends or retracts piston when button A is pressed, 
// // can only extend or retract again until button A is released and pressed again
// void Assembly::scraper_piston_control() {
//     if (btnA_new_press(Controller.ButtonA.pressing())) {
//         scraper_piston.toggle();
//     }
// }

// void Assembly::cascadeControl(){
//     if (Controller.ButtonR1.pressing()) {
//         cascade_left.spin(fwd, 8, volt);
//         cascade_right.spin(reverse, 8, volt);
//     } 
//     else if (Controller.ButtonR2.pressing()) {
//         cascade_left.spin(reverse, 8, volt);
//         cascade_right.spin(fwd, 8 , volt);
//     } 
//     else if (Controller.ButtonL1.pressing()) {
//         cascade_left.spin(reverse, 8, volt);
//         cascade_right.spin(reverse, 8, volt);
//     } 
//     else if (Controller.ButtonL2.pressing()) {
//         cascade_left.spin(fwd, 8, volt);
//         cascade_right.spin(fwd, 8, volt);
//     } 
//     else { 
//         cascade_left.stop();
//         cascade_right.stop();
//     }
// }

// Up opens the claw, Down closes it, B flips whichever state it is in.
// Each acts on the press itself, and the solenoid latches, so the claw stays
// where it was last commanded instead of needing a button held down.
void Assembly::clawControl() {
    // Local edge detectors: the btn*_new_press macros share one static per
    // button across the whole program, and the UI task already uses them.
    // Start as "already held" so a button still held when driver control begins
    // (e.g. the X press that closes the auton selector) is ignored until released.
    static bool prev_up = true, prev_down = true, prev_toggle = true;

    bool up = Controller.ButtonUp.pressing();
    bool down = Controller.ButtonDown.pressing();
    bool toggle = Controller.ButtonX.pressing();

    if (up && !prev_up) {
        claw.open();
    } else if (down && !prev_down) {
        claw.close();
    } else if (toggle && !prev_toggle) {
        claw.toggle();
    }

    prev_up = up;
    prev_down = down;
    prev_toggle = toggle;
}

void Assembly::sixBarControl(){
    if (Controller.ButtonR1.pressing()) {
        sixbar_left.spin(fwd, 12, volt);
        sixbar_right.spin(reverse, 12, volt);
    } 
    else if (Controller.ButtonR2.pressing()) {
        sixbar_left.spin(reverse, 3, volt);
        sixbar_right.spin(fwd, 3, volt);
    } 
    else{
        sixbar_left.stop();
        sixbar_right.stop();
    }
}

void Assembly::toggleControl(){
    if (Controller.ButtonL1.pressing()) {
        toggle.spin(fwd, 12, volt);
    } 
    else if (Controller.ButtonL2.pressing()) {
        toggle.spin(reverse, 12, volt);
    } 
    else{
        toggle.stop();
    }
}

// Smallest bounding box (pixels^2) that counts as the toggle, so stray specks of color are ignored.
static const int toggle_min_area = 35000;

// Max time (ms) a toggle flip may spin before giving up and moving on with the auton.
static const int toggle_flip_timeout_ms = 1500;

// For autons. Spins the toggle until the camera sees red, then stops and holds.
// Yellow, blue, or nothing in view (mid-flip) all keep it spinning.
void Assembly::flip_toggle_to_red() {
    vex::timer flip_timer;  // gives up after toggle_flip_timeout_ms so a missing/mistuned camera can't stall the auton
    while (!(ai_vision.takeSnapshot(red_toggle, 6) > 0 && ai_vision.objects[0].width * ai_vision.objects[0].height >= toggle_min_area)) {
        if (flip_timer.time(msec) >= toggle_flip_timeout_ms) break;
        toggle.spin(fwd, 12, volt);
        vex::task::sleep(20);
    }
    toggle.stop();
}

// For autons. Spins the toggle until the camera sees blue, then stops and holds.
// Yellow, red, or nothing in view (mid-flip) all keep it spinning.
// void Assembly::flip_toggle_to_blue() {
//     while (!(ai_vision.takeSnapshot(blue_toggle, 6) > 0 && ai_vision.objects[0].width * ai_vision.objects[0].height >= toggle_min_area)) {
//         toggle.spin(fwd, 12, volt);
//         claw.open();
//         vex::task::sleep(20);
//     }
//     toggle.stop();
// }

void Assembly::flip_toggle_to_blue() {
    vex::timer flip_timer;  // gives up after toggle_flip_timeout_ms so a missing/mistuned camera can't stall the auton
    while (!(ai_vision.takeSnapshot(blue_toggle, 6) > 0 && ai_vision.objects[0].width * ai_vision.objects[0].height >= toggle_min_area)) {
        if (flip_timer.time(msec) >= toggle_flip_timeout_ms) break;
        toggle.spin(fwd, 12, volt);
        vex::task::sleep(20);
    }
    toggle.stop();
}


// ---------------- Auto-clamp ----------------
// Closes the claw when the clamp distance sensor sees something right at the clamp.
// Runs as a background task during autons, alongside the normal claw.open()/claw.close() calls.
//
// ARMED    -> object within clamp_trigger_mm for clamp_trigger_ms while claw is open -> close claw, DISARM
// DISARMED -> nothing within clamp_rearm_mm for clamp_rearm_ms -> ARM again
// Also disarms whenever the claw is closed on something (e.g. a preload or an auton claw.close()),
// so opening the claw to drop that object never re-grabs it.
// Auto-clamp only ever closes the claw, never opens it.

static const double clamp_trigger_mm = 20;    // clamp when object is this close (mm)
static const double clamp_rearm_mm   = 120;   // re-arm once object is farther than this (mm)
static const int    clamp_trigger_ms = 60;    // object must stay close this long before clamping
static const int    clamp_rearm_ms   = 300;   // must stay clear this long before re-arming
static const int    clamp_loop_ms    = 10;

// claw.open() physically closes the claw (piston is plumbed backwards), so these swap them.
void Assembly::claw_grab()    { claw.open(); }
void Assembly::claw_release() { claw.close(); }
bool Assembly::claw_is_open() { return !claw.state(); }

static vex::task auto_clamp_task;
static bool auto_clamp_running = false;

static int auto_clamp_loop() {
    bool armed = true;
    int close_time = 0;
    int clear_time = 0;

    while (true) {
        bool detected = Assembly::clamp_distance.isObjectDetected();
        double dist = Assembly::clamp_distance.objectDistance(vex::distanceUnits::mm);
        bool object_close = detected && dist <= clamp_trigger_mm;
        bool object_clear = !detected || dist > clamp_rearm_mm;
        bool claw_open = Assembly::claw_is_open();

        if (!claw_open && !object_clear) {
            // Claw is holding something: don't re-grab it when it gets opened
            armed = false;
        }

        if (armed) {
            if (object_close && claw_open) {
                close_time += clamp_loop_ms;
                if (close_time >= clamp_trigger_ms) {
                    Assembly::claw_grab();
                    Controller.rumble(".");  // feedback so you can tell auto-clamp fired
                    armed = false;
                    close_time = 0;
                    clear_time = 0;
                }
            } else {
                close_time = 0;
            }
        } else {
            if (object_clear) {
                clear_time += clamp_loop_ms;
                if (clear_time >= clamp_rearm_ms) {
                    armed = true;
                    clear_time = 0;
                }
            } else {
                clear_time = 0;
            }
        }

        vex::task::sleep(clamp_loop_ms);
    }
    return 0;
}

void Assembly::start_auto_clamp() {
    if (auto_clamp_running) auto_clamp_task.stop();
    auto_clamp_task = vex::task(auto_clamp_loop);
    auto_clamp_running = true;
}

void Assembly::stop_auto_clamp() {
    if (auto_clamp_running) auto_clamp_task.stop();
    auto_clamp_running = false;
}

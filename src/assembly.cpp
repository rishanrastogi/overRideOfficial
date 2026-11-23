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

// X flips the claw: open (spins forward nonstop) <-> closed (spins reverse for 1.5 s, then stops).
void Assembly::clawControl() {
    // Local edge detector: the btn*_new_press macros share one static per
    // button across the whole program, and the UI task already uses them.
    // Start as "already held" so a button still held when driver control begins
    // (e.g. the X press that closes the auton selector) is ignored until released.
    static bool prev_x = true;

    bool x = Controller.ButtonX.pressing();
    if (x && !prev_x) {
        claw_toggle();
    }
    prev_x = x;

    // Re-apply every loop, so the claw keeps going even if something stopped all motors
    // (doesn't restart the reverse spin once its 1.5 s is up)
    claw_keep_spinning();
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


// ---------------- Claw motor ----------------
// Open: spins forward and never stops.
// Closed: spins reverse for claw_close_ms, then stops (holding) so it isn't stalled the whole match.
// grab/release/X change the direction; a new command restarts the close timer.
static const double claw_volts    = 12;    // voltage the claw spins at
static const int    claw_close_ms = 1500;  // how long reverse (close) spins before stopping

static bool claw_open_state = true;   // claw starts physically open at power-on
static bool claw_close_done = false;  // true once reverse has run its claw_close_ms and stopped
static int  claw_move_id = 0;         // bumps on every command, so an old timer can't stop a newer move

static void claw_spin() {
    if (claw_open_state) {
        Assembly::claw.spin(fwd, claw_volts, volt);
    } else if (!claw_close_done) {
        Assembly::claw.spin(reverse, claw_volts, volt);
    }
}

// Background timer: stops the reverse spin after claw_close_ms, unless a newer command replaced it.
static int claw_stop_after_close(void* arg) {
    int my_id = (int)(intptr_t)arg;
    vex::task::sleep(claw_close_ms);
    if (my_id == claw_move_id) {
        claw_close_done = true;
        Assembly::claw.stop(vex::brakeType::hold);
    }
    return 0;
}

static void claw_move(bool open) {
    claw_open_state = open;
    claw_close_done = false;
    claw_move_id++;
    claw_spin();
    if (!open) {
        vex::task(claw_stop_after_close, (void*)(intptr_t)claw_move_id);
    }
}

void Assembly::claw_grab()    { claw_move(true); }   // spins forward nonstop
void Assembly::claw_release() { claw_move(false); }  // spins reverse for claw_close_ms, then stops
void Assembly::claw_toggle()  { claw_move(!claw_open_state); }
bool Assembly::claw_is_open() { return claw_open_state; }
void Assembly::claw_keep_spinning() { claw_spin(); }


// Auto-clamp (clamp distance sensor) is disabled: commented out below.
// // ---------------- Auto-clamp ----------------
// // Closes the claw when the clamp distance sensor sees something right at the clamp.
// // Runs as a background task during autons, alongside the normal claw_grab()/claw_release() calls.
// //
// // ARMED    -> object within clamp_trigger_mm for clamp_trigger_ms while claw is open -> close claw, DISARM
// // DISARMED -> nothing within clamp_rearm_mm for clamp_rearm_ms -> ARM again
// // Also disarms whenever the claw is closed on something (e.g. a preload or an auton claw_grab()),
// // so opening the claw to drop that object never re-grabs it.
// // Auto-clamp only ever closes the claw, never opens it.

// static const double clamp_trigger_mm = 50;    // clamp when object is this close (mm)
// static const double clamp_rearm_mm   = 120;   // re-arm once object is farther than this (mm)
// static const int    clamp_trigger_ms = 20;    // object must stay close this long before clamping
// static const int    clamp_rearm_ms   = 300;   // must stay clear this long before re-arming
// static const int    clamp_loop_ms    = 10;


// bool Assembly::wait_for_clamp(int timeout_ms) {
//     vex::timer t;
//     while (claw_is_open() && t.time(msec) < timeout_ms) {
//         vex::task::sleep(10);
//     }
//     if (claw_is_open()) {
//         claw_grab();  // sensor didn't see it in time: grab anyway
//         return false;
//     }
//     return true;
// }

// static vex::task auto_clamp_task;
// static bool auto_clamp_running = false;
// static volatile bool auto_clamp_arm_request = false;  // set by arm_auto_clamp(), read by the loop

// void Assembly::arm_auto_clamp() { auto_clamp_arm_request = true; }

// static int auto_clamp_loop() {
//     auto_clamp_arm_request = false;  // ignore requests left over from a previous run
//     bool armed = true;
//     int close_time = 0;
//     int clear_time = 0;

//     while (true) {
//         bool detected = Assembly::clamp_distance.isObjectDetected();
//         double dist = Assembly::clamp_distance.objectDistance(vex::distanceUnits::mm);
//         bool object_close = detected && dist <= clamp_trigger_mm;
//         bool object_clear = !detected || dist > clamp_rearm_mm;
//         bool claw_open = Assembly::claw_is_open();

//         if (auto_clamp_arm_request) {
//             // Auton says we've left the last goal: arm now instead of waiting to see clear
//             auto_clamp_arm_request = false;
//             armed = true;
//             close_time = 0;
//             clear_time = 0;
//         }

//         if (!claw_open && !object_clear) {
//             // Claw is holding something: don't re-grab it when it gets opened
//             armed = false;
//         }

//         if (armed) {
//             if (object_close && claw_open) {
//                 close_time += clamp_loop_ms;
//                 if (close_time >= clamp_trigger_ms) {
//                     Assembly::claw_grab();
//                     Controller.rumble(".");  // feedback so you can tell auto-clamp fired
//                     armed = false;
//                     close_time = 0;
//                     clear_time = 0;
//                 }
//             } else {
//                 close_time = 0;
//             }
//         } else {
//             if (object_clear) {
//                 clear_time += clamp_loop_ms;
//                 if (clear_time >= clamp_rearm_ms) {
//                     armed = true;
//                     clear_time = 0;
//                 }
//             } else {
//                 clear_time = 0;
//             }
//         }

//         vex::task::sleep(clamp_loop_ms);
//     }
//     return 0;
// }

// void Assembly::start_auto_clamp() {
//     if (auto_clamp_running) auto_clamp_task.stop();
//     auto_clamp_task = vex::task(auto_clamp_loop);
//     auto_clamp_running = true;
// }

// void Assembly::stop_auto_clamp() {
//     if (auto_clamp_running) auto_clamp_task.stop();
//     auto_clamp_running = false;
// }

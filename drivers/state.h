// // ===============================================
// //  Module: Robot State Machine
// //  Description: Manages robot states for Demo 2
// //               Line Following + Barcode Navigation
// // ===============================================
// #ifndef STATE_MACHINE_H
// #define STATE_MACHINE_H

// #include "pico/stdlib.h"

// // Robot states
// typedef enum {
//     STATE_IDLE = 0,
//     STATE_LINE_FOLLOWING,
//     STATE_BARCODE_DETECTED,
//     STATE_TURNING_LEFT,
//     STATE_TURNING_RIGHT,
//     STATE_STOPPED,
//     STATE_ERROR
// } robot_state_t;

// // Events that trigger state transitions
// typedef enum {
//     EVENT_START,
//     EVENT_LINE_LOST,
//     EVENT_LINE_FOUND,
//     EVENT_BARCODE_LEFT,
//     EVENT_BARCODE_RIGHT,
//     EVENT_BARCODE_STOP,
//     EVENT_BARCODE_FORWARD,
//     EVENT_TURN_COMPLETE,
//     EVENT_STOP_REQUESTED,
//     EVENT_ERROR
// } robot_event_t;

// // State machine context (telemetry data)
// typedef struct {
//     robot_state_t current_state;
//     robot_state_t previous_state;
//     float current_speed_left;
//     float current_speed_right;
//     float distance_traveled_m;
//     float line_error;
//     bool line_on_track;
// } robot_context_t;

// // ---- Initialization ----
// void state_machine_init(void);

// // ---- Process event (triggers state transitions) ----
// void state_machine_process_event(robot_event_t event);

// // ---- Get current state ----
// robot_state_t state_machine_get_state(void);

// // ---- Get context (for telemetry) ----
// const robot_context_t* state_machine_get_context(void);

// // ---- Update context data (called by main loop) ----
// void state_machine_update_context(float speed_left, float speed_right, 
//                                    float distance_m, float line_err, bool on_track);

// // ---- State name (for logging) ----
// const char* state_machine_state_name(robot_state_t state);
// const char* state_machine_event_name(robot_event_t event);

// #endif
// // ===============================================
// //  Module: Robot State Machine
// //  Description: Manages robot states for Demo 2
// // ===============================================
// #include "state.h"
// #include <stdio.h>
// #include <string.h>

// static robot_context_t context;

// // State transition table
// static const char* STATE_NAMES[] = {
//     "IDLE",
//     "LINE_FOLLOWING",
//     "BARCODE_DETECTED",
//     "TURNING_LEFT",
//     "TURNING_RIGHT",
//     "STOPPED",
//     "ERROR"
// };

// static const char* EVENT_NAMES[] = {
//     "START",
//     "LINE_LOST",
//     "LINE_FOUND",
//     "BARCODE_LEFT",
//     "BARCODE_RIGHT",
//     "BARCODE_STOP",
//     "BARCODE_FORWARD",
//     "TURN_COMPLETE",
//     "STOP_REQUESTED",
//     "ERROR"
// };

// void state_machine_init(void) {
//     memset(&context, 0, sizeof(context));
//     context.current_state = STATE_IDLE;
//     context.previous_state = STATE_IDLE;
    
//     printf("[STATE_MACHINE] Initialized in IDLE state\n");
// }

// static void enter_state(robot_state_t new_state) {
//     context.previous_state = context.current_state;
//     context.current_state = new_state;
    
//     printf("[STATE_MACHINE] %s -> %s\n", 
//            STATE_NAMES[context.previous_state],
//            STATE_NAMES[context.current_state]);
// }

// void state_machine_process_event(robot_event_t event) {
//     robot_state_t current = context.current_state;
    
//     printf("[STATE_MACHINE] Event: %s (in state %s)\n", 
//            EVENT_NAMES[event], STATE_NAMES[current]);
    
//     switch (current) {
//         case STATE_IDLE:
//             if (event == EVENT_START) {
//                 enter_state(STATE_LINE_FOLLOWING);
//             }
//             break;
            
//         case STATE_LINE_FOLLOWING:
//             switch (event) {
//                 case EVENT_BARCODE_LEFT:
//                     enter_state(STATE_TURNING_LEFT);
//                     break;
//                 case EVENT_BARCODE_RIGHT:
//                     enter_state(STATE_TURNING_RIGHT);
//                     break;
//                 case EVENT_BARCODE_STOP:
//                     enter_state(STATE_STOPPED);
//                     break;
//                 case EVENT_BARCODE_FORWARD:
//                     // Continue line following
//                     break;
//                 case EVENT_LINE_LOST:
//                     // Could enter error recovery state
//                     printf("[STATE_MACHINE] WARNING: Line lost!\n");
//                     break;
//                 case EVENT_STOP_REQUESTED:
//                     enter_state(STATE_STOPPED);
//                     break;
//                 default:
//                     break;
//             }
//             break;
            
//         case STATE_TURNING_LEFT:
//         case STATE_TURNING_RIGHT:
//             if (event == EVENT_TURN_COMPLETE) {
//                 enter_state(STATE_LINE_FOLLOWING);
//             } else if (event == EVENT_ERROR) {
//                 enter_state(STATE_ERROR);
//             }
//             break;
            
//         case STATE_STOPPED:
//             if (event == EVENT_START) {
//                 enter_state(STATE_LINE_FOLLOWING);
//             }
//             break;
            
//         case STATE_ERROR:
//             if (event == EVENT_START) {
//                 enter_state(STATE_IDLE);
//             }
//             break;
            
//         default:
//             break;
//     }
// }

// robot_state_t state_machine_get_state(void) {
//     return context.current_state;
// }

// const robot_context_t* state_machine_get_context(void) {
//     return &context;
// }

// void state_machine_update_context(float speed_left, float speed_right,
//                                    float distance_m, float line_err, bool on_track) {
//     context.current_speed_left = speed_left;
//     context.current_speed_right = speed_right;
//     context.distance_traveled_m = distance_m;
//     context.line_error = line_err;
//     context.line_on_track = on_track;
// }

// const char* state_machine_state_name(robot_state_t state) {
//     if (state >= 0 && state < sizeof(STATE_NAMES)/sizeof(STATE_NAMES[0])) {
//         return STATE_NAMES[state];
//     }
//     return "UNKNOWN";
// }

// const char* state_machine_event_name(robot_event_t event) {
//     if (event >= 0 && event < sizeof(EVENT_NAMES)/sizeof(EVENT_NAMES[0])) {
//         return EVENT_NAMES[event];
//     }
//     return "UNKNOWN";
// }
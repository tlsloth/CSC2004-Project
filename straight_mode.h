#ifndef STRAIGHT_MODE_H
#define STRAIGHT_MODE_H

/**
 * @brief Runs the robot in "straight mode".
 *
 * This function contains an infinite loop that continuously drives the robot
 * straight at a predefined BASE_SPEED. It also handles periodic telemetry
 * reporting to the console.
 *
 * It is intended to be a blocking, top-level task for testing or simple
 * straight-line runs.
 */
void drive_straight_task(float target_heading);

#endif // STRAIGHT_MODE_H

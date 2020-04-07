#ifndef __CONTROLLER_H__
#define __CONTROLLER_H__

// Include files.
#include "vehicle.h"

// Forward declare the vehicle type so it can be visible during compilation.
// You can comment this out if you don't need it based on how you've included files / declared structures.
struct t_vehicle;

// Define the control structure.
typedef struct
{
    double speed;
    double angular_velocity; // rad/s [NOTE: would be better to name this variable angular_velocity_rad_s, but the older name has already been released]
} control;

/// YOUR CODE HERE
// Declare possible controller functions here.
// Each one should return a control structure given a vehicle;
// it should compute the desired speed and angular velocity of the vehicle for the next timestep.
control get_proportional_waypoint_control(struct t_vehicle* vehicle);

#endif // __CONTROLLER_H__

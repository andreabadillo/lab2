#ifndef __VEHICLE_H__
#define __VEHICLE_H__

// Include files.
#include "controller.h"

// Define the vehicle structure.
typedef struct t_vehicle
{
    // Vehicle state.
    double position[3]; // x, y, theta
    double velocity[3]; // dx, dy, dtheta
    int num_waypoints;
    double** target_waypoints; // List of x, y points to drive to; target_waypoints[waypoint_index][0] is the x position of the waypoint_index waypoint
    double* current_waypoint;  // The current x, y point you're aiming for.
    int current_waypoint_idx;  // The index of the current waypoint you're aiming for

    // Vehicle function calls
    void (*set_position)   (struct t_vehicle* v, double* values); // sets the vehicle position (after applying constraints)
    void (*set_velocity)   (struct t_vehicle* v, double* values); // sets the vehicle velocities (after applying constraints)
    void (*control_vehicle)(struct t_vehicle* v);  // calculates and applies new vehicle velocities
    void (*update_state)   (struct t_vehicle* v, double time); // updates vehicle states based on current velocity and position (i.e. the vehicle dynamics)
} vehicle;

// Declare vehicle creation function.
vehicle * create_vehicle(double* starting_position, int num_waypoints, double** offset_waypoints);

// Declare standard vehicle methods (pointers to which are in the vehicle structure).
void set_position   (struct t_vehicle* v, double* values);
void set_velocity   (struct t_vehicle* v, double* values);
void control_vehicle(struct t_vehicle* v);
void update_state   (struct t_vehicle* v, double time);

/// YOUR CODE HERE
// Declare any additional functions that you may want.


#endif // __VEHICLE_H__

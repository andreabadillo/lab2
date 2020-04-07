#ifndef __SIMULATOR_H__
#define __SIMULATOR_H__

// Include files.
#include "controller.h"
#include "vehicle.h"

// Configure some of the display/printing output options.
#define USE_DISPLAY 1     // Set to 0 to disable the graphical display.
#define RUN_IN_REALTIME 1 // Set to 0 to step through simulation time as fast as possible.
                          // This setting will be ignored if the graphical display is used.
#define PRINT_SIMULATION_TIME 1 // Whether to print the current time (which will overwrite text on the current terminal line)

// Define the simulator structure.
typedef struct t_simulator
{
    int n_vehicles;
    vehicle* vehicles;
    double current_time;
    double max_time;
    double time_increment;
    double vehicle_update_rate; // Hz NOTE: probably better to name this vehicle_update_rate_hz
    int num_waypoints;
    int radius;
    double** offset_waypoints;
    double turn_rate; // rad/s NOTE: probably better to name this turn_rate_rad_s
    void (*run)(struct t_simulator*);
    control (*get_control)(struct t_simulator*,double);
} simulator;

// Declare functions to initialize and run a simulator.
simulator* create_simulator();
void run(struct t_simulator * sim);

#endif

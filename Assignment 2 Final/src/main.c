
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <client.h>
#include <unistd.h>
#include "simulator.h"
#include "vehicle.h"
#include <pthread.h>

// NOTE: See simulator.h for preprocessor constants that can turn the display on or off

int main(int argc, char *argv[])
{
    // Kill any lingering instances of the display process.
    system("pkill -f DisplayServer.py");

    #if USE_DISPLAY
    printf("\nLaunching the display server\n");
    // Launch the Python display server (in a separate process).
    launch_display_server();
    // Add a pause for python to start up.
    // If you get error messages about sockets,
    // a good start would be to increase this time.
    sleep(1);
    #endif

    // Create the simulator.
    printf("\nCreating the simulator");
    simulator* sim = create_simulator();
    // Simulator configuration.
    sim->n_vehicles = 3;
    sim->max_time = 60;
    sim->time_increment = 0.1;

    // Create 3 vehicles and add them to the simulator.
    sim->vehicles = malloc(sim->n_vehicles * sizeof(vehicle));
    for(int vehicle_index = 0; vehicle_index < sim->n_vehicles; vehicle_index++)
    {
        double vehicle_starting_position[3] = {vehicle_index*10+30, vehicle_index*10+30, 0}; // x, y, theta
        vehicle* vehicle = create_vehicle(vehicle_starting_position, sim->num_waypoints, sim->offset_waypoints);
        // Create a copy so we can free the original vehicle.
        sim->vehicles[vehicle_index] = *vehicle;
        free(vehicle);
    }

    // Run the simulation.
    printf("\nRunning the simulation");
    if(argc > 1){
        if (argv[1] == "threaded"){ (*sim).run_threaded(sim);}
        else{sim->run(sim);}
    } else {sim->run(sim);}
    // sim->run(sim);

    
    #if USE_DISPLAY
    // Wait for the user to quit the program.
    printf("\npress 'q' to quit the program\n");
    while(getchar() != 'q')  sleep(0.1);
    // Close the display server
    close_display_server();
    #endif

    // Clean up
    printf("\n\n");
}

#include "simulator.h"
#include "vehicle.h"
#include <stdlib.h>
#include "client.h"
#include <math.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/time.h>
#include <pthread.h>

// Initialize a simulator.
simulator* create_simulator()
{
    // Initialization
    simulator* sim = malloc(sizeof(simulator));
    sim->run = &run;
    sim->n_vehicles = 0;
    sim->vehicles = NULL;

    // Simulator settings
    sim->vehicle_update_rate = 25; // Hz
    sim->max_time = 60.0;
    sim->current_time = 0.0;
    sim->time_increment = 0.1;

    // Create "offset waypoints", a series of waypoints for the vehicles to follow.
    // Note that these waypoints are *relative to each vehicle's starting point*.
    sim->num_waypoints = 5;
    sim->radius = 30;
    sim->offset_waypoints = malloc((sim->num_waypoints + 1) * 2 * sizeof(double));
    for(int i = 0; i < sim->num_waypoints + 1; i++)
    {
        sim->offset_waypoints[i] = malloc(2 * sizeof(double));
        sim->offset_waypoints[i][0] = sim->radius * cos( i * 2 * M_PI / (sim->num_waypoints));
        sim->offset_waypoints[i][1] = sim->radius * sin( i * 2 * M_PI / (sim->num_waypoints));
    }

    // Return
    return sim;
}

// Run a simulator.
void run(struct t_simulator* sim)
{
    #if USE_DISPLAY
    // Start the display.
    printf("\nOpening the display server\n");
    open_server(IP, PORTNUM);
    sleep(1);
    // Send the vehicle waypoints.
    printf("\n");
    for(int vehicle_index = 0; vehicle_index < sim->n_vehicles; vehicle_index++)
    {
      printf("\rSending waypoint data to the display for vehicle %d/%d", vehicle_index+1, sim->n_vehicles);
      fflush(stdout); // force printf statements to be printed to the screen immediately
      send_vehicle_waypoints(vehicle_index, sim->num_waypoints, sim->vehicles[vehicle_index].target_waypoints);
    }
    #endif

    // Step through the simulation.
    struct timeval t_i;
    struct timeval t_f;
    int count = 0;
    
    sim->current_time = 0.0;
    double time_since_last_vehicle_message = 0.0;
    printf("\nStarting the simulation");
    #if USE_DISPLAY
    printf("\npress 'q' to quit the program");
    #endif
    printf("\n\n");
    
    gettimeofday(&t_i, NULL);
    while(sim->current_time < sim->max_time)
    {   count ++;
        #if PRINT_SIMULATION_TIME
        // Note that \r is a carriage return to send the cursor to the beginning of the line,
        // so the following print statement will overwrite text currently printed on that line of the terminal.
        printf("\rt = %0.04f", sim->current_time);
        #endif
        fflush(stdout); // force printf statements to be printed to the screen immediately
        time_since_last_vehicle_message += sim->time_increment;
        sim->current_time += sim->time_increment;
        // Display the new vehicle positions.
        if(time_since_last_vehicle_message > 1.0/sim->vehicle_update_rate)
        {
            #if USE_DISPLAY
            send_vehicles(sim->n_vehicles, sim->vehicles);
            #endif
            time_since_last_vehicle_message = 0.0;
        }
        // Control the vehicles and update their state.
        for(vehicle* v = sim->vehicles; v < sim->vehicles + sim->n_vehicles; v++)
        {
            v->control_vehicle(v);
            v->update_state(v, sim->time_increment); // delta t
        }
        // Sleep for roughly the time increment so we get quasi-realtime behavior.
        #if RUN_IN_REALTIME || USE_DISPLAY
        usleep(sim->time_increment*1e6);
        #endif
    }
    gettimeofday(&t_f, NULL);

    unsigned long t_i_us = 1000000*t_i.tv_sec + t_i.tv_usec;
    unsigned long t_f_us = 1000000*t_f.tv_sec + t_f.tv_usec;
    double avg_time = ((double)t_f_us - (double)t_i_us)/((double)count);
    printf("average time %f\n", avg_time);
    fflush(stdout);

    // Close the display process.
    #if USE_DISPLAY
    close_server();
    #endif
}


void * thread (void * arg){
    thread_data_t * data = (thread_data_t *)arg;
    vehicle * v = (*data).v;
    (*v).control_vehicle(v);
    (*v).update_state(v, (*data).time);
    
    pthread_exit(NULL);
}

void run_threaded(struct t_simulator* sim){
    #if USE_DISPLAY
    // Start the display.
    printf("\nOpening the display server\n");
    open_server(IP, PORTNUM);
    sleep(1);
    // Send the vehicle waypoints.
    printf("\n");
    for(int vehicle_index = 0; vehicle_index < sim->n_vehicles; vehicle_index++)
    {
      printf("\rSending waypoint data to the display for vehicle %d/%d", vehicle_index+1, sim->n_vehicles);
      fflush(stdout); // force printf statements to be printed to the screen immediately
      send_vehicle_waypoints(vehicle_index, sim->num_waypoints, sim->vehicles[vehicle_index].target_waypoints);
    }
    #endif

    // Step through the simulation.
    struct timeval t_i;
    struct timeval t_f;
    int count = 0;

    sim->current_time = 0.0;
    double time_since_last_vehicle_message = 0.0;
    printf("\nStarting the simulation");
    #if USE_DISPLAY
    printf("\npress 'q' to quit the program");
    #endif
    printf("\n\n");


    //Initializing threading variables
    int num = (*sim).n_vehicles;
    pthread_t thr[num];
    struct thread_data_t * thr_data = malloc(sizeof(struct thread_data_t)*num);

    
    gettimeofday(&t_i, NULL);
    while(sim->current_time < sim->max_time)
    {   count ++;
        #if PRINT_SIMULATION_TIME
        // Note that \r is a carriage return to send the cursor to the beginning of the line,
        // so the following print statement will overwrite text currently printed on that line of the terminal.
        printf("\rt = %0.04f", sim->current_time);
        #endif
        fflush(stdout); // force printf statements to be printed to the screen immediately
        time_since_last_vehicle_message += sim->time_increment;
        sim->current_time += sim->time_increment;
        // Display the new vehicle positions.
        if(time_since_last_vehicle_message > 1.0/sim->vehicle_update_rate)
        {
            #if USE_DISPLAY
            send_vehicles(sim->n_vehicles, sim->vehicles);
            #endif
            time_since_last_vehicle_message = 0.0;
        }

       //THREADING
       // Control the vehicles and update their state.
        for(int i = 0; i < num; i++)
        {   thr_data[i].thread_id = i;
            thr_data[i].v = &((*sim).vehicles[i]);
            thr_data[i].time = (*sim).time_increment;
            pthread_create(&thr[i], NULL, &thread, &thr_data[i]);
        }

        for (int i = 0; i < num; ++i)
        {
            pthread_join(thr[i], NULL);
        }
            
        // Sleep for roughly the time increment so we get quasi-realtime behavior.
        #if RUN_IN_REALTIME || USE_DISPLAY
        usleep(sim->time_increment*1e6);
        #endif
    }
    gettimeofday(&t_f, NULL);

    unsigned long t_i_us = 1000000*t_i.tv_sec + t_i.tv_usec;
    unsigned long t_f_us = 1000000*t_f.tv_sec + t_f.tv_usec;
    double avg_time = ((double)t_f_us - (double)t_i_us)/((double)count);
    printf("average time %f\n", avg_time);
    fflush(stdout);

    // Close the display process.
    #if USE_DISPLAY
    close_server();
    #endif
}
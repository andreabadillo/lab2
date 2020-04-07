#include "simulator.h"
#include "vehicle.h"
#include "controller.h"
#include <stdlib.h>
#include "client.h"
#include <math.h>
#include <unistd.h>
#include <stdio.h>


void set_position(struct t_vehicle * v,double * values){
    /*
    SHALL
    not exceed [0,100)
    heading limited to [-pi,pi)
    */

    printf("calling set position\n");
    fflush(stdout);
    for (int i = 0; i < 3; ++i)
    {
        printf("in for loop set POSITION, iteration %d\n", i);
        fflush(stdout);
        (*v).position[i] = values[i];
    }
    printf("set position\n");
    fflush(stdout);
}

void set_velocity(struct t_vehicle * v, double * values){
    /*
    lin vel not exceed range [5,10]
    ang vel not exceed range [-pi/4,pi/4]

    SHOULD take grav warping into account (can ignore)
    */
    printf("calling set velocity\n");
    fflush(stdout);
    for (int i = 0; i < 3; ++i)
    {
        printf("in for loop set velocity, iteration %d\n", i);
        fflush(stdout);

        (*v).velocity[i] = values[i];
        
        printf("set velocity\n");
        fflush(stdout);
    }
}


void control_vehicle(struct t_vehicle * v){
    /*
    intl routine to func that calls get_prop_waypt_ctrl and applies control
    */
    printf("calling control vehicle\n");
    fflush(stdout);


    control ctrl = get_proportional_waypoint_control(v);

    printf("control successfully called\n");
    fflush(stdout);

    double th = (*v).position[2];
    double x_vel = ctrl.speed * cos(th);
    double y_vel = ctrl.speed * sin(th);

    printf("linear velocities set\n");
    fflush(stdout);
    
    double values[3] = {x_vel, y_vel, ctrl.angular_velocity};

    set_velocity(v, values);
}


void update_state(struct t_vehicle * v, double delta_t){
    /*
    x += x_dot*delta_t
    y += y_dot*delta_t
    th += th_dot*delta_t
    */
    printf("update state\n");
    fflush(stdout);
    double temp;
    double update;

    for (int i = 0; i <3; ++i)
    {   printf("in for loop update state, iteration %d\n", i);
        fflush(stdout);

        temp = (*v).position[i] + (*v).velocity[i]*delta_t;
        
        printf("got temp variable\n");
        fflush(stdout);
        
        printf("about to update state\n");
        fflush(stdout);
        
        (*v).position[i] = update;
        
        printf("updated state\n");
        fflush(stdout);
    }
}


vehicle * create_vehicle(double * starting_pos, int n_waypoints, double ** offset_waypoints){
    // initialization
    vehicle * v = malloc(sizeof(vehicle));
    

    //set starting position on vehicle making sure that the starting position is within the bounds specified
    
    //initialize function pointers
    (*v).set_position = &set_position;
    (*v).set_velocity = &set_velocity;
    (*v).control_vehicle = &control_vehicle;
    (*v).update_state = &update_state;


    double pos_x = check_pos_bounds(starting_pos[0]);
    double pos_y = check_pos_bounds(starting_pos[1]);
    double th = check_head_bounds(starting_pos[2]);
    printf("accessed positions create vehicle\n");
    fflush(stdout);

    double pos[3] = {pos_x, pos_y, th};
    set_position(v, pos);
    printf("set positions create vehicle\n");
    fflush(stdout);

    //initializing target waypoints to be rel to world frame rather than starting pos
    (*v).num_waypoints = n_waypoints;
    (*v).current_waypoint_idx = 0;
    (*v).current_waypoint = malloc(2*sizeof(double));

    (*v).target_waypoints = malloc(((*v).num_waypoints) * sizeof(double*));

    for (int i = 0; i < n_waypoints+1; i++) {
        printf("in for loop target waypoints, iteration %d\n",i);
        fflush(stdout);
        (*v).target_waypoints[i] = malloc(2 * sizeof(double));
        (*v).target_waypoints[i][0] = check_pos_bounds(offset_waypoints[i][0] + (*v).position[0]);
        (*v).target_waypoints[i][1] = check_pos_bounds(offset_waypoints[i][1] + (*v).position[1]);
        printf("set target waypoints, iteration %d\n",i);
        fflush(stdout);
        if(i == (*v).current_waypoint_idx){
            printf("setting initial waypoints\n");
            fflush(stdout);
            (*v).current_waypoint[0] = (*v).target_waypoints[i][0];
            (*v).current_waypoint[1] = (*v).target_waypoints[i][1];
            printf("initial waypoints set\n");
            fflush(stdout);
        }
    }

    //set initial vel using controller
    control * init = malloc(sizeof(control));
    printf("getting prop control initial\n");
    fflush(stdout);
    *init = get_proportional_waypoint_control(v);
    printf("prop control achieved, setting values\n");
    fflush(stdout);
    double x_vel = (*init).speed * cos(th);
    double y_vel = (*init).speed * sin(th);
    double vel[3] = {x_vel, y_vel, (*init).angular_velocity};
    printf("values set\n");
    fflush(stdout);
    set_velocity(v, vel);

    return v;
}
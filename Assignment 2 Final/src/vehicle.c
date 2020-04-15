#include "simulator.h"
#include "vehicle.h"
#include "controller.h"
#include <stdlib.h>
#include "client.h"
#include <math.h>
#include <unistd.h>
#include <stdio.h>
#include <pthread.h>

double check_pos_bounds(double val){
    if(val < 0){
        val = 0.0;
    }
    if(val >=100.0){
        val =  99.9999;
    }
    return val;
}

double check_head_bounds(double val){
    // printf("checking head bounds\n");
    // fflush(stdout);

	//either returns original theta if within bounds or will
	//simplify theta to be within bounds

	while(val < 0.0){
		val += 2*M_PI;
	}

	//pass through theta if already bounded
    if((val >= -M_PI) && (val < M_PI)){
        return val;
    }

   //get equivalent theta in 0 to 2pi if not bounded
    val = fmod(val,(2*M_PI));

    //check if less than pi, if not, make a neg angle (will always be between -pi and pi now)
    if (val < M_PI)
    {
    	return val;
    }

    return val - 2*M_PI;
  

}

void set_position(struct t_vehicle * v,double * values){
    /*
    SHALL
    not exceed [0,100)
    heading limited to [-pi,pi)
    */
    for (int i = 0; i < 3; ++i)
    {
    	(*v).position[i] = values[i];
    }
    printf("x pos: %f, y pos %f, theta = %f\n", (*v).position[0],(*v).position[1], (*v).position[2]);
    fflush(stdout);
}

void set_velocity(struct t_vehicle * v, double * values){
    /*
    lin vel not exceed range [5,10]
    ang vel not exceed range [-pi/4,pi/4]

    SHOULD take grav warping into account (can ignore)
    */
    for (int i = 0; i < 3; ++i)
    {

        (*v).velocity[i] = values[i];
        
    }
}


void control_vehicle(struct t_vehicle * v){
    /*
    intl routine to func that calls get_prop_waypt_ctrl and applies control
    */
    
    control ctrl = get_proportional_waypoint_control(v);
    
        
    double th = (*v).position[2];
    double x_vel = ctrl.speed * cos(th);
    double y_vel = ctrl.speed * sin(th);
    double ang_vel = ctrl.angular_velocity;
    

    double values[3] = {x_vel, y_vel, ang_vel};

    set_velocity(v, values);
}


void update_state(struct t_vehicle * v, double delta_t){
    /*
    x += x_dot*delta_t
    y += y_dot*delta_t
    th += th_dot*delta_t
    */

    double temp;
    double update;
    double vals[3];
   
    for (int i = 0; i <3; ++i)
    {   
        
        temp = (*v).position[i] + (*v).velocity[i]*delta_t;
        
        
        if (i<2){
            update = check_pos_bounds(temp);
        }
        else{
            update = check_head_bounds(temp);
        }
        
        vals[i] = update;

    }
    set_position(v, vals);

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


    (*v).position[0] = check_pos_bounds(starting_pos[0]);
    (*v).position[1] = check_pos_bounds(starting_pos[1]);
    (*v).position[2] = check_head_bounds(starting_pos[2]);

    //set initial vel to min values and ang_vel 0, let next update control
    (*v).velocity[0] = 6.0;
    (*v).velocity[1] = 6.0;
    (*v).velocity[2] = M_PI/5;


    //initializing target waypoints to be rel to world frame rather than starting pos
    //deleting waypoints that don't fit into this frame
    (*v).current_waypoint_idx = 0;
    int counter = 0;

    double ** act_waypoints = malloc(n_waypoints * sizeof(double*));

    for (int i = 0; i < n_waypoints; i++) {
        double temp_x = offset_waypoints[i][0] + (*v).position[0];
        double temp_y = offset_waypoints[i][1] + (*v).position[1];
        
        if ((temp_x == check_pos_bounds(temp_x)) && temp_y == check_pos_bounds(temp_y))
        {
        	act_waypoints[counter] = malloc(2 * sizeof(double));
	        act_waypoints[counter][0] = temp_x;
	        act_waypoints[counter][1] = temp_y;
  			counter++;
        }
              
    }

    (*v).num_waypoints = counter;
    (*v).target_waypoints = malloc((*v).num_waypoints * sizeof(double*));

    for (int i = 0; i < counter; ++i)
    {
    	(*v).target_waypoints[i] = malloc(2 * sizeof(double));
        (*v).target_waypoints[i][0] = act_waypoints[i][0];
        (*v).target_waypoints[i][1] = act_waypoints[i][1];
    }

    (*v).current_waypoint = malloc(2*sizeof(double));
    (*v).current_waypoint = (*v).target_waypoints[0];
    return v;

    }

    
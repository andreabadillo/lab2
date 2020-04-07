#include "simulator.h"
#include "vehicle.h"
#include "controller.h"
#include <stdlib.h>
#include "client.h"
#include <math.h>
#include <unistd.h>
#include <stdio.h>

double check_pos_bounds(double val){
    // printf("checking pos bounds\n");
    // fflush(stdout);
    if(val < 0){
        val = 0.0;
    }
    if(val >=100.0){
        val =  99.9999;
    }
    printf("value at pos_bounds: %f\n", val);
    fflush(stdout);
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
    printf("calling set position\n");
    fflush(stdout);
    for (int i = 0; i < 3; ++i)
    {
    	printf("position value: %f, idx %d\n", values[i],i );
    	fflush(stdout);
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
    printf("calling set velocity\n");
    fflush(stdout);
    for (int i = 0; i < 3; ++i)
    {
        // printf("in for loop set velocity, iteration %d\n", i);
        // fflush(stdout);

        (*v).velocity[i] = values[i];
        
        // printf("set velocity\n");
        // fflush(stdout);
    }
    printf("x vel: %f, y vel: %f, theta = %f\n", (*v).velocity[0],(*v).velocity[1], (*v).velocity[2]);
    fflush(stdout);
}


void control_vehicle(struct t_vehicle * v){
    /*
    intl routine to func that calls get_prop_waypt_ctrl and applies control
    */
    printf("calling control vehicle\n");
    fflush(stdout);

    control ctrl = get_proportional_waypoint_control(v);
    
    printf("control successfully called: speed = %f, angvel = %f\n", ctrl.speed, ctrl.angular_velocity);
    fflush(stdout);
    
    double th = (*v).position[2];
    double x_vel = ctrl.speed * cos(th);
    double y_vel = ctrl.speed * sin(th);
    double ang_vel = ctrl.angular_velocity;
    

    printf("variables x vel: %f, y vel: %f, th vel = %f\n", x_vel,y_vel, ang_vel);
    fflush(stdout);

    // printf("linear velocities set\n");
    // fflush(stdout);
    double values[3] = {x_vel, y_vel, ang_vel};

    printf("array x vel: %f, y vel: %f, th vel = %f\n", values[0],values[1], values[2]);
    fflush(stdout);

    set_velocity(v, values);
}


void update_state(struct t_vehicle * v, double delta_t){
    /*
    x += x_dot*delta_t
    y += y_dot*delta_t
    th += th_dot*delta_t
    */
    // printf("update state\n");
    // fflush(stdout);

    double temp;
    double update;
    double vals[3];
   
    for (int i = 0; i <3; ++i)
    {   
    	// printf("in for loop update state, iteration %d\n", i);
    //     fflush(stdout);
        
        temp = (*v).position[i] + (*v).velocity[i]*delta_t;
        printf("updating state: %f\n", temp);
        fflush(stdout);
        // printf("got temp variable\n");
        // fflush(stdout);
        
        if (i<2){
            update = check_pos_bounds(temp);
            printf("update value x_y: %f\n", update);
        	fflush(stdout);
        }
        else{
            update = check_head_bounds(temp);
            printf("update value th: %f\n", update);
        	fflush(stdout);
        }
        
        // printf("about to update state\n");
        // fflush(stdout);
        
        vals[i] = update;
        printf("did array update properly? %f\n", vals[i]);
        fflush(stdout);
        // printf("updated state\n");
        // fflush(stdout);
    }
    set_position(v, vals);

    //updating waypoint maybe
    double dist = sqrt(pow((*v).current_waypoint[0]-(*v).position[0],2) + pow((*v).current_waypoint[1]-(*v).current_waypoint[1],2));
    
    printf("calculated distance, %f\n", dist);
    fflush(stdout);
    
    if (dist < 0.5)
    {
        printf("increasing index\n");
        fflush(stdout);

        int num = (*v).current_waypoint_idx + 1;
        (*v).current_waypoint_idx = num%((*v).num_waypoints);

        printf("updating current waypoint to match next one\n");
        fflush(stdout);

        (*v).current_waypoint[0] = (*v).target_waypoints[(*v).current_waypoint_idx][0];
        (*v).current_waypoint[1] = (*v).target_waypoints[(*v).current_waypoint_idx][1];
        
        printf("successfully increased waypoint\n");
        fflush(stdout);
    }

}


vehicle * create_vehicle(double * starting_pos, int n_waypoints, double ** offset_waypoints){
    // initialization
    // printf("initializing vehicle\n");
    // fflush(stdout);
    vehicle * v = malloc(sizeof(vehicle));
    // printf("allocated vehicle memory\n");
    // fflush(stdout);
    //set starting position on vehicle making sure that the starting position is within the bounds specified
    
    //initialize function pointers
    (*v).set_position = &set_position;
    (*v).set_velocity = &set_velocity;
    (*v).control_vehicle = &control_vehicle;
    (*v).update_state = &update_state;


    (*v).position[0] = check_pos_bounds(starting_pos[0]);
    (*v).position[1] = check_pos_bounds(starting_pos[1]);
    (*v).position[2] = check_head_bounds(starting_pos[2]);

    printf("x pos: %f, y pos %f, theta = %f\n", ((*v).position[0],(*v).position[1], (*v).position[2]));
    fflush(stdout);
    // printf("accessed positions create vehicle\n");
    // fflush(stdout);

    //set initial vel to min values and ang_vel 0, let next update control
    (*v).velocity[0] = 6.0;
    (*v).velocity[1] = 6.0;
    (*v).velocity[2] = M_PI/5;

    // printf("set positions create vehicle\n");
    // fflush(stdout);

    //initializing target waypoints to be rel to world frame rather than starting pos
    (*v).num_waypoints = n_waypoints;
    (*v).current_waypoint_idx = 0;

    

    (*v).target_waypoints = malloc(((*v).num_waypoints) * sizeof(double*));

    for (int i = 0; i < n_waypoints; i++) {
        // printf("in for loop target waypoints, iteration %d\n",i);
        // fflush(stdout);

        (*v).target_waypoints[i] = malloc(2 * sizeof(double));
        (*v).target_waypoints[i][0] = fmod(fmod(offset_waypoints[i][0] + (*v).position[0],100)+100, 100);
        printf("waypoint %d x pos is %f\n",(i,(*v).target_waypoints[i][0]));
        fflush(stdout);

        (*v).target_waypoints[i][1] = fmod(fmod(offset_waypoints[i][1] + (*v).position[1],100)+100, 100);
        printf("waypoint %d y pos is %f\n",(i,(*v).target_waypoints[i][1]));
        fflush(stdout);
        // printf("set target waypoints, iteration %d\n",i);
        // fflush(stdout);
        }
    (*v).current_waypoint = malloc(2*sizeof(double));
    (*v).current_waypoint = (*v).target_waypoints[0];
    return v;

    }

    


    // control * init = malloc(sizeof(control));
    // *init = get_proportional_waypoint_control(v);
    
    // printf("prop control achieved, setting values\n");
    // fflush(stdout);
    
    // double x_vel = (*init).speed * cos(th);
    // double y_vel = (*init).speed * sin(th);
    // double vel[3] = {x_vel, y_vel, (*init).angular_velocity};
    
    // printf("values set\n");
    // fflush(stdout);
    
    // set_velocity(v, vel);

    
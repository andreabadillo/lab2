#include "controller.h"
#include <stdlib.h>
#include <math.h>
#include<stdio.h>

double check_pos_bounds(double val){
    printf("checking pos bounds\n");
    fflush(stdout);
    if(val < 0){
        printf("will be zero\n");
        fflush(stdout);
        return 0;
    }
    if(val >=100){
        printf("will be 99.9\n");
        fflush(stdout);
        return 99.9;
    }
    printf("will pass value pos\n");
    fflush(stdout);
    return val;
}

double check_head_bounds(double val){
    printf("checking head bounds\n");
    fflush(stdout);
    if(val < -M_PI){
        printf("will be lower bound\n");
        fflush(stdout);
        return -M_PI;
    }

    if(val >= M_PI){
        printf("will be higher bound\n");
        fflush(stdout);
        return 179*M_PI/180;
    }
    printf("pass value head\n");
    fflush(stdout);
    return val;
}

double check_linv_bounds(double val){
    printf("checking linv bounds\n");
    fflush(stdout);
    if(val < 5){
        printf("will be lower bound\n");
        fflush(stdout);
        return 5;
    }
    if(val > 10){
        printf("will be higher bound\n");
        fflush(stdout);
        return 10;
    }
    printf("pass value linv\n");
    fflush(stdout);
    return val;
}

double check_angv_bounds(double val){
    printf("checking angv bounds\n");
    fflush(stdout);
    if(val < -M_PI_4){
        printf("will be lower bounds\n");
        fflush(stdout);
        return -M_PI_4;
    }
    if(val > M_PI_4){
        printf("will be higher bound\n");
        fflush(stdout);
        return M_PI_4;
    }
    printf("pass value angv\n");
    fflush(stdout);
    return val;
}

control get_proportional_waypoint_control(struct t_vehicle * vehicle){
	printf("initializing controller \n");
    fflush(stdout);
    control * ctrl = malloc(sizeof(control));
    printf("allocated space to controller, about to get pos\n");
    fflush(stdout);
    
    /*
    //pos_v is position of vehicle and pos_w is position of waypoint
    double pos_vx = (*vehicle).position[0];
    double pos_vy = (*vehicle).position[1];
    printf("got pos vehicle\n");
    printf("%f%f\n",(pos_vx,pos_vy));
    fflush(stdout);
    
    double pos_wx = (*vehicle).current_waypoint[0];
    double pos_wy = (*vehicle).current_waypoint[1]; 
    printf("got pos waypoint\n");
    printf("%f%f\n", (pos_wx,pos_wy));
    fflush(stdout);

    //---------------------------------------------------------------
    //checking if waypoint needs to be updated and if vehicle has 
    //passed through all the waypoints
    double dist = sqrt(pow(pos_wx-pos_vx,2) + pow(pos_wy-pos_vy,2));
    printf("calculated distance\n");
    fflush(stdout);
    if (dist < 0.5)
    {
        printf("increasing index\n");
        fflush(stdout);
        (*vehicle).current_waypoint_idx++;

        if((*vehicle).current_waypoint_idx < (*vehicle).num_waypoints){
        printf("updating current waypoint to match next one\n");
        fflush(stdout);
        (*vehicle).current_waypoint[0] = (*vehicle).target_waypoints[(*vehicle).current_waypoint_idx][0];
        (*vehicle).current_waypoint[1] = (*vehicle).target_waypoints[(*vehicle).current_waypoint_idx][1];
        printf("successfully increased waypoint\n");
        fflush(stdout);
        }

        else{
            (*ctrl).speed = 0;
            (*ctrl).angular_velocity = 0;
            return *ctrl;
        }
    }

    //---------------------------------------------------------------
	//getting angle between vehicle heading and waypoint
    printf("getting ang vehicle\n");
    fflush(stdout);
    double ang_v = (*vehicle).position[2];
    printf("successfully got ang vehicle\n");
    fflush(stdout);
    double ang_w  = tan(pos_wy/pos_wx); //angle of the waypoint
    
    double angle_diff;
    
    double v_w = ang_v - ang_w;
    double w_v = ang_w - ang_v;
    
    if (abs(v_w) < abs(w_v)){
        angle_diff = v_w;
    }
    else{
        angle_diff = w_v;
    }
    
    //---------------------------------------------------------------
	//controllers

	//speed prop controller based on distance error
    double speed = check_linv_bounds(dist); 

	//angular controller based off of specifications
	double ang_vel = check_angv_bounds(angle_diff);
	*/

    //for debugging purposes
    double speed = 6;
    double ang_vel = M_PI/6;

	(*ctrl).speed = speed;
	(*ctrl).angular_velocity = ang_vel; 
    printf("returning control\n");
    fflush(stdout);
	return *ctrl;
}

/*
SHALLS

angular vel with angular vel equal to diff in radians from angle between
angular heading of vehicle and current waypoint

operate on smallest angle diff |theta - th_hat| <= pi
//question what is theta_hat ???

absolute control output bounded (lin vel [5,10) (ang vel [-pi/4, pi/4])

SHOULDS

alt control policy in polynomial time finds min dist path causing
vehicle to visit all target waypoints
*/
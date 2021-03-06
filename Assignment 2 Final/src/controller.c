#include "controller.h"
#include <stdlib.h>
#include <math.h>
#include<stdio.h>

double check_angle(double val){
	//either returns original theta if within bounds or will
	//simplify theta to be within bounds

	while(val < 0.0){
		val += 2*M_PI;
	}

	//pass through theta if already bounded
    if((val >= -M_PI) && (val <= M_PI)){
        return val;
    }

   //get equivalent theta in 0 to 2pi if not bounded
    val = fmod(val,(2*M_PI));

    //check if less than pi, if not, make a neg angle (will always be between -pi and pi now)
    if (val <= M_PI)
    {
    	return val;
    }

    return val - 2*M_PI;
  

}

double check_linv_bounds(double val){
    if(val < 5){

        return 5;
    }
    if(val > 10){

        return 10;
    }

    return val;
}

double check_angv_bounds(double val){

    if(val < -M_PI_4){

        return -M_PI_4;
    }
    if(val > M_PI_4){

        return M_PI_4;
    }
    return val;
}

void update_waypoint(vehicle * v){
    if ((abs((*v).position[0]-(*v).current_waypoint[0])<0.2) && (abs((*v).position[1] - (*v).current_waypoint[1])<0.2))
    {
        int num = (*v).current_waypoint_idx + 1;
        if (num == (*v).num_waypoints)
        {
            num = 0;
        }
        (*v).current_waypoint_idx = num;

        (*v).current_waypoint[0] = (*v).target_waypoints[(*v).current_waypoint_idx][0];
        (*v).current_waypoint[1] = (*v).target_waypoints[(*v).current_waypoint_idx][1];
        
    }
}

control get_proportional_waypoint_control(struct t_vehicle * vehicle){
      
    control * ctrl = malloc(sizeof(control));
    
    update_waypoint(vehicle);

    //pos_v is position of vehicle and pos_w is position of waypoint
    double pos_vx = (*vehicle).position[0];
    double pos_vy = (*vehicle).position[1];
        
    double pos_wx = (*vehicle).current_waypoint[0];
    double pos_wy = (*vehicle).current_waypoint[1]; 
    
    double dist = sqrt(pow(pos_wx-pos_vx,2) + pow(pos_wy-pos_vy,2));
   
    //---------------------------------------------------------------
	//getitng angle difference relative to vehicle and waypoint
    double ang_v = (*vehicle).position[2]; //angle of vehicle heading
    double ang_g  = atan2((pos_wy-pos_vy),(pos_wx-pos_vx)); //goal angle
    
    //since abs(angle diff) must be less than pi, make this happen
    double angle_diff;

    //makes sure that angle difference is within neg pi and pi
    angle_diff = fmod((ang_g-ang_v),2*M_PI);
    angle_diff = check_angle(angle_diff);
        
    //---------------------------------------------------------------
	//controllers

	//speed prop controller based on distance error
    (*ctrl).speed = check_linv_bounds(dist); 
    
	//angular controller based off of specifications
	(*ctrl).angular_velocity = check_angv_bounds(angle_diff);

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
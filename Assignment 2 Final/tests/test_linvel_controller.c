#include <string.h>
#include <math.h>
#include <stdlib.h>
#include "vehicle.h"
#include "controller.h"

control start_test_c(double starting_pos[], int n_waypoints, double ** offset_waypoints){
	vehicle * test_v = create_vehicle(starting_pos, n_waypoints, offset_waypoints);
	control ctrl = get_proportional_waypoint_control(test_v);
   	return ctrl;
   }

//Unit Tests: Velocity Controller Working Properly
//Set heading and angle of waypoint equal to just account for velocity output

//Lower bound vel 5 (offset waypoint 4,3) | Should spit out 5
int low_bound(){
	double * offset[2];
	offset[0] = malloc(2*sizeof(double));
	offset[0][0] = 4;
	offset[0][1] = 3;
	double start[3] = {0, 0, atan2(offset[0][1],offset[0][0])};

	control ctrl = start_test_c(start, 1, offset);
	if(ctrl.speed == 5){return 0;}
	else {return -1;}
}
//Outside lower bound vel (offset waypoint 2,3) | Should spit out 5
int out_low_bound(){
	double * offset[2];
	offset[0] = malloc(2*sizeof(double));
	offset[0][0] = 2;
	offset[0][1] = 3;
	double start[3] = {0, 0, atan2(offset[0][1],offset[0][0])};

	control ctrl = start_test_c(start, 1, offset);
	if(ctrl.speed == 5){return 0;}
	else {return -1;}
}
//Upper bound vel 10 (offset waypoints 8,6) | Should spit out 10
int upp_bound(){
	double * offset[2];
	offset[0] = malloc(2*sizeof(double));
	offset[0][0] = 8;
	offset[0][1] = 6;
	double start[3] = {0, 0, atan2(offset[0][1],offset[0][0])};

	control ctrl = start_test_c(start, 1, offset);
	if(ctrl.speed == 10){return 0;}
	else {return -1;}
}

//Outside upper bound vel (offset waypoint 12,6) | Should spit out 10
int out_upp_bound(){
	double * offset[2];
	offset[0] = malloc(2*sizeof(double));
	offset[0][0] = 12;
	offset[0][1] = 6;
	double start[3] = {0, 0, atan2(offset[0][1],offset[0][0])};

	control ctrl = start_test_c(start, 1, offset);
	
	if(ctrl.speed == 10){return 0;}
	else {return -1;}
}


int main (int argc, char ** argv) {
    // test controller functionality
    int test = atoi(argv[1]);

    switch(test){
    	case 1:
    	return low_bound();
    	break;

    	case 2:
    	return  out_low_bound();
    	break;

    	case 3:
    	return upp_bound();
    	break;

    	case 4:
    	return out_upp_bound();
    	break;
    }

    return -1;
    
}
#include <string.h>
#include "vehicle.h"
#include "controller.h"
#include <stdio.h>

control start_test_c(double * starting_pos, int num_waypoints, double ** offset_waypoints){
	vehicle * test_v = create_vehicle(double * starting_pos, int n_waypoints, double ** offset_waypoints)
	control ctrl = get_proportional_waypoint_control(test_v);
   	return ctrl;
   }

//Unit Tests: Angular Controller Working Properly
//Set waypoint offset equal (to just account for angular ouput

//Heading facing Target 
int zero_ang(){
	double * offset[2];
	offset[0] = malloc(2*sizeof(double));
	offset[0][0] = 10;
	offset[0][1] = 10;
	double start[3] = {0, 0, atan2(offset[0][1],offset[0][0])};

	control ctrl = get_proportional_waypoint_control(start_test_c(start, 1, offset);
	if(ctrl.angular_velocity == 0){return 0;}
	else {return -1;}
}


//Heading to left of target
int pos_angle(){
	double * offset[2];
	offset[0] = malloc(2*sizeof(double));
	offset[0][0] = 10;
	offset[0][1] = 10;
	double start[3] = {0, 0, 0};

	control ctrl = start_test_c(start, 1, offset);
	if(ctrl.angular_velocity == M_PI/4){return 0;}
	else {return -1;}
}

int pos_angle_outlim(){
	double * offset[2];
	offset[0] = malloc(2*sizeof(double));
	offset[0][0] = 10;
	offset[0][1] = 15;
	double start[3] = {0, 0, 0};

	control ctrl = start_test_c(start, 1, offset);
	if(ctrl.angular_velocity == M_PI/4){return 0;}
	else {return -1;}
}

//Heading to right of target
int neg_angle(){
	double * offset[2];
	offset[0] = malloc(2*sizeof(double));
	offset[0][0] = 10;
	offset[0][1] = -10;
	double start[3] = {20, 20, 0};

	control ctrl = start_test_c(start, 1, offset);
	if(ctrl.angular_velocity == -M_PI/4){return 0;}
	else {return -1;}
}

int main (int argc, char ** argv) {
    // test controller functionality
    int test = atoi(argv[1])

    switch(test){
    	case 1:
    	return zero_ang();
    	break;

    	case 2:
    	return  pos_angle();
    	break;

    	case 3:
    	return pos_angle_outlim();
    	break;

    	case 4:
    	return neg_angle();
    	break;
    }

    return -1
}
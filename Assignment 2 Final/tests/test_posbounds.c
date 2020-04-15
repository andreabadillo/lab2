#include <string.h>
#include <math.h>
#include <stdlib.h>
#include "vehicle.h"
#include "controller.h"

vehicle * start_test_v(double starting_pos[], int num_waypoints, double ** offset_waypoints){
	vehicle * test_v= create_vehicle(starting_pos, num_waypoints, offset_waypoints);
	(*test_v).control_vehicle(test_v);
	(*test_v).update_state(test_v, 0.1); //matching time increment of sim given
   	return test_v;
   }

//Unit Tests: 
//Set waypoint offset equal (to just account for angular ouput

//Heading facing Target 
int upper_linbound(){
	double * offset[2];
	offset[0] = malloc(2*sizeof(double));
	offset[0][0] = -15;
	offset[0][1] = -0.5;
	double start[3] = {50, 99.8, M_PI_4};

	vehicle * v = start_test_v(start, 1, offset);
	if((*v).position[1] < 100){return 0;}
	else {return -1;}
}


//trying to exit thru bottom
int lower_linbound(){
	double * offset[2];
	offset[0] = malloc(2*sizeof(double));
	offset[0][0] = 15;
	offset[0][1] = 0;
	double start[3] = {50, 0.1, -M_PI/2};

	vehicle * v = start_test_v(start, 1, offset);
	if((*v).position[1] == 0){return 0;}
	else {return -1;}
}

//trying to exit thru left
int left_linbound(){
	double * offset[2];
	offset[0] = malloc(2*sizeof(double));
	offset[0][0] = 0;
	offset[0][1] = -10;
	double start[3] = {0.1, 50, 9*M_PI/10};

	vehicle * v = start_test_v(start, 1, offset);
	if((*v).position[0] == 0){return 0;}
	else {return -1;}
}

//trying to exit thru right
int right_linbound(){
	double * offset[2];
	offset[0] = malloc(2*sizeof(double));
	offset[0][0] = 0;
	offset[0][1] = 10;
	double start[3] = {99.8, 50, -M_PI/10};

	vehicle * v = start_test_v(start, 1, offset);
	if((*v).position[0] < 100){return 0;}
	else {return -1;}
}

//trying to increase past pi
int upp_angbound(){
	double * offset[2];
	offset[0] = malloc(2*sizeof(double));
	offset[0][0] = -10;
	offset[0][1] = -10;
	double start[3] = {50, 50, 179*M_PI/180};

	vehicle * v = start_test_v(start, 1, offset);
	if((*v).position[2] < M_PI){return 0;}
	else {return -1;}
}

//trying to decrease past neg pi
int low_angbound(){
	double * offset[2];
	offset[0] = malloc(2*sizeof(double));
	offset[0][0] = 10;
	offset[0][1] = 10;
	double start[3] = {50, 50, -179*M_PI/180};

	vehicle * v = start_test_v(start, 1, offset);
	if((*v).position[2] > -M_PI){return 0;}
	else {return -1;}
}

int main (int argc, char ** argv) {
    // test controller functionality
    int test = atoi(argv[1]);

    switch(test){
    	case 1:
    	return upper_linbound();
    	break;

    	case 2:
    	return  lower_linbound();
    	break;

    	case 3:
    	return left_linbound();
    	break;

    	case 4:
    	return right_linbound();
    	break;

    	case 5:
    	return low_angbound();
    	break;

    	case 6:
    	return upp_angbound();
    	break;
    }

    return -1;
}
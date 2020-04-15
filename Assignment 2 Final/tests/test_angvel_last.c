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

int main (int argc, char ** argv){
	double * offset[2];
	offset[0] = malloc(2*sizeof(double));
	offset[0][0] = 10;
	offset[0][1] = -15;
	double start[3] = {20, 20, 0};

	control ctrl = start_test_c(start, 1, offset);


	if(ctrl.angular_velocity == -M_PI/4){return 0;}
	else {return -1;}
}
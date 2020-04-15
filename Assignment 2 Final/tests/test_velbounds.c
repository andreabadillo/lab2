#include <string.h>
#include <math.h>
#include <stdlib.h>
#include "vehicle.h"
#include "controller.h"

//Maxing out Velocity
int upvel_linx(){
	double * offset[2];
	offset[0] = malloc(2*sizeof(double));
	offset[0][0] = 15;
	offset[0][1] = 0;
	double start[3] = {50, 50, 0};
	vehicle * v = create_vehicle(start, 1, offset);

	double ii[3] = {9.8,9.8,9*M_PI/40};
	(*v).set_velocity(v, ii);
	(*v).control_vehicle(v);

	if(abs((*v).velocity[0]) == 10){return 0;}
	else {return -1;}
}

int upvel_liny(){
	double * offset[2];
	offset[0] = malloc(2*sizeof(double));
	offset[0][0] = 0;
	offset[0][1] = 15;
	double start[3] = {50, 50, M_PI/2};

	vehicle * v = create_vehicle(start, 1, offset);

	double ii[3] = {9.8,9.8,0};
	(*v).set_velocity(v, ii);
	(*v).control_vehicle(v);
	if(abs((*v).velocity[1]) == 10){return 0;}
	else {return -1;}
}

int upvel_th(){
	double * offset[2];
	offset[0] = malloc(2*sizeof(double));
	offset[0][0] = 10;
	offset[0][1] = 15;
	double start[3] = {50, 50, 0};
	vehicle * v = create_vehicle(start, 1, offset);

	double ii[3] = {9.8,9.8,9*M_PI/40};
	(*v).set_velocity(v, ii);
	(*v).control_vehicle(v);
	
	if(abs((*v).velocity[2]) <= M_PI_4){return 0;}
	else {return -1;}
}


//Minning out Velocity
int lvel_linx(){
	double * offset[2];
	offset[0] = malloc(2*sizeof(double));
	offset[0][0] = 3;
	offset[0][1] = 0;
	double start[3] = {50, 50, 0};
	vehicle * v = create_vehicle(start, 1, offset);

	double ii[3] = {9.8,9.8,0};
	(*v).set_velocity(v, ii);
	(*v).control_vehicle(v);
	if((*v).velocity[0] == 5){return 0;}
	else {return -1;}
}

int lvel_liny(){
	double * offset[2];
	offset[0] = malloc(2*sizeof(double));
	offset[0][0] = 0;
	offset[0][1] = 3;
	double start[3] = {50, 50, M_PI/2};
	vehicle * v = create_vehicle(start, 1, offset);

	double ii[3] = {9.8,9.8,0};
	(*v).set_velocity(v, ii);
	(*v).control_vehicle(v);
	if((*v).velocity[1] == 5){return 0;}
	else {return -1;}
}

int lvel_th(){
	double * offset[2];
	offset[0] = malloc(2*sizeof(double));
	offset[0][0] = 10;
	offset[0][1] = -15;
	double start[3] = {50, 50, -179*M_PI/180};
	vehicle * v = create_vehicle(start, 1, offset);

	double ii[3] = {6,6,-9*M_PI/40};
	(*v).set_velocity(v, ii);
	(*v).control_vehicle(v);
	if((*v).velocity[2] >= -M_PI_4){return 0;}
	else {return -1;}
}



int main (int argc, char ** argv) {
    // test controller functionality
    int test = atoi(argv[1]);

    switch(test){
    	case 1:
    	return upvel_linx();
    	break;

    	case 2:
    	return  upvel_liny();
    	break;

    	case 3:
    	return upvel_th();
    	break;

    	case 4:
    	return lvel_linx();
    	break;

    	case 5:
    	return  lvel_liny();
    	break;

    	case 6:
    	return lvel_th();
    	break;
    }

    return -1;
}
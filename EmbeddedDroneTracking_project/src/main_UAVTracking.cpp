 //The main run file for the UAVTracking project

#include "UAVTracking.h"
UAVTracking<double> uav_tracking;
UAVTracking_parameters uavParams;

int main(){
uav_tracking.UAVTracking_Solver(uavParams);
return 0;
}
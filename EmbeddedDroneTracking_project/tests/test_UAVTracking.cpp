//Testing the functionality from UAVTracking.h
#include <gtest/gtest.h>
#include "UAVTracking.h"

UAVTracking <double> uav_tracking;

TEST(MissionAttempt,InvalidParameters){
//Ensures that the tracking program won't start
UAVTracking_parameters uavParams(1e-7,100,2,12,35,700,2,4,1e2,5,200,2,-100,1.0,2.0,1.0,false,-1e-4);
EXPECT_THROW({uav_tracking.UAVTracking_Solver(uavParams);},std::invalid_argument);
}

TEST(Solution,GuessInsideTheRadar){
//Testing the stability for the G-N strategy by checking valid maximal velocity and step-size
UAVTracking_parameters uavParams(1e-7,1000,2,12,35,700,2,4,1e2,5,50,5.0,0.1,1e-5,2.0,1.0,false,1e-4);
uav_tracking.UAVTracking_Solver(uavParams);
}

TEST(Solution,GuessOutsideTheRadar){
//Testing the stability for the G-N strategy by checking valid maximal velocity and step-size
UAVTracking_parameters uavParams(1e-7,1000,2,12,35,700,2,4,1e2,5,101,5,10,1e-5,2.0,1.0,false,1e-4);
uav_tracking.UAVTracking_Solver(uavParams);
}

TEST(FastUAVs,GuessInsideTheRadar){
//Testing the stability for the G-N strategy by checking valid maximal velocity and step-size. The trick is to choose a distance to the target
//that is far away when instability can occur.
UAVTracking_parameters uavParams(1e-7,1000,2,12,35,700,2,4,1e2,5,10,2,100,1.0,2.0,1.0,false,1e-4);
uav_tracking.UAVTracking_Solver(uavParams);
}

TEST(FastUAVs,GuessOutsideTheRadar){
//Testing the stability for the G-N strategy by checking valid maximal velocity and step-size. The trick is to choose a distance to the target
//that is far away when instability can occur.
UAVTracking_parameters uavParams(1e-7,1000,2,12,35,700,2,4,1e2,5,1e4,100,10000,1.0,2.0,1.0,false,1e-4);
uav_tracking.UAVTracking_Solver(uavParams);
}
//Testing the functionality from UAVTracking.h
#include <gtest/gtest.h>
#include "UAVTracking.h"

UAVTracking <double> uav_tracking;

TEST(MissionAttempt,InvalidParameters){
//Ensures that the tracking program won't start
UAVTracking_parameters uavParams(-1e-5,100,2,3,4,5,6,-2,10,3,8,20.0,0.0,-1e-2);
EXPECT_THROW({uav_tracking.UAVTracking_Solver(uavParams);},std::invalid_argument);
}

TEST(Solution,GuessInsideTheRadar){
//Testing the stability for the G-N strategy by checking valid maximal velocity and step-size
UAVTracking_parameters uavParams(1e-5,100,2,3,4,5,6,2,10,8,2,10,1e-2,1e-2);
uav_tracking.UAVTracking_Solver(uavParams);
}

TEST(Solution,GuessOutsideTheRadar){
//Testing the stability for the G-N strategy by checking valid maximal velocity and step-size
UAVTracking_parameters uavParams(1e-5,100,2,3,4,5,6,2,10,8,12,20,5*1e-1,1e-2);
uav_tracking.UAVTracking_Solver(uavParams);
}

TEST(FastUAVs,GuessInsideTheRadar){
//Testing the stability for the G-N strategy by checking valid maximal velocity and step-size. The trick is to choose a distance to the target
//that is far away when instability can occur.
UAVTracking_parameters uavParams(1e-5,100,2,15,32,12,3,2,1e8,5,1e7,100,1,1e-2);
EXPECT_THROW({uav_tracking.UAVTracking_Solver(uavParams);},std::runtime_error);
}

TEST(FastUAVs,GuessOutsideTheRadar){
//Testing the stability for the G-N strategy by checking valid maximal velocity and step-size. The trick is to choose a distance to the target
//that is far away when instability can occur.
UAVTracking_parameters uavParams(1e-5,100,2,6,8,12,13,2,1e7,1,1e8,200,0.9,1e-2);
EXPECT_THROW({uav_tracking.UAVTracking_Solver(uavParams);},std::runtime_error);
}
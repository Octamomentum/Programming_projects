//Testing the functionality for the radar system setup 
#include <gtest/gtest.h>
#include "generateRadarStructure.h"

generateRadarStructure <double> radar;

TEST(LandmarksInitialization,TooFewLandmarks){
//Testing if the number of landmarks are less than three
UAVTracking_parameters uavParams(1e-5,1,23,3,4,5,6,2,2,3,3,1.0,1e-3,1e-2);
auto [lm_seed,uav_ref_seed,uav_guess_seed,vel_seed] = radar.generateSeeds(uavParams);
EXPECT_THROW({radar.generateRadar(uavParams,lm_seed,uav_ref_seed);},std::invalid_argument);
}

TEST(GenerateSeeds,NegativeSeedNumbers){
//Testing the uniqueness of seeding (radar)
UAVTracking_parameters uavParams(1e-5,3,23,-6,8,-7,-5,2,2,3,3,1.0,1e-3,1e-2);
auto [lm_seed,uav_ref_seed,uav_guess_seed,vel_seed] = radar.generateSeeds(uavParams);
EXPECT_THROW({radar.generateRadar(uavParams,lm_seed,uav_ref_seed);},std::invalid_argument);
}

TEST(GenerateSeeds,InvalidRadialBoundary){
//Testing radial feasibility 
UAVTracking_parameters uavParams(1e-5,3,23,6,3,2,8,2,-1,3,3,1.0,1e-3,1e-2);
auto [lm_seed,uav_ref_seed,uav_guess_seed,vel_seed] = radar.generateSeeds(uavParams);
EXPECT_THROW({radar.generateRadar(uavParams,lm_seed,uav_ref_seed);},std::invalid_argument);
}

TEST(GenerateSeeds,InvalidGuessRadius){
//Testing radial feasibility 
UAVTracking_parameters uavParams(1e-5,3,23,6,3,2,8,2,100,3,-5,1.0,1e-3,1e-2);
auto [lm_seed,uav_ref_seed,uav_guess_seed,vel_seed] = radar.generateSeeds(uavParams);
EXPECT_THROW({radar.generateSensorCoordinates(uavParams,uav_guess_seed);},std::invalid_argument);
}

TEST(GenerateVelocity,InvalidMaximalVelocityMagnitude){
//Testing valid maximal velocity magnitude 
UAVTracking_parameters uavParams(1e-5,3,23,6,3,2,8,2,1,3,3,-1.0,1e-3,1e-2);
EXPECT_THROW({radar.generateGlobalEnvironment(uavParams);},std::invalid_argument);
}

TEST(GenerateRadarEnvironment,GuessInsideTheRadar){
//Testing working example for guessed coordinates inside the radar 
UAVTracking_parameters uavParams(1e-5,3,23,6,3,2,8,2,5,3,3,1.0,1e-3,1e-2);
auto [LandmarkMatrix, UAV, Velocities] = radar.generateGlobalEnvironment(uavParams);
}

TEST(GenerateRadarEnvironment,GuessOutsideTheRadar){
//Testing working example for guessed coordinates inside the radar 
UAVTracking_parameters uavParams(1e-5,3,23,6,3,2,8,2,5,3,9,1.0,1e-3,1e-2);
auto [LandmarkMatrix, UAV, Velocities] = radar.generateGlobalEnvironment(uavParams);
}
//Testing the functionality for the radar system setup 
#include <gtest/gtest.h>
#include "generateRadarStructure.h"
#include "UAVTracking_parameters.h"


generateRadarStructure <double> radar;

TEST(NegativeSeeds,InvalidSeedValues){
//Negative seed test
UAVTracking_parameters uavParams; 
uavParams.seeds.landmark_seedvalue = 6;
uavParams.seeds.uav_guess_seedvalue = -8;
uavParams.seeds.uav_truth_seedvalue = 1;
uavParams.seeds.initial_velocity_seedvalue = 2;
uavParams.seeds.initial_wind_velocity_seedvalue = 3;
uavParams.seeds.variance_distances = 9;
EXPECT_THROW({radar.generateSeeds(uavParams);},std::invalid_argument);
}

TEST(LandmarksInitialization,TooFewLandmarks){
//Testing if the number of landmarks are less than three
UAVTracking_parameters uavParams; 
uavParams.dimension.num_of_landmarks = 2;
uavParams.dimension.num_of_uavs = 1000;
uavParams.radii.radar_radius = 10.0;
EXPECT_THROW({radar.generateSeeds(uavParams);},std::invalid_argument);
}

TEST(UAVsInitializations,TooFewUAVs){
//Testing if the number of landmarks are less than three
UAVTracking_parameters uavParams; 
uavParams.dimension.num_of_landmarks = 2;
uavParams.dimension.num_of_uavs = -1000;
uavParams.radii.radar_radius = 10.0;
EXPECT_THROW({radar.generateSeeds(uavParams);},std::invalid_argument);
}

TEST(GenerateRadar,negativeRadii){
//Testing if any of the radii is negative for the radar field
UAVTracking_parameters uavParams; 
uavParams.dimension.num_of_landmarks = 10;
uavParams.dimension.num_of_uavs = 1e4;
uavParams.options.max_iterations = 50;
uavParams.radii.radar_radius = -10;
uavParams.radii.uav_guess_radius = 20;
auto [lm_seed,uav_truth_seed,uav_guess_seed,vel_uav_seed,vel_wind_seed,variance_distance_seed] = radar.generateSeeds(uavParams);
EXPECT_THROW({radar.generateRadar(uavParams,lm_seed,uav_truth_seed,uav_guess_seed);},std::invalid_argument);
}

TEST(GenerateRadarSystem,AWorkingExample){
//Testing a successful generated radar environment
UAVTracking_parameters uavParams; 
uavParams.dimension.num_of_landmarks = 150;
uavParams.dimension.num_of_uavs = 1000;
uavParams.options.max_iterations = 50;
uavParams.seeds.landmark_seedvalue = 6;
uavParams.seeds.uav_guess_seedvalue = 8;
uavParams.seeds.uav_truth_seedvalue = 1;
uavParams.seeds.initial_velocity_seedvalue = 2;
uavParams.seeds.initial_wind_velocity_seedvalue = 3;
uavParams.seeds.variance_distances = 9;
uavParams.radii.radar_radius = 100;
uavParams.radii.uav_guess_radius = 1e5;
auto [LandmarkMatrix,UAV,VelocitySeeds,VarianceSeeds] = radar.generateGlobalEnvironment(uavParams);
}
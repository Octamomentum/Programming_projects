//Testing the functionality for the radar system setup 
#include <gtest/gtest.h>
#include "generateRadarStructure.h"
#include "UAVTracking_parameters.h"


generateRadarStructure <double> radar;

TEST(NegativeSeeds,InvalidSeedValues){
//Negative seed test
UAVTracking_parameters uavParams; 
uavParams.seeds.landmark_seedvalue = 6;
uavParams.seeds.uav_seedvalue = -8;
uavParams.seeds.radar_anchors_seedvalue = -3232;
uavParams.seeds.variance_seedvalue = 7;
EXPECT_THROW({radar.generateRadar(uavParams);},std::invalid_argument);
}

TEST(LandmarksInitialization,TooFewLandmarks){
//Testing if the number of landmarks are less than three
UAVTracking_parameters uavParams; 
uavParams.dimension.num_of_landmarks = 2;
uavParams.dimension.num_of_uavs = 1000;
uavParams.posParams.radar_radius = 10.0;
EXPECT_THROW({radar.generateRadar(uavParams);},std::invalid_argument);
}

TEST(UAVsInitializations,TooFewUAVs){
//Testing if the number of UAVs is valid
UAVTracking_parameters uavParams; 
uavParams.dimension.num_of_landmarks = 2;
uavParams.dimension.num_of_uavs = -1000;
uavParams.posParams.radar_radius = 10.0;
EXPECT_THROW({radar.generateRadar(uavParams);},std::invalid_argument);
}

TEST(GenerateRadar,negativeposParams){
//Testing if any of the relevant posParams parameters is negative for the radar domain construction
UAVTracking_parameters uavParams; 
uavParams.dimension.num_of_landmarks = 10;
uavParams.dimension.num_of_uavs = 1e4;
uavParams.posParams.radar_radius = -10;
uavParams.posParams.uav_guess_radius = 50;
EXPECT_THROW({radar.generateRadar(uavParams);},std::invalid_argument);
}

TEST(GenerateRadarSystem,AWorkingExample){
//Testing a successful generated radar environment
UAVTracking_parameters uavParams; 
uavParams.dimension.num_of_landmarks = 150;
uavParams.dimension.num_of_uavs = 2;
uavParams.seeds.landmark_seedvalue = 12;
uavParams.seeds.uav_seedvalue = 13;
uavParams.seeds.radar_anchors_seedvalue = 13;
uavParams.seeds.variance_seedvalue = 23;
uavParams.posParams.radar_radius = 100;
uavParams.posParams.uav_guess_radius = 1000;
auto [LandmarkMatrix,UAV,RadarAnchors] = radar.generateGlobalEnvironment(uavParams);
}
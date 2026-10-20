//Testing the functionality from UAVTracking.h
#include <gtest/gtest.h>
#include "PhysicalConstraints.h"

PhysicalConstraints <double> phys_const;
UAVTracking_parameters uavParams;

TEST(KinematicPrediction,TestExample){
//Testing the kinematic prediction for a fleet of one UAV given a velocity
UAVTracking_parameters uavParams; 
uavParams.dynamical_UAV_params.h = 1e-1;
uavParams.posParams.radar_radius = 4.0;
Matrix<double> uav(3,1);
Matrix<double> vel_vec(3,1);

uav(0,0) = 0.5;
uav(1,0) = 0.5;
uav(2,0) = -0.5;

vel_vec(0,0) = 1/std::sqrt(2);
vel_vec(1,0) = 1/std::sqrt(2);
vel_vec(2,0) = 0.0;

bool MakeAReverseMovement = phys_const.MovementDecision(uavParams,uav,vel_vec);
EXPECT_TRUE(MakeAReverseMovement == false);

}
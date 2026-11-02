//Testing the functionality from StateEstimator.h

#include "StateEstimator.h"
#include "UAVTracking_parameters.h"
#include <gtest/gtest.h>
#include <iostream>

StateEstimator<double> state_est;

TEST(StateEstimator,GaussNewton){
//Gauss-Newton test
UAVTracking_parameters uavParams;
uavParams.dimension.num_of_landmarks = 3;
uavParams.dimension.num_of_uavs = 1;
uavParams.est.eps = 1e-7;
uavParams.options.activate_method = "G-N";
uavParams.options.applyWGN = false;
Matrix<double> LandmarkMatrix(3,3);
Matrix<double> UAV(1,3);
Matrix<double> distRadar(3,1);
Matrix<double> P(3*uavParams.dimension.num_of_uavs,3*uavParams.dimension.num_of_uavs);
Matrix<size_t> varianceSeed(1,1);
size_t iter = 0;
auto varianceParams = std::make_tuple(varianceSeed,iter);
double lambda = 0.0;

LandmarkMatrix(0,0) = 1.0;
LandmarkMatrix(0,1) = -1.0;
LandmarkMatrix(0,2) = 0.0;
LandmarkMatrix(1,0) = 0.0;
LandmarkMatrix(1,1) = 1.0;
LandmarkMatrix(1,2) = 2.0;
LandmarkMatrix(2,0) = 1.0;
LandmarkMatrix(2,1) = 1.0;
LandmarkMatrix(2,2) = 0.0;

UAV(0,0) = 1.0;
UAV(0,1) = 1.0;
UAV(0,2) = 1.0;

distRadar(0,0) = 3.0;
distRadar(1,0) = 2.0;
distRadar(2,0) = 5.0;


TrackingParameters trackingParams{LandmarkMatrix,UAV,P,iter,lambda};

auto[dir_vec_normval,res_normval] = state_est.State_Estimator(uavParams,trackingParams,distRadar,trackingParams.iter);
std::cout << "Norm, dir vec: " << dir_vec_normval;
std::cout << "Norm, residual: " << res_normval;
Matrix<double> x_est(1,3);

x_est(0,0) = 3 + 2*std::sqrt(2);
x_est(0,1) = 0.5*(3*std::sqrt(5) - 7);
x_est(0,2) = 5;

std::cout << "\nSolution:\n";
UAV.print();
std::cout << "\nTest solution:\n";
x_est.print();

EXPECT_EQ(x_est,UAV);

}

TEST(StateEstimator,LevenbergMarquardt){
//Levenberg-Marquardt test
UAVTracking_parameters uavParams;
uavParams.dimension.num_of_landmarks = 3;
uavParams.dimension.num_of_uavs = 1;
uavParams.est.eps = 1e-7;
uavParams.est.lambda_threshold = 1e-2;
uavParams.est.lambda_tuning_factor = 2.0;
uavParams.est.rho_threshold = 1e-7;
uavParams.est.tau = 1e-12;
uavParams.options.activate_method = "L-M";
uavParams.options.applyWGN = false;
Matrix<double> LandmarkMatrix(3,3);
Matrix<double> UAV(1,3);
Matrix<double> distRadar(3,1);
Matrix<double> P(3*uavParams.dimension.num_of_uavs,3*uavParams.dimension.num_of_uavs);
Matrix<size_t> varianceSeed(1,1);
size_t iter = 0;
auto varianceParams = std::make_tuple(varianceSeed,iter);
double lambda = 0.0;

LandmarkMatrix(0,0) = 1.0;
LandmarkMatrix(0,1) = -1.0;
LandmarkMatrix(0,2) = 0.0;
LandmarkMatrix(1,0) = 0.0;
LandmarkMatrix(1,1) = 1.0;
LandmarkMatrix(1,2) = 2.0;
LandmarkMatrix(2,0) = 1.0;
LandmarkMatrix(2,1) = 1.0;
LandmarkMatrix(2,2) = 0.0;

UAV(0,0) = 1.0;
UAV(0,1) = 1.0;
UAV(0,2) = 1.0;

distRadar(0,0) = 3.0;
distRadar(1,0) = 2.0;
distRadar(2,0) = 5.0;


TrackingParameters trackingParams{LandmarkMatrix,UAV,P,iter,lambda};

auto[dir_vec_normval,res_normval] = state_est.State_Estimator(uavParams,trackingParams,distRadar,trackingParams.iter);
std::cout << "Norm, dir vec: " << dir_vec_normval;
std::cout << "Norm, residual: " << res_normval;
Matrix<double> x_est(1,3);

x_est(0,0) = 1.0074;
x_est(0,1) = 1.01085;
x_est(0,2) = 1.06384;

std::cout << "\nSolution:\n";
UAV.print();
std::cout << "\nTest solution:\n";
x_est.print();

EXPECT_EQ(x_est,UAV);

}

TEST(StateEstimator,KalmanFilter){
//Kalman filter test
UAVTracking_parameters uavParams;
uavParams.dimension.num_of_landmarks = 3;
uavParams.dimension.num_of_uavs = 1;
uavParams.est.eps = 1e-7;
uavParams.est.tau = 1e-12;
uavParams.est.gamma = 1e-4;
uavParams.dynamical_UAV_params.h = 1.0;
uavParams.dynamical_UAV_params.uav_velocity = 30.0;
uavParams.dynamical_UAV_params.wind_velocity = 15.0;
uavParams.options.activate_method = "Kalman";
uavParams.options.applyWGN = false;
Matrix<double> LandmarkMatrix(3,3);
Matrix<double> UAV(1,3);
Matrix<double> distRadar(3,1);
Matrix<double> P(3*uavParams.dimension.num_of_uavs,3*uavParams.dimension.num_of_uavs);
uavParams.posParams.maximal_offset = 10.0;
for(size_t i = 0;i < P.getRows();i++){
   P(i,i) = uavParams.posParams.maximal_offset*uavParams.posParams.maximal_offset; 
}
Matrix<size_t> varianceSeed(1,1);
size_t iter = 0;
auto varianceParams = std::make_tuple(varianceSeed,iter);
double lambda = 0.0;

LandmarkMatrix(0,0) = 1.0;
LandmarkMatrix(0,1) = -1.0;
LandmarkMatrix(0,2) = 0.0;
LandmarkMatrix(1,0) = 0.0;
LandmarkMatrix(1,1) = 1.0;
LandmarkMatrix(1,2) = 2.0;
LandmarkMatrix(2,0) = 1.0;
LandmarkMatrix(2,1) = 1.0;
LandmarkMatrix(2,2) = 0.0;

UAV(0,0) = 1.0;
UAV(0,1) = 1.0;
UAV(0,2) = 1.0;

distRadar(0,0) = 3.0;
distRadar(1,0) = 2.0;
distRadar(2,0) = 5.0;

TrackingParameters trackingParams{LandmarkMatrix,UAV,P,iter,lambda};

auto[dir_vec_normval,res_normval] = state_est.State_Estimator(uavParams,trackingParams,distRadar,trackingParams.iter);
std::cout << "Norm, dir vec: " << dir_vec_normval;
std::cout << "Norm, residual: " << res_normval;
Matrix<double> x_est(1,3);

x_est(0,0) = 3 + 2*std::sqrt(2);
x_est(0,1) = 0.5*(3*std::sqrt(5) - 7);
x_est(0,2) = 5;

std::cout << "\nSolution:\n";
UAV.print();
std::cout << "\nTest solution:\n";
x_est.print();

EXPECT_EQ(x_est,UAV);

}

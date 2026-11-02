/**
 * @file TrackingParameters.h
 * @brief A header-file that passes through tracking-related parameters between UAVTracking and StateEstimator for each state.
 
 * */ 

#pragma once
#include "MatrixObject.h"

//Matrix objects and lambda passed by reference
struct TrackingParameters{
//Landmark matrix
MatrixObject<double>& LandmarkMatrix;
//Matrix consisting of UAV coordinates row-wise
MatrixObject<double>& UAV;
//Covariance matrix P
MatrixObject<double>& P;
//Iteration
size_t& iter;
//Used for regularizing the UAV Tracking problem if Levenberg-Marquardt method is used
double& lambda;
};
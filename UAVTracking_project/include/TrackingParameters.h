/**
 * @file TrackingParameters.h
 * @brief A header-file that passes through tracking-related parameters between UAVTracking and StateEstimator for each state.
 
 * */ 

#pragma once
#include "MatrixObject.h"

struct TrackingParameters{
//Matrix objects and lambda passed by reference
//Landmark matrix
MatrixObject<double>& mat;
//Matrix structures for ground truth respective guess
MatrixObject<double>& UAVTruth;
MatrixObject<double>& UAVGuess;
//Covariance matrix P
MatrixObject<double>& P;
//Used for regularizing the UAV Tracking problem if Levenberg-Marquardt method is used
double& lambda;
};
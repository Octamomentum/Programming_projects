/**
 * @file TrackingParameters.h
 * @brief A header-file that passes through tracking-related parameters between UAVTracking and StateEstimator for each state.
 
 * */ 

#pragma once
#include "MatrixObject.h"

struct TrackingParameters{
//Matrix objects and lambda passed by reference
MatrixObject<double>& mat;
MatrixObject<double>& UAVTruth;
MatrixObject<double>& UAVGuess;
MatrixObject<double>& P;
double& lambda;
};
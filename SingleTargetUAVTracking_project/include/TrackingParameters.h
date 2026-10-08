/**
 * @file TrackingParameters.h
 * @brief A header-file that passes through tracking-related parameters between UAVTracking and MatrixCalculations for each state.
 
 * */ 

#pragma once
#include "MatrixObject.h"

struct TrackingParameters{
//Matrix objects and lambda passed by reference
MatrixObject<double>& mat;
MatrixObject<double>& x_truth;
MatrixObject<double>& x_guess;
double& lambda;
};
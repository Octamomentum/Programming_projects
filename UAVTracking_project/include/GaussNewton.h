/**
 * @file GaussNewton.h
 * @brief A state estimator based on using the Gauss-Newton strategy.
 * 
 * 
 * */ 

#pragma once
#include "MatrixCalculations.h"
#include "TrackingParameters.h"

template <typename T>
class GaussNewton {

public:

MatrixCalculations<T> matcalc;

//Gauss-Newton method
Matrix<T> GaussNewton_Approach(const Matrix<T>& R, const Matrix<T>& b_tilde)const{

 //Generate Matrix form and truncate to form an upper triangular system of equations
 auto [R_filtered,b_tilde_filtered] = matcalc.generateTruncatedMatrices(R,b_tilde);
 //Solve the upper triangular system of equations
 Matrix<T> dir_vec = matcalc.BackwardSubstitution(R_filtered,b_tilde_filtered);

return dir_vec;
}

};

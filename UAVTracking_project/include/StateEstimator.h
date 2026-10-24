/**
 * @file StateEstimator.h
 * @brief The main class for using a desired numerical estimation method and optional choice to generate WGN to the UAV Tracking problem.
  
  *Three strategies are used in this class:
   Gauss-Newton, Levenberg-Marquardt and an approach using Kalman filtering techniques. Their implementations can be found in their unique classes.
 * 
 * */ 

#pragma once
#include "MatrixCalculations.h"
#include "GaussNewton.h"
#include "LevenbergMarquardt.h"
#include "KalmanFilter.h"
#include "TrackingParameters.h"

template <typename T>
class StateEstimator{

private:

MatrixCalculations<T> matcalc;
GaussNewton<T> gn_solver;
LevenbergMarquardt<T> lm_solver;
KalmanFilter<T> kf_solver;

public:

struct EstOutput{
T dir_vec_normval = 0.0;
T res_normval = 0.0;
};

//Method that retrieves the first lambda by default w.r.t. the parameter tau, treated as an initialized regularization term for the L-M method

T initializeLambda(const Matrix<T>& A, const T& tau)const{

Matrix<T> potential_lambda(A.getCols(),1);
T lambda = 0.0;

for(size_t i = 0;i < A.getCols();i++){
   Matrix<T> col_vec = A.getColumn(i);
   T summation = 0.0;
   for(size_t j = 0;j < col_vec.getRows();j++){
      summation += matcalc.square(col_vec(j,0));
   }
   potential_lambda(i,0) = summation;
}
lambda = tau*matcalc.maximal_vector_element(potential_lambda);

return lambda;   
}

//The numerical estimation solver for the UAV Tracking problem: it either uses Gauss-Newton, Levenberg-Marquardt or an approach using Kalman filtering
//techniques
auto State_Estimator(const UAVTracking_parameters& uavParams, TrackingParameters& trackingParams,
  const std::tuple<Matrix<size_t>,size_t>& varianceParams)const{
 
    //Checking if the chosen method is wrong
if(uavParams.options.activate_method != "G-N" && uavParams.options.activate_method != "L-M" && uavParams.options.activate_method != "Kalman"){
  throw std::invalid_argument("Wrong method.");
}
T dir_vec_normval = 0.0;
T res_normval = 0.0;  Matrix<T> dir_vec(3*uavParams.dimension.num_of_uavs,1);
  Matrix<T> trial_vec(uavParams.dimension.num_of_uavs,3);
  auto [J,r] = matcalc.GenerateMatrixData(uavParams,trackingParams,trackingParams.UAVGuess,varianceParams);
   auto [R,b_tilde] = matcalc.QRDecomposition_GivensApproach(J,r,uavParams.est.eps); 
   
//Initialize damping
size_t iter = std::get<1>(varianceParams);
if(iter == 0){
  trackingParams.lambda = initializeLambda(J,uavParams.est.tau); 
}
  //Perform G-N if such option is chosen
  if(uavParams.options.activate_method == "G-N"){
    dir_vec = gn_solver.GaussNewton_Approach(R,b_tilde);
    }

//Perform L-M if the option is chosen
if(uavParams.options.activate_method == "L-M"){
   dir_vec = lm_solver.LevenbergMarquardt_Approach(uavParams,trackingParams,R,b_tilde);
}

//Perform Kalman filtering if the option is chosen
if(uavParams.options.activate_method == "Kalman"){
dir_vec = kf_solver.KalmanFilter_Approach(uavParams,trackingParams,R,b_tilde);
}

 dir_vec_normval = matcalc.L2Norm(dir_vec);
 res_normval = matcalc.L2Norm(r);
 Matrix<T> transformedSolution = matcalc.transformVectorToMatrix(dir_vec,uavParams.dimension.num_of_uavs);
 trial_vec = transformedSolution + trackingParams.UAVGuess;   
 trackingParams.UAVGuess = trial_vec;
return std::make_tuple(dir_vec_normval,res_normval);

}

};

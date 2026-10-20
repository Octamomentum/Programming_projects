/**
 * @file StateEstimator.h
 * @brief The main class for using a desired numerical estimation method and optional choice to generate WGN to the UAV Tracking problem.
  
  *Three approaches are used in this class:
   Gauss-Newton, Levenberg-Marquardt and an approach using Kalman filtering techniques. Their implementations can be found in their unique classes.
 * 
 * */ 

#pragma once
#include "MatrixCalculations.h"
#include "GaussNewton.h"
#include "LevenbergMarquardt.h"
#include "Kalmanfilter.h"
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

//Method that uses Armijo conditioned line search for the data assimilation problem

T ArmijoLineSearch(const UAVTracking_parameters& uavParams, TrackingParameters& trackingParams, const Matrix<T>& dir_vec,
    const std::tuple<T,T>& Gradientparams, const std::tuple<Matrix<size_t>,size_t>& varianceParams)const{

T res = std::get<0>(Gradientparams);
T norm_value = std::get<1>(Gradientparams);
T alpha = 1.0;
const T alpha_threshold = 1e-4;
bool LineSearchStillGoing = true;
//Perform line search until a decreased cost occur
while(LineSearchStillGoing){
    Matrix<T> scaled_term = dir_vec.scalarMatrix(alpha);
    Matrix<T> transformedSolution = matcalc.transformVectorToMatrix(dir_vec,uavParams.dimension.num_of_uavs);
    Matrix<T> trial_vec = trackingParams.UAVGuess + transformedSolution;
    auto[J_new,r_new] = matcalc.GenerateMatrixData(uavParams,trackingParams,trial_vec,varianceParams);
    if(0.5*matcalc.square(matcalc.L2Norm(r_new)) <= 0.5*matcalc.square(res) - uavParams.est.c_param*alpha*norm_value){
      LineSearchStillGoing = false;
    }
    else{
       alpha *= uavParams.est.beta;
       std::cout<<"\nAlpha: " << alpha << "\n";

    }
    if(alpha < alpha_threshold){
      alpha = alpha_threshold;
      LineSearchStillGoing = false;

    }
}
return alpha;
}

//The numerical estimation solver for the UAV Tracking problem: it either uses Gauss-Newton, Levenberg-Marquardt or an approach using Kalman filtering
//techniques
auto State_Estimator(const UAVTracking_parameters& uavParams, TrackingParameters& trackingParams,
const std::tuple<Matrix<size_t>,size_t>& varianceParams)const{
  
  EstOutput est_output;
  Matrix<T> dir_vec(3*uavParams.dimension.num_of_uavs,1);
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
    dir_vec = gn_solver.GaussNewton_Approach(uavParams,trackingParams,R,b_tilde);
    }

//Perform L-M if the option is chosen
if(uavParams.options.activate_method == "L-M"){
   dir_vec = lm_solver.LevenbergMarquardt_Approach(uavParams,trackingParams,R,b_tilde);
}

//Perform Kalman filtering if the option is chosen
if(uavParams.options.activate_method == "Kalman"){
dir_vec = kf_solver.KalmanFilter_Approach(uavParams,trackingParams,J,r);
}

est_output.dir_vec_normval = matcalc.L2Norm(dir_vec);
 est_output.res_normval = matcalc.L2Norm(r);
 Matrix<T> grad_term = J*dir_vec;
 T norm_term = matcalc.square(matcalc.L2Norm(grad_term));
 auto Gradientparams = std::make_tuple(est_output.res_normval,norm_term);
 //Perform Line Search based on Armijo condition
 T alpha = ArmijoLineSearch(uavParams,trackingParams,dir_vec,Gradientparams,varianceParams);
 Matrix<T> scaled_term = dir_vec.scalarMatrix(alpha);
 Matrix<T> transformedSolution = matcalc.transformVectorToMatrix(dir_vec,uavParams.dimension.num_of_uavs);
 trial_vec = transformedSolution + trackingParams.UAVGuess;   
 trackingParams.UAVGuess = trial_vec;
return est_output;

}

};

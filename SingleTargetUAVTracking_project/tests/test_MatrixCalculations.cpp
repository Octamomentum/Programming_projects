//Testing the functionality from MatrixCalculations.h

#include "MatrixCalculations.h"
#include "UAVTracking_parameters.h"
#include <gtest/gtest.h>
#include <iostream>

MatrixCalculations<double> matcalc;

TEST(LeastSquare,StandardOperations){
//Testing standard operations
UAVTracking_parameters uavParams;
MatrixObject<double> J(4,3);
uavParams.least_square.tau = 1e-2;
double val = 0.5;
//Square function
EXPECT_DOUBLE_EQ(matcalc.square(val),0.25);

MatrixObject<double> vec1(3,1);
MatrixObject<double> vec2(3,1);

J(0,0) = 0.0;
J(0,1) = 1.0;
J(0,2) = 2.0;
J(1,0) = 1.0;
J(1,1) = 2.0;
J(1,2) = 1.0;
J(2,0) = -1.0;
J(2,1) = 1.0;
J(2,2) = 1.0;
J(3,0) = 0.0;
J(3,1) = -2.0;
J(3,2) = 0.0;

//Initialize lambda based on Jacobian matrix J and tau
double lambda = matcalc.initializeLambda(J,uavParams.least_square.tau);
EXPECT_DOUBLE_EQ(lambda,1e-1);


vec1(0,0) = 1.0;
vec1(1,0) = -2.0;
vec1(2,0) = 2.0;

vec2(0,0) = -1.0;
vec2(1,0) = 2.0;
vec2(2,0) = -5.0;

//Verifying that the L2-norm function calculates correctly
EXPECT_DOUBLE_EQ(matcalc.L2Norm(vec1),3.0);

//Test dot product calculation
EXPECT_DOUBLE_EQ(matcalc.dot_product(vec1,vec2),-15.0);

double pivot_val = 2.0;
double targeted_val = 5.0;
//Givens parameters based on row elements for a certain column
auto [c_qr,s_qr] = matcalc.GivensParameters_QR(pivot_val,targeted_val);
//Checking the trigonometric terms for an obvious example
EXPECT_DOUBLE_EQ(c_qr,2/std::sqrt(29));
EXPECT_DOUBLE_EQ(s_qr,5/std::sqrt(29));

}

TEST(LeastSquare,QRExampleGaussNewton){
//A general example solving a least square problem using the Gauss-Newton idea
MatrixObject<double> A(3,2);
MatrixObject<double> b(3,1);
UAVTracking_parameters uavParams;
uavParams.least_square.eps = 1e-7;
uavParams.least_square.tau = 1e-7;

A(0,0) = 1.0;
A(0,1) = 1.0;
A(1,0) = 1.0;
A(1,1) = 2.0;
A(2,0) = 0.0;
A(2,1) = -1.0;

b(0,0) = -1.0;
b(1,0) = 1.0;
b(2,0) = 0.0;

double lambda = matcalc.initializeLambda(A,uavParams.least_square.tau);

auto [R,b_tilde] = matcalc.QRDecomposition_GivensApproach(A,b,uavParams.least_square.eps);

MatrixObject<double> R_test(3,2);
MatrixObject<double> btilde_test(3,1);

R.print();
b_tilde.print();

R_test(0,0) = 1.4142;
R_test(0,1) = 2.1213;
R_test(1,0) = 0.0;
R_test(1,1) = 1.2247;
R_test(2,0) = 0.0;
R_test(2,1) = 0.0;

btilde_test(0,0) = 0.0;
btilde_test(1,0) = 0.8165;
btilde_test(2,0) = 1.1547;

EXPECT_EQ(R,R_test);
EXPECT_EQ(b_tilde,btilde_test);

//Solve
auto [R_filtered,btilde_filtered] = matcalc.generateTruncatedMatrices(uavParams,R,b_tilde,lambda);

MatrixObject<double> Rfiltered_test(2,2);
MatrixObject<double> btildefiltered_test(2,1);

Rfiltered_test(0,0) = R_test(0,0);
Rfiltered_test(0,1) = R_test(0,1);
Rfiltered_test(1,0) = R_test(1,0);
Rfiltered_test(1,1) = R_test(1,1);

btildefiltered_test(0,0) = btilde_test(0,0);
btildefiltered_test(1,0) = btilde_test(1,0);

EXPECT_EQ(Rfiltered_test,R_filtered);
EXPECT_EQ(btildefiltered_test,btilde_filtered);

MatrixObject<double> x_sol = matcalc.BackwardSubstitution(R_filtered,btilde_filtered);

x_sol.print();

MatrixObject<double> x_test(2,1);

x_test(0,0) = -1.0;
x_test(1,0) = 0.6667;

EXPECT_EQ(x_test,x_sol);
}

TEST(LeastSquare,QRExampleLevenbergMarquardt){
//A general example solving a least square problem using the Levenberg-Marquardt idea
MatrixObject<double> A(3,2);
MatrixObject<double> b(3,1);
UAVTracking_parameters uavParams;
uavParams.least_square.eps = 1e-7;
uavParams.least_square.tau = 1e-12;

A(0,0) = 1.0;
A(0,1) = 1.0;
A(1,0) = 1.0;
A(1,1) = 2.0;
A(2,0) = 0.0;
A(2,1) = -1.0;

b(0,0) = -1.0;
b(1,0) = 1.0;
b(2,0) = 0.0;

double lambda = matcalc.initializeLambda(A,uavParams.least_square.tau);

auto [R,b_tilde] = matcalc.QRDecomposition_GivensApproach(A,b,uavParams.least_square.eps);
std::cout << "Lambda: " << lambda;
auto [A_new,b_new] = matcalc.generateMatrixForm(R,b_tilde,lambda,"L-M");
auto [R_new,btilde_new] = matcalc.QRDecomposition_GivensApproach(A_new,b_new,uavParams.least_square.eps);
auto [R_filtered,btilde_filtered] = matcalc.generateTruncatedMatrices(uavParams,R_new,btilde_new,lambda);
MatrixObject<double> x_sol = matcalc.BackwardSubstitution(R_filtered,btilde_filtered);

MatrixObject<double> x_test(2,1);

x_test(0,0) = -1.0;
x_test(1,0) = 0.6667;

EXPECT_EQ(x_test,x_sol);
}
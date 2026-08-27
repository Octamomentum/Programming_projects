//Testing the functionality from MatrixCalculations.h

#include "MatrixCalculations.h"
#include <gtest/gtest.h>
#include <iostream>

MatrixCalculations<double> matcalc;

TEST(LeastSquare,StandardOperations){
double val = 0.5;
EXPECT_DOUBLE_EQ(matcalc.square(val),0.25);
MatrixObject<double> vec1(3,1);
MatrixObject<double> vec2(3,1);

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
auto [c_qr,s_qr] = matcalc.GivensParameters_QR(pivot_val,targeted_val);
//Checking the trigonometric terms for an obvious example
EXPECT_DOUBLE_EQ(c_qr,2/std::sqrt(29));
EXPECT_DOUBLE_EQ(s_qr,5/std::sqrt(29));
}

TEST(LeastSquare,LinearDependentExample){
//Testing the linear depedency example with a regularization parameter lambda for solving a least square problem
MatrixObject<double> A(3,2);
MatrixObject<double> b(3,1);
double eps = 1e-7;
double lambda = 1e-2;

A(0,0) = 1.0;
A(0,1) = -1.0;
A(1,0) = 1.0;
A(1,1) = -1.0;
A(2,0) = 1.0;
A(2,1) = -1.0;

b(0,0) = -2e52;
b(1,0) = 131.0;
b(2,0) = 5.0;


auto [R,b_tilde] = matcalc.QRDecomposition_GivensApproach(A,b,eps);
auto [R_filtered,btilde_filtered] = matcalc.generateTruncatedMatrices(R,b_tilde,eps,lambda);
MatrixObject<double> x_sol = matcalc.BackwardSubstitution(R_filtered,btilde_filtered);
x_sol.print();
}

TEST(LeastSquare,QRExample){
//A general example solving a least square problem using QR-decomposition
MatrixObject<double> A(3,2);
MatrixObject<double> b(3,1);
double eps = 1e-7;
double lambda = 0.0;

A(0,0) = 1.0;
A(0,1) = 1.0;
A(1,0) = 1.0;
A(1,1) = 2.0;
A(2,0) = 0.0;
A(2,1) = -1.0;

b(0,0) = -1.0;
b(1,0) = 1.0;
b(2,0) = 0.0;
auto [R,b_tilde] = matcalc.QRDecomposition_GivensApproach(A,b,eps);

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

auto [R_filtered,btilde_filtered] = matcalc.generateTruncatedMatrices(R,b_tilde,eps,lambda);

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

TEST(LeastSquare,GaussNewton){
//A test that performs the Gauss-Newton based least square problem and returns a solution and the residual distances relative to the landmarks
MatrixObject<double> L(4,3);
MatrixObject<double> x_ref(3,1);
MatrixObject<double> x_init(3,1);
double eps = 1e-7;
double lambda = 1e-2;

L(0,0) = 1.0;
L(0,1) = -1.0;
L(0,2) = 1.0;
L(1,0) = 0.0;
L(1,1) = 2.0;
L(1,2) = 4.0;
L(2,0) = -1.0;
L(2,1) = 0.0;
L(2,2) = 3.0;
L(3,0) = 4.0;
L(3,1) = 3.0;
L(3,2) = 0.0;

x_ref(0,0) = 1.0;
x_ref(1,0) = 2.0;
x_ref(2,0) = 4.0;
x_init(0,0) = 7.0;
x_init(1,0) = -8.0;
x_init(2,0) = 10.0;

auto [x_L2,b] = matcalc.LeastSquareSolver_GaussNewtonQR(L,x_ref,x_init,eps,lambda);
std::cout << "\nSolution: \n";
x_L2.print();
std::cout << "\n Norm: " << matcalc.L2Norm(x_L2) << "\n";
std::cout << "\nResidual distances relative to landmarks: \n";
b.print();
std::cout << "\n Norm: " << matcalc.L2Norm(b) << "\n";
}

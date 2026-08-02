#include <gtest/gtest.h>
#include "MatrixCalculations.h"
#include <iostream>
#include <tuple>

using namespace std;
MatrixCalculations<double> matcalc;

TEST(GivensCalc,Example){
   
   MatrixObject<double> A(4,3);
   A(0,0) = 1.0;
   A(0,1) = 1.0;
   A(0,2) = -1.0;
   A(1,0) = 2.0;
   A(1,1) = 1.0;
   A(1,2) = 1.0;
   A(2,0) = 1.0;
   A(2,1) = 1.0;
   A(2,2) = 1.0;
   A(3,0) = 0.0;
   A(3,1) = 1.0;
   A(3,2) = 1.0;

   auto [Q,R] = matcalc.QRDecomposition_GivensApproach(A);
   cout << "\nQ matrix:\n";
   Q.print();
   cout << "\nR matrix:\n";
   R.print();
   MatrixObject<double> A_QR = Q*R;
   MatrixObject<double> Q_T = Q.Transpose();
   EXPECT_EQ(Q_T*Q,matcalc.generateIdentityMatrix(Q.getRows()));

   MatrixObject<double> b(4,1);
   b(0,0) = 0.0;
   b(1,0) = 1.0;
   b(2,0) = 1.0;
   b(3,0) = 1.0;

   MatrixObject<double> x = matcalc.LeastSquareSolver_QRDecomposition(A,b);
   cout << "\nSolution x\n";
   x.print();

   MatrixObject<double> c = A*x;
   MatrixObject<double> v = c - b;
   double L2_normvalue = matcalc.L2Norm(v);
   cout << "L2 norm: " << L2_normvalue;
}


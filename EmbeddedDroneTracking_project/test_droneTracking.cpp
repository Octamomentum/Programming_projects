#include <gtest/gtest.h>
#include "MatrixCalculations.h"
#include <iostream>
#include <tuple>

using namespace std;
MatrixCalculations<double> matcalc;

TEST(GivensCalc,Example){
   MatrixObject<double> A(3,2);
   A(0,0) = 2.0;
   A(0,1) = 1.0;
   A(1,0) = 1.0;
   A(1,1) = 3.0;
   A(2,0) = 4.0;
   A(2,1) = 5.0;

   auto [Q,R] = matcalc.QRDecomposition_GivensApproach(A);
   Q.print();
   R.print();
   MatrixObject<double> A_QR = Q*R;
   A_QR.print();
}


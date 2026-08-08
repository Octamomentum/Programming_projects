#include <gtest/gtest.h>
#include "MatrixCalculations.h"
#include <iostream>
#include <tuple>

MatrixCalculations<double> matcalc;
using namespace std;

TEST(QRTest,Example){
    MatrixObject<double> A(3,2);
    A(0,0) = 1.0;
    A(0,1) = 1.0;
    A(1,0) = 0.0;
    A(1,1) = 1.0;
    A(2,0) = 1.0;
    A(2,1) = 2.0;

    auto [Q,R] = matcalc.QRDecomposition_GivensApproach(A);
    cout << "\nQ Matrix\n";
    Q.print();
    cout << "\nR Matrix\n";
    R.print();
    MatrixObject<double> A_QR = Q*R;
    EXPECT_EQ(A_QR,A);
    A_QR.print();
}


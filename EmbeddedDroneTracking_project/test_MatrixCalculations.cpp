#include <gtest/gtest.h>
#include "MatrixCalculations.h"
#include <iostream>
#include <tuple>

MatrixCalculations<double> matcalc;

using namespace std;

TEST(TrackOneUAV,Example){
    MatrixObject<double> A(3,2);
    MatrixObject<double> b(3,1);

    A(0,0) = -1.0;
    A(0,1) = 1.0;
    A(1,0) = 0.0;
    A(1,1) = 1.0;
    A(2,0) = 1.0;
    A(2,1) = 0.0;

    b(0,0) = 1.0;
    b(1,0) = 1.0;
    b(2,0) = 1.0;
     
    auto [Q,R] = matcalc.QRDecomposition_GivensApproach(A);
    cout << "\nQ\n";
    Q.print();
    cout << "\nR\n";
    R.print();
    MatrixObject<double> x = matcalc.LeastSquareSolver_QRDecomposition(A,b);
    cout << "\nx solution\n";
    x.print();



}


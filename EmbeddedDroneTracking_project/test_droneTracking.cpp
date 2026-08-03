#include <gtest/gtest.h>
#include "UAVTracking.h"
#include <iostream>
#include <tuple>

MatrixCalculations<double> matcalc;
UAVTracking<double,1,3> uav_tracking;

using namespace std;

TEST(DimensionalMap,generateInputs){
    tuple<double,double> map_range {-27,27};
    int l_seed = 12;
    int uav_seed = 15;
    MatrixObject<double> L = uav_tracking.generateLandmarks(l_seed,map_range);
    cout << "\nPRINTING L\n";
    L.print();    
    MatrixObject<double> x = uav_tracking.generate_x(uav_seed,map_range);
    cout << "\nPRINTING initial true coordinates x\n";
    x.print();
    auto[A,b] = uav_tracking.generateMatrixData(L,x,map_range); 
    cout << "\nPRINTING A\n";
    A.print();
    cout << "\nPRINTINT b\n";
    b.print();
}

TEST(SolveLeastSquareProblem,LSQRExample){
    tuple<double,double> map_range {-7,7};
    int l_seed = 5;
    int uav_seed = 25;
    MatrixObject<double> L = uav_tracking.generateLandmarks(l_seed,map_range);
    cout << "\nPRINTING L\n";
    L.print();    
    MatrixObject<double> x = uav_tracking.generate_x(uav_seed,map_range);
    cout << "\nPRINTING initial true coordinates x\n";
    x.print();
    auto[A,b] = uav_tracking.generateMatrixData(L,x,map_range); 
    cout << "\nPRINTING A\n";
    A.print();
    cout << "\nPRINTINT b\n";
    b.print();
    uav_tracking.OperationUAVTracking(A,b,x);
}


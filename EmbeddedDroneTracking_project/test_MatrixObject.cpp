#include "MatrixObject.h"
#include <iostream>

using namespace std;

int main(){

MatrixObject<double> A(2,2);
A(0,0) = 3.14;
A(0,1) = 25.7;
A(1,0) = 7.45;
A(1,1) = 75.3;

double c = 5.0;

A.print();

MatrixObject<double> B = A;
B.print();

MatrixObject<double> A_T = A.Transpose();
A_T.print();

MatrixObject<double> C = A_T * A;
C.print();

MatrixObject<double> D = A + B;
D.print();

MatrixObject<double> E = A - B;
E.print();

MatrixObject<double> scalarA = A.scalarMatrix(c);
scalarA.print();



return 0;
}
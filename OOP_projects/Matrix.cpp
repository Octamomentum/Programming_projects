#include "Matrix.h"
#include <iostream>
#include <cassert>
#include <cmath>
#include <ctime>
#include <functional>
using namespace std;

bool NormTest(double a, double b, double eps){
    cout << "Residual norm value:" << abs(a-b); 
    return abs(a-b) < eps; 
}

int main(){

Matrix <double> A(4,3);
Matrix <double> b(4,1);

A(0,0) = 1.0;
A(0,1) = 1.0;
A(0,2) = -1.0;
A(1,0) = 1.0;
A(1,1) = -1.0;
A(1,2) = 1.0;
A(2,0) = 1.0;
A(2,1) = 0.0;
A(2,2) = -1.0;
A(3,0) = 1.0;
A(3,1) = 1.0;
A(3,2) = 0.0;

b(0,0) = 0.0;
b(1,0) = 1.0;
b(2,0) = -1.0;
b(3,0) = 2.0;



cout << "A elements: \n";
A.print();
cout << "\nb elements: \n"; 
b.print();
cout << "\nA transpose elements: \n"; 
Matrix <double> A_t = A.Transpose();
A_t.print();
Matrix<double> C = A_t * A;
cout << "\nC elements: \n";
C.print();
Matrix<double> b_tilde = A_t * b;
cout << "\nb_tilde elements: \n";
b_tilde.print();
Matrix <double> L(C.getRows(), C.getColumns());
Matrix <double> U(C.getRows(), C.getColumns());
C.LU_decomposition(L,U);
cout << "\nL elements: \n";
L.print();
cout << "\nU elements: \n";
U.print();
Matrix <double> y(b_tilde.getRows(),1);
Matrix <double> x(y.getRows(),1);
L.ForwardSubstitution(y,b_tilde);
cout << "\ny elements: \n";
y.print();
U.BackwardSubstitution(x,y);
cout << "\nx elements: \n";
x.print();
double epsilon = A.L2norm(x,b);
cout << "\nL2-norm: " << epsilon << "\n";

assert(NormTest(epsilon,0.2357,1e-5));
cout << "\nAll tests passed.";


}

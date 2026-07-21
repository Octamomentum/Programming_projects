#include "MatrixCalcs.h"
#include <iostream>
#include <cassert>
using namespace std;


template <typename T>


bool isDatatypeIdentical(const MatrixCalcs<T>& M1, const MatrixCalcs<T>& M2, double eps){
if (M1.getRows() != M2.getRows() || M1.getColumns() != M2.getColumns()){
   cout << "This test failed based on the dimensionality.";
   return false;
}
for(int i = 0;i < M1.getRows(); i++){
    for(int j = 0; j < M1.getColumns();j++){
       double val = abs(M1(i,j) - M2(i,j));
       if(!(abs(val) < eps)){
         cout << "This test failed based on the relative error element-wise. Residual value: " << abs(val);
         return false;
       } 
    }
}
return true;
}  

int main(){

double eps = 1e-5;    
MatrixCalcs<double> A(3,3);
A(0,0) = 1.0;
A(0,1) = 1.0;
A(0,2) = 1.0;
A(1,0) = 0.0;
A(1,1) = 0.0;
A(1,2) = 1.0;
A(2,0) = 0.0;
A(2,1) = 1.0;
A(2,2) = 0.0;



A.print();

MatrixCalcs<double> row_vec(3,1);

row_vec(0,0) = 2.0;
row_vec(1,0) = 3.5;
row_vec(2,0) = -5.0;

MatrixCalcs<double> AddedRowToA = A.appendRowToMatrix(row_vec,3);

AddedRowToA.print();

MatrixCalcs<double> col_vec(4,1);

col_vec(0,0) = 5.0;
col_vec(1,0) = -2.5;
col_vec(2,0) = 1000;

MatrixCalcs<double> AddedColumnToA = A.appendColumnToMatrix(col_vec,3);

AddedColumnToA.print();
}
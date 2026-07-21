#include <iostream>
#include <cmath>
#include <math.h>
using namespace std;

template <typename T>

class MatrixCalcs{

private:

struct MatrixData{

int rows;
int cols;
T* MatrixPtr;    

};

MatrixData data;

public:

MatrixCalcs(int r, int c){

data.rows = r;
data.cols = c;
int number = data.rows*data.cols;
data.MatrixPtr = new T[number]{};    
}

MatrixCalcs(MatrixCalcs<T>& copied_mat){

data.rows = copied_mat.data.rows;
data.cols = copied_mat.data.cols;
int number = data.rows*data.cols;
data.MatrixPtr = new T[number]{};
for(int i = 0; i < number;i++){
    data.MatrixPtr[i] = copied_mat.data.MatrixPtr[i]; 
}    

}

~MatrixCalcs(){
delete[] data.MatrixPtr;
}

T& operator()(int r, int c){
return data.MatrixPtr[data.cols*r + c];    
}

const T& operator()(int r, int c) const{
return data.MatrixPtr[data.cols*r + c];    
}

int getRows() const{
return data.rows;    
}

int getColumns() const{
return data.cols;
}

MatrixCalcs<T> getRowVector(int r){
MatrixCalcs<T> row_vec(data.rows,1);
for(int j = 0;j < data.cols;j++){
   row_vec(j,0) = (*this)(r,j); 
}
return row_vec;
}

MatrixCalcs<T> getColumnVector(int c){
MatrixCalcs<T> col_vec(data.cols,1);
for(int i = 0;i < data.rows;i++){
   col_vec(i,0) = (*this)(i,c); 
}
return col_vec;
}

MatrixCalcs<T> appendRowToMatrix(MatrixCalcs<T>& row_vec, int r){
MatrixCalcs<T> newA(data.rows+1,data.cols);
for(int i = 0;i < data.rows;i++){
   for(int j = 0; j < data.cols;j++){
      newA(i,j) = (*this)(i,j);
   }
}

for(int j = 0;j < data.cols;j++){
   newA(r,j) = row_vec(j,0);
}

return newA;   
}

MatrixCalcs<T> appendColumnToMatrix(MatrixCalcs<T>& col_vec, int c){
MatrixCalcs<T> newA(data.rows,data.cols+1);
for(int i = 0;i < data.rows;i++){
   for(int j = 0; j < data.cols;j++){
      newA(i,j) = (*this)(i,j);
   }
}

for(int i = 0;i < data.rows;i++){
   newA(i,c) = col_vec(i,0);
}

return newA;   
}

MatrixCalcs<T> operator*(const MatrixCalcs<T>& A){
MatrixCalcs<T> C(data.rows,A.getColumns());
for(int i = 0;i < data.rows;i++){
   for(int k = 0;k < A.getColumns();k++){
      T summation = 0.0;
      for(int j = 0;j < data.cols;j++){
         summation += (*this)(i,j)*A(j,k);
      }
      C(i,k) = summation;
   } 
}
return C;
}

MatrixCalcs<T> Transpose(){
MatrixCalcs<T> A_T(data.cols,data.rows);
for(int i = 0;i < data.rows;i++){
    for(int j = 0; j < data.cols;j++){
       A_T(i,j) = (*this)(j,i); 
    }
}    
return A_T;
}

void print()const{
cout << "\n";
for(int i = 0;i < data.rows;i++){
    for(int j = 0; j < data.cols;j++){
        cout << "Matrix element (" << i << "," << j << "): " << (*this)(i,j) << "\n"; 
    }
}
}

double square(T x){
return x*x;    
}

double L2_norm(MatrixCalcs<T>& v){
T summation = 0.0;
for(int i = 0;i < data.rows;i++){
   summation += square((*this)(i,0));
}    
return sqrt(summation);    
}

double ScalarProduct(MatrixCalcs<T>& w,MatrixCalcs<T>& v){
double summation = 0.0;
for(int i = 0;i < data.rows;i++){
   summation += w(i,0)*v(i,0);
}
return summation;    
}

MatrixCalcs<T> GSOrthogonalization(){
MatrixCalcs<T> Q(data.rows,data.cols);

for(int k = 0;k < data.cols;k++){
   MatrixCalcs<T> vec = getColumnVector(k);
   if(k == 0){
     double norm_term = L2_norm(vec);
     for(int i = 0;i < data.rows;i++){
        Q(i,k) = vec(i,0)/norm_term;
     }
   }
   else{
      for(int j = 0; j < k; j++){
        double dp = 0.0;
         MatrixCalcs<T> temp_vec = Q.getColumnVector(j);
         dp += ScalarProduct(temp_vec,vec);
         for(int i = 0;i < data.rows;i++){
            vec(i,0) -= dp*temp_vec(i,0); 
         }
      }
      double norm_term = L2_norm(vec);
      for(int i = 0;i < data.rows;i++){
        Q(i,k) = vec(i,0)/norm_term;
      }
   } 
}

return Q;
}

};
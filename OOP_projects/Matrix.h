#include <iostream>
#include <tuple>
#include <math.h>
#include <cmath>
using namespace std;

template <typename T>

class Matrix{

private:

struct MatrixData{
int rows;
int cols;
T* ptr_matrix;

};

MatrixData data;

public:

Matrix(int row, int col){
    data.rows = row;
    data.cols = col;
    int number = row*col;
    data.ptr_matrix = new T[number]{};
}

~Matrix(){
delete[] data.ptr_matrix;    
}

//Matrix(const Matrix<T>& copied_mat){
  //  data.rows = copied_mat.data.rows;
    //data.cols = copied_mat.data.rows;
    //int number = data.rows*data.cols;
    //data.ptr_matrix = new T[number]{};

    //for (int i = 0;i < number;i++){
      //  data.ptr_matrix[i] = copied_mat.data.ptr_matrix[i];
   // }
//}

T& operator()(int row, int col){
   return data.ptr_matrix[(row*data.cols) + col];
}

int getRows() const{
return data.rows;
}

int getColumns() const{
return data.cols;
}

const T& operator()(int row, int col) const{
   return data.ptr_matrix[(row*data.cols) + col];
}

void print() const{
for(int i = 0;i < data.rows; i++){
    for(int j = 0;j < data.cols; j++){
        cout << "Matrix element: " << (*this)(i,j) << "\n";
    }
}     
}

Matrix <double> Transpose(){
Matrix <double> A_T(data.cols,data.rows);

for(int i = 0;i < data.rows; i++){
    for(int j = 0;j < data.cols;j++){
        A_T(j,i) = (*this)(i,j);
    }
}

return A_T;
}

Matrix<T> operator*(const Matrix<T>& A){
Matrix<T> C(data.rows,A.getColumns());

if(data.cols != A.getRows()){
   throw invalid_argument("The dimensions doesn't match.");  
}
else{
    for(int i = 0;i < data.rows;i++){
       for(int k = 0;k < A.getColumns();k++){
          T summation = 0.0;
          for(int j = 0;j < data.cols;j++){
              summation += (*this)(i,j)*A(j,k);
          }
          C(i,k) = summation;
       } 
    }
}

return C;
}

void LU_decomposition(Matrix <T>& L, Matrix <T>& U) const{

for(int i = 0;i < data.rows;i++){
   L(i,i) = 1.0;
}

for(int i = 0; i < data.rows;i++){
    for(int j = i;j < data.rows; j++){
        T summation1 = 0.0;
        for(int k = 0;k < i; k++){
            summation1 += L(i,k) * U(k,j);
        }
        U(i,j) = (*this)(i,j) - summation1;
    }

    for(int j = i + 1;j < data.rows; j++){
       T summation2 = 0.0;
       for(int k = 0;k < i;k++){
          summation2 += L(j,k) * U(k,i);
       } 
       L(j,i) = ((*this)(j,i) - summation2)/U(i,i);
    }
}
}

void ForwardSubstitution(Matrix<T>& y, Matrix<T>& b){
for(int i = 0;i < data.rows;i++){
   T sum1 = 0.0; 
   for(int j = 0; j < data.cols;j++){
      sum1 += (*this)(i,j)*y(j,0);  
   } 
   y(i,0) = b(i,0) - sum1;

}
   
}    

void BackwardSubstitution(Matrix<T>& x, Matrix<T>& y){
for(int i = data.cols-1;i>=0;i--){
   T sum1 = 0.0; 
   for(int j = i+1; j < data.cols;j++){
      sum1 += (*this)(i,j)*x(j,0);  
   } 
   x(i,0) = (y(i,0) - sum1)/(*this)(i,i);

}
   
}

T square(double & x){
return x*x;   
}

double L2norm(Matrix<T>& x, Matrix<T>& b){
double total_sum = 0.0;
for(int i = 0; i < data.rows; i++){
   double sum = 0.0;
   for(int j = 0; j < data.cols;j++){
      sum += (*this)(i,j)*x(j,0);   
   }
   double term = sum - b(i,0);
   total_sum += square(term); 
}

return sqrt(total_sum);
}

};
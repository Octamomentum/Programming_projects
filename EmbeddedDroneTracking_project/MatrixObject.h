//This class constructs a matrix as a data  containter that takes element (i,j) and is implemented
//by its constructors (copy, move) and operators that adds, subtracts, multiplies or treats reading/adding matrix elements.

#include <iostream>
#include <stdexcept>
#include <cmath>
#include <math.h>
using namespace std;

template <typename T>

class MatrixObject{

private:

struct MatrixData{

int rows;
int cols;
T* MatrixPtr;

};

MatrixData data;

public:

//Default constructor and destructor

MatrixObject(int r, int c){
if(r <= 0 || c <= 0){
  throw invalid_argument("Invalid dimension.");  
}
else{
   data.rows = r;
   data.cols = c;
   int number = data.rows*data.cols;
   data.MatrixPtr = new T[number]{}; 
}       
}

~MatrixObject(){
delete[] data.MatrixPtr;     
}

//Copy constructor and assignment

MatrixObject(const MatrixObject<T>& copied_mat){
data.rows = copied_mat.data.rows;
data.cols = copied_mat.data.cols;
int number = data.rows*data.cols;
data.MatrixPtr = new T[number]{};
for(int i = 0;i < number;i++){
   data.MatrixPtr[i] = copied_mat.data.MatrixPtr[i];    
}      
}

MatrixObject<T>& operator=(const MatrixObject<T>& copied_mat){
if(this != &copied_mat){
  delete[] data.MatrixPtr; 
  data.rows = copied_mat.data.rows;
  data.cols = copied_mat.data.cols;
  int number = data.rows*data.cols;
  data.MatrixPtr = new T[number]{};
  for(int i = 0;i < number;i++){
     data.MatrixPtr[i] = copied_mat.data.MatrixPtr[i];    
  }   
}
return *this;    
}

//Move constructor and assignment

MatrixObject(MatrixObject<T>&& copied_mat) noexcept{ 
data.rows = copied_mat.data.rows;
data.cols = copied_mat.data.cols;
data.MatrixPtr = copied_mat.data.MatrixPtr;

copied_mat.data.MatrixPtr = nullptr;
copied_mat.data.rows = 0;
copied_mat.data.cols = 0;
}

MatrixObject<T>& operator=(MatrixObject<T>&& copied_mat) noexcept{ 
if(this != &copied_mat){
   delete[] data.MatrixPtr;
   data.rows = copied_mat.data.rows;
   data.cols = copied_mat.data.cols;
   data.MatrixPtr = copied_mat.data.MatrixPtr;

   copied_mat.data.MatrixPtr = nullptr;
   copied_mat.data.rows = 0;
   copied_mat.data.cols = 0;
}   
return *this;
}

//Reading and writing element to a matrix

T& operator()(int r, int c){
if(r >= data.rows || r < 0 || c >= data.cols || c < 0){
  throw invalid_argument("Invalid element.");  
}
else{
   return data.MatrixPtr[data.cols*r + c];
}    
}

const T& operator()(int r, int c) const{
if(r >= data.rows || r < 0 || c >= data.cols || c < 0){
  throw invalid_argument("Invalid element.");  
}
else{
   return data.MatrixPtr[data.cols*r + c];
} 
}

//Row and column getters

const int getRows() const{
return data.rows;    
}

const int getCols() const{
return data.cols;    
}

//Print out the matrix if desired

void print() const{
for(int i = 0;i < data.rows;i++){
    for(int j = 0;j < data.cols;j++){
       cout << "Element (" << i << "," << j << "): " << (*this)(i,j) << "\n"; 
    }
}        
}

//A method that takes the transpose of a matrix 

const MatrixObject<T> Transpose() const{

MatrixObject<T> A_T(data.cols,data.rows);

for(int i = 0;i < data.rows;i++){
   for(int j = 0;j < data.cols;j++){
      A_T(j,i) = (*this)(i,j);
   }    
}
return A_T;    
}

//Multiplication, addition and subtraction of two matrices

const MatrixObject<T> operator*(const MatrixObject<T>& A) const{
MatrixObject<T> B(this->getRows(),A.getCols());
if(this->getCols() != A.getRows()){
  throw invalid_argument("The dimensions doesn't match.");  
}
else{
   for(int i = 0;i < this->getRows();i++){
   for(int j = 0;j < A.getCols();j++){
      double summation = 0.0; 
      for(int k = 0;k < this->getCols(); k++){
         summation += (*this)(i,k)*A(k,j);
      }
      B(i,j) = summation;
   } 
}
return B;    
}

}

const MatrixObject<T> operator+(const MatrixObject<T>& A) const{
MatrixObject<T> B(this->getRows(),A.getCols());
if(this->getRows() != A.getRows() && this->getCols() != A.getCols()){
  throw invalid_argument("The dimensions doesn't match.");  
}
else{
   for(int i = 0;i < this->getRows();i++){
   for(int j = 0;j < A.getCols();j++){
      B(i,j) = (*this)(i,j)+A(i,j);
   } 
}
return B;    
}
}

const MatrixObject<T> operator-(const MatrixObject<T>& A) const{
MatrixObject<T> B(data.rows,A.getCols());
if(data.rows != A.getRows() && data.cols != A.getCols()){
  throw invalid_argument("The dimensions doesn't match.");  
}
else{
   for(int i = 0;i < data.rows;i++){
   for(int j = 0;j < A.getCols();j++){
      B(i,j) = (*this)(i,j) - A(i,j);
   } 
}
return B;    
}
}

//A method that calculates the matrix element by a scalar value

const MatrixObject<T> scalarMatrix(const T& c)const{
MatrixObject<T> B = *this;

for(int i = 0;i < data.rows;i++){
   for(int j = 0;j < data.cols;j++){
      B(i,j) = c*B(i,j);
   } 
}

return B;    
}

//Equality operator

const bool operator==(const MatrixObject<T>& A) const{
if(this->getRows() != A.getRows() ||this -> getCols() != A.getCols()){
  return false; 
}
const double eps = 1e-4;
for(size_t i = 0;i < getRows();i++){
   for(size_t j = 0;j < getCols();j++){
      double result = (*this)(i,j) - A(i,j);
      cout << "Residual value: " << abs(result) << "\n";
      if(abs(result) > eps){
         return false;
      }
   }
}
return true;   
}

//Retrieving row/column vector given a column/row from a matrix

const MatrixObject<T> getRowVector(int r) const{
MatrixObject<T> row_vec(data.cols,1);
for(int j = 0;j < data.cols;j++){
   row_vec(j,0) = (*this)(r,j);
}
return row_vec;    
}

const MatrixObject<T> getColumnVector(int c) const{
MatrixObject<T> col_vec(data.rows,1);
for(int i = 0;i < data.rows;i++){
   col_vec(i,0) = (*this)(i,c);
}
return col_vec;    
}

//A method that swaps two rows of a matrix

void row_swap(int r1, int r2){
for(int j = 0; j < this -> getCols();j++){
   T temp = (*this)(r1,j);
   (*this)(r1,j) = (*this)(r2,j);
   (*this)(r2,j) = temp;
}
}

};

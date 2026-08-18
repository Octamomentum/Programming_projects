//This class constructs a matrix as a data container that takes element (i,j) and is implemented
//by its constructors (copy, move) and operators that adds, subtracts, multiplies or treats reading/adding matrix elements.

#include <iostream>
#include <stdexcept>
#include <cmath>
#include <algorithm>
#include <math.h>
#include <memory>

using namespace std;

template <typename T>

class MatrixObject{

private:

struct MatrixData{

size_t rows;
size_t cols;
unique_ptr<T[]> MatrixPtr = nullptr;

};

MatrixData data;

public:

//Default constructor and destructor

MatrixObject(size_t r, size_t c){
if(r <= 0 || c <= 0){
  throw invalid_argument("Invalid dimension.");  
}
else{
   data.rows = r;
   data.cols = c;
   size_t number = data.rows*data.cols;
   data.MatrixPtr = make_unique<T[]>(number); 
}       
}

MatrixObject(const MatrixObject<T>& copied_mat){
data.rows = copied_mat.data.rows;
data.cols = copied_mat.data.cols;
size_t number = data.rows*data.cols;
data.MatrixPtr = make_unique<T[]>(number);
copy(copied_mat.data.MatrixPtr.get(),copied_mat.data.MatrixPtr.get() + number, data.MatrixPtr.get());

}

MatrixObject<T>& operator=(const MatrixObject<T>& copied_mat){

if(this != &copied_mat){
   data.rows = copied_mat.data.rows;
   data.cols = copied_mat.data.cols;
   size_t number = data.rows*data.cols;
   data.MatrixPtr = make_unique<T[]>(number);
   copy(copied_mat.data.MatrixPtr.get(),copied_mat.data.MatrixPtr.get() + number, data.MatrixPtr.get());

}
  
return *this;    
}

MatrixObject(MatrixObject<T>&& copied_mat) noexcept{
data.rows = copied_mat.data.rows;
data.cols = copied_mat.data.cols;
data.MatrixPtr = move(copied_mat.data.MatrixPtr);
copied_mat.data.rows = 0;
copied_mat.data.cols = 0;
}

MatrixObject<T>& operator=(MatrixObject<T>&& copied_mat) noexcept{
if(this != &copied_mat){
  data.rows = copied_mat.data.rows;
  data.cols = copied_mat.data.cols;
  data.MatrixPtr = move(copied_mat.data.MatrixPtr);
  copied_mat.data.rows = 0;
  copied_mat.data.cols = 0;
 
}
return *this;  
}

//Reading and writing element to a matrix

T& operator()(const size_t& r,const size_t& c){
if(r >= data.rows || r < 0 || c >= data.cols || c < 0){
  throw invalid_argument("Invalid element.");  
}
else{
   return data.MatrixPtr[data.cols*r + c];
}    
}

const T& operator()(const size_t& r,const size_t& c) const{
if(r >= data.rows || r < 0 || c >= data.cols || c < 0){
  throw invalid_argument("Invalid element.");  
}
else{
   return data.MatrixPtr[data.cols*r + c];
} 
}

//Row and column getters

const size_t getRows() const{
return data.rows;    
}

const size_t getCols() const{
return data.cols;    
}

//Print out the matrix if desired

void print() const{
for(size_t i = 0;i < data.rows;i++){
    for(size_t j = 0;j < data.cols;j++){
       cout << "Element (" << i << "," << j << "): " << (*this)(i,j) << "\n"; 
    }
}        
}

//A method that takes the transpose of a matrix 

MatrixObject<T> Transpose() const{

MatrixObject<T> A_T(data.cols,data.rows);

for(size_t i = 0;i < data.rows;i++){
   for(size_t j = 0;j < data.cols;j++){
      A_T(j,i) = (*this)(i,j);
   }    
}
return A_T;    
}

//Multiplication, addition and subtraction of two matrices

MatrixObject<T> operator*(const MatrixObject<T>& A) const{
MatrixObject<T> B(this->getRows(),A.getCols());
if(this->getCols() != A.getRows()){
  throw invalid_argument("The dimensions doesn't match.");  
}
else{
   for(size_t i = 0;i < this->getRows();i++){
   for(size_t j = 0;j < A.getCols();j++){
      T summation = 0.0; 
      for(size_t k = 0;k < this->getCols(); k++){
         summation += (*this)(i,k)*A(k,j);
      }
      B(i,j) = summation;
   } 
}
return B;    
}

}

MatrixObject<T> operator+(const MatrixObject<T>& A) const{
MatrixObject<T> B(this->getRows(),A.getCols());
if(this->getRows() != A.getRows() && this->getCols() != A.getCols()){
  throw invalid_argument("The dimensions doesn't match.");  
}
else{
   for(size_t i = 0;i < this->getRows();i++){
   for(size_t j = 0;j < A.getCols();j++){
      B(i,j) = (*this)(i,j)+A(i,j);
   } 
}
return B;    
}
}

MatrixObject<T> operator-(const MatrixObject<T>& A) const{
MatrixObject<T> B(data.rows,A.getCols());
if(data.rows != A.getRows() && data.cols != A.getCols()){
  throw invalid_argument("The dimensions doesn't match.");  
}
else{
   for(size_t i = 0;i < data.rows;i++){
   for(size_t j = 0;j < A.getCols();j++){
      B(i,j) = (*this)(i,j) - A(i,j);
   } 
}
return B;    
}
}

//A method that calculates the matrix element by a scalar value

//Equality operator

bool operator==(const MatrixObject<T>& A) const{
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

};

/**
 * @file MatrixObject.h
 * @brief Flat 1d-array using smart pointers, to handle low and efficient memory allocation.
 * Copy constructors and assignment operators perform explicit deep copies of existing matrices via copy to ensure full pointer management,
 * while move constructors and assignment operators use move to perform move operations of existing matrices.
 *  Additionally, operators such as multiplication, addition and subtraction are implemented to perform matrix calculations in the other header files.
 * */ 

#pragma once
#include <iostream>
#include <stdexcept>
#include <cmath>
#include <algorithm>
#include <math.h>
#include <memory>

template <typename T>

class MatrixObject{

private:
struct MatrixData{
//A limit for the number of rows and columns is set to control the management of negative inputs to matrix indices.
size_t maxDimension = 1e12;
size_t rows;
size_t cols;
std::unique_ptr<T[]> MatrixPtr = nullptr;

};

MatrixData data;

public:

//Constructor for the matrix container
MatrixObject(size_t r, size_t c){
//Controls the matrix dimension feasbility
if(r == 0 || c == 0 || r >= data.maxDimension || c >= data.maxDimension){
  throw std::runtime_error("Invalid dimension");
}
else{
   data.rows = r;
   data.cols = c;
   size_t number = data.rows*data.cols;
   data.MatrixPtr = std::make_unique<T[]>(number); 
}       
}

//Copy constructor
MatrixObject(const MatrixObject<T>& original_mat){
data.rows = original_mat.data.rows;
data.cols = original_mat.data.cols;
data.maxDimension = original_mat.data.maxDimension;
size_t number = data.rows*data.cols;
data.MatrixPtr = std::make_unique<T[]>(number);
std::copy(original_mat.data.MatrixPtr.get(),original_mat.data.MatrixPtr.get() + number, data.MatrixPtr.get());
}

//Copy assignment operator
MatrixObject<T>& operator=(const MatrixObject<T>& original_mat){

if(this != &original_mat){
   data.rows = original_mat.data.rows;
   data.cols = original_mat.data.cols;
   data.maxDimension = original_mat.data.maxDimension;
   size_t number = data.rows*data.cols;
   data.MatrixPtr = std::make_unique<T[]>(number);
   std::copy(original_mat.data.MatrixPtr.get(),original_mat.data.MatrixPtr.get() + number, data.MatrixPtr.get());

}
  
return *this;    
}

//Move constructor
MatrixObject(MatrixObject<T>&& original_mat) noexcept{
data.rows = original_mat.data.rows;
data.cols = original_mat.data.cols;
data.maxDimension = original_mat.data.maxDimension;
data.MatrixPtr = std::move(original_mat.data.MatrixPtr);
original_mat.data.rows = 0;
original_mat.data.cols = 0;
original_mat.data.maxDimension = 1e12;

}

//Move assignment operator
MatrixObject<T>& operator=(MatrixObject<T>&& original_mat) noexcept{
if(this != &original_mat){
  data.rows = original_mat.data.rows;
  data.cols = original_mat.data.cols;
  data.maxDimension = original_mat.data.maxDimension;
  data.MatrixPtr = std::move(original_mat.data.MatrixPtr);
  original_mat.data.rows = 0;
  original_mat.data.cols = 0;
  original_mat.data.maxDimension = 1e12;


}
return *this;  
}

//Write matrix element at valid index
T& operator()(const size_t& r,const size_t& c){
//Controls that neither negative values or integer values larger or equal to the maximal dimension are set
if(r >= data.rows || r > data.maxDimension || c >= data.cols || c > data.maxDimension){
  throw std::invalid_argument("Invalid element.");  
}
else{
   return data.MatrixPtr[data.cols*r + c];
}    
}

//Read-only matrix element at valid index
const T& operator()(const size_t& r,const size_t& c) const{
//Controls that neither negative values or integer values larger or equal to the maximal dimension are set
if(r >= data.rows || r > data.maxDimension || c >= data.cols || c > data.maxDimension){
  throw std::invalid_argument("Invalid element.");  
}
else{
   return data.MatrixPtr[data.cols*r + c];
} 
}

//Getters for the number of rows and columns for a matrix
const size_t getRows() const{
return data.rows;    
}

const size_t getCols() const{
return data.cols;    
}

//Prints the content of a matrix 
void print() const{
for(size_t i = 0;i < data.rows;i++){
    for(size_t j = 0;j < data.cols;j++){
       std::cout << "Element (" << i << "," << j << "): " << (*this)(i,j) << "\n"; 
    }
}        
}

//Arithmetic operators

MatrixObject<T> operator*(const MatrixObject<T>& A) const{
MatrixObject<T> B(this->getRows(),A.getCols());
//Controls if the dimensions is valid
if(this->getCols() != A.getRows()){
  throw std::runtime_error("The dimensions doesn't match.");  
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
//Controls if the dimensions is valid
if(this->getRows() != A.getRows() || this->getCols() != A.getCols()){
  throw std::runtime_error("The dimensions doesn't match.");  
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
//Controls if the dimensions is valid
if(data.rows != A.getRows() || data.cols != A.getCols()){
  throw std::runtime_error("The dimensions doesn't match.");  
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

//Equal operator to check if two matrices have the same dimension and the same elements
bool operator==(const MatrixObject<T>& A) const{
if(this->getRows() != A.getRows() ||this -> getCols() != A.getCols()){
  return false; 
}
const double eps = 1e-4;
for(size_t i = 0;i < getRows();i++){
   for(size_t j = 0;j < getCols();j++){
      double result = (*this)(i,j) - A(i,j);
      if(abs(result) > eps){
         return false;
      }
   }
}
return true;   
}

};

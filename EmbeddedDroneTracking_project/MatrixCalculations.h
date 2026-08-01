//This class performs matrix calculations which mainly focuses on linearly dependence, dot product and decompositions of a matrix
//in order to solve a least square problem, especially LU/QR where the latter uses Givens rotation matrices. 

#include "MatrixObject.h"
#include <tuple>
#include <iostream>
#include <math.h>
#include <cmath>
#include <cstdlib>

using namespace std;

template <typename T>

class MatrixCalculations{

public:

const bool isLinearDependent(const MatrixObject<T>& A) const{
MatrixObject<T> G = A;
bool isLinDep = false;
int num_of_iterations = min(G.getRows(),G.getCols());
for(int i = 0;i < num_of_iterations;i++){
   T pivot = G(i,i);
   for(int j = i+1;j < G.getRows();j++){
      T multiplier = G(j,i)/pivot;
      G(j,i) = 0.0; 
     for(int k = i+1;k < G.getCols();k++){
         G(j,k) -= multiplier*G(i,k);
      }
   }   
}

cout << "\nGaussian elimination of input matrix:\n";
G.print();

const double eps = 1e-7;
for(int i = 0;i < G.getRows();i++){
   int number_of_zeros = 0;
   for(int j = 0;j < G.getCols();j++){
      if(abs(G(i,j)) < eps){
        number_of_zeros++; 
      }  
   }
   if(number_of_zeros == G.getCols()){
      isLinDep = true;
      break;
   }
}
if(isLinDep == false){
  cout << "The matrix is linear independent.";
}
return isLinDep;   
}

const T dot_product(const MatrixObject<T>& v1, const MatrixObject<T>& v2) const{
T summation = 0.0;
for(int i = 0;i < v1.getRows();i++){
   summation += v1(i,v1.getCols()-1)*v2(i,v2.getCols()-1);
}
return summation;   
}

const MatrixObject<T> generateIdentityMatrix(int r) const{
MatrixObject<T> identity_matrix(r,r);
for(int i = 0;i < r;i++){
   identity_matrix(i,i) = 1.0;
}
return identity_matrix;   
}

tuple<MatrixObject<T>,int> RowSwapPossibilities(const MatrixObject<T>& A) const{
int counter = 0;
const double eps = 1e-7;
MatrixObject<T> rowswap_vec(A.getRows(),1);
int min_number = min(A.getRows(),A.getCols());
for(int i = 0;i < A.getRows();i++){
   for(int j = 0;j < min_number;j++)
   {
   if(j == i && abs(A(i,j)) < eps){
      rowswap_vec(i,rowswap_vec.getCols()-1) = 1.0;
      counter++;
      }
   }   
}
return {rowswap_vec,counter};
}

const tuple<MatrixObject<T>,MatrixObject<T>> generateP(const MatrixObject<T>& A) const{
MatrixObject<T> P = generateIdentityMatrix(A.getRows());
MatrixObject<T> PA = P*A;
auto [rowswap_vec,counter] = RowSwapPossibilities(A);
srand(time(0));
while(counter != 0){
    for(int i1 = 0;i1 < rowswap_vec.getRows();i1++){
       if(rowswap_vec(i1,rowswap_vec.getCols()-1) == 1.0){
         int i2 = rand()%rowswap_vec.getRows();
         while(i2 == i1){
             int i2 = rand()%rowswap_vec.getRows();
             if(i2 != i1){
               break;
             }
         }
         P.row_swap(i1,i2);
       }
    }
    PA = P*A;
    auto [rowswap_vec,counter] = RowSwapPossibilities(PA);
    if(counter == 0){
      break;
    }
    }
return {P,PA};
}

const tuple <MatrixObject<T>,MatrixObject<T>> LU_decomposeMatrix(const MatrixObject<T>& A) const{

MatrixObject<T> L = generateIdentityMatrix(A.getRows());
MatrixObject<T> U = A;
int num_of_iterations = min(A.getRows(),A.getCols());

for(int i = 0;i < num_of_iterations;i++){
   T pivot = U(i,i);
   const double eps = 1e-7;
   if(abs(pivot) < eps){
     throw invalid_argument("Linear independence!"); 
   }
   else {
      for(int j = i+1;j < A.getRows();j++){
         T multiplier = U(j,i)/pivot;
         L(j,i) = multiplier;
         U(j,i) = 0.0;
         for(int k = i+1;k < A.getCols();k++){
            U(j,k) -= multiplier*U(i,k);
      } 
   }
}
}
return {L,U};
}

const MatrixObject<T> ForwardSubstitution(const MatrixObject<T>& L, const MatrixObject<T>& b)const{

MatrixObject<T> y(b.getRows(),b.getCols());
for(int i = 0;i < b.getRows();i++){
   T summation = 0.0;
   for(int j = 0;j < L.getCols();j++){
      summation += L(i,j)*y(j,b.getCols()-1);
   } 
   y(i,b.getCols() - 1) = b(i,b.getCols() - 1) - summation;
}    
    
return y;    
}

const MatrixObject<T> BackwardSubstitution(const MatrixObject<T>& U, const MatrixObject<T>& y)const {

MatrixObject<T> x(y.getRows(),y.getCols());
for(int i = y.getRows()-1;i >= 0;i--){
   T summation = 0.0;
   for(int j = 0;j < U.getCols();j++){
      summation += U(i,j)*x(j,y.getCols()-1);
   } 
   x(i,y.getCols()-1) = (y(i,y.getCols()-1) - summation)/U(i,i);
}    
    
return x;    
}

const double square(const double x)const{
return x*x;
}

const double L2Norm(const MatrixObject<T>& res_vec)const{

double res = 0.0;

for(int i = 0;i < res_vec.getRows();i++){
   res += square(res_vec(i,res_vec.getCols()-1)); 
}

return sqrt(res);    
}

const tuple<T,T> GivensParameters(const T& r1, const T& r2) const{ 
T denum = sqrt(square(r1) + square(r2));
if(denum == 0){
  throw runtime_error("The Givens approach diverges."); 
}
else{
   T c = r1/denum;
   T s = r2/denum;
   return {c,s};
}
}

const MatrixObject<T> GivensMatrix(const MatrixObject<T>& A, const int& annihil_ind, const int& curr_col)const{
auto [c,s] = GivensParameters(A(annihil_ind-1,curr_col),A(annihil_ind,curr_col)); 
MatrixObject<T> G = generateIdentityMatrix(A.getRows());
G(annihil_ind-1,annihil_ind-1) = c;
G(annihil_ind-1,annihil_ind) =  -s;
G(annihil_ind, annihil_ind-1) = s;
G(annihil_ind,annihil_ind) = c;
return G;   
}

const tuple<MatrixObject<T>,MatrixObject<T>> QRDecomposition_GivensApproach(const MatrixObject<T>& A)const{
const double eps = 1e-7;
MatrixObject<T> Q = generateIdentityMatrix(A.getRows());
MatrixObject<T> R = A;
T num_of_iterations = min(A.getRows(),A.getCols());
for(int j = 0;j < num_of_iterations;j++){
   for(int i = A.getRows()-1;i > j;i--){
      T val = R(i,j);
      if(abs(val) > eps){
        T pivot_val = R(i-1,j);
        T annihilated_val = R(i,j);
        MatrixObject<T> G = GivensMatrix(R,i,j);
        R = G.Transpose()*R;
        Q = Q*G;
      }
   }
}
return {Q,R};   
}

const tuple<MatrixObject<T>,MatrixObject<T>> generateTruncatedMatrices(const MatrixObject<T> & A, const MatrixObject<T>& b)const{
T truncated_rows = min(A.getRows(),A.getCols());
MatrixObject<T> A_trunc(truncated_rows,A.getCols());
MatrixObject<T> b_trunc(truncated_rows,b.getCols());

for(int i = 0;i < A_trunc.getRows();i++){
   for(int j = 0;j < A_trunc.getCols();j++){
      A_trunc(i,j) = A(i,j);
   }
   b_trunc(i,b_trunc.getCols()-1) = b(i,b.getCols()-1);
}

return {A_trunc, b_trunc};
}

const MatrixObject<T> LeastSquareSolver_QRDecomposition(const MatrixObject<T>& A, const MatrixObject<T>& b)const{
const double eps = 1e-7;
auto [Q,R] = QRDecomposition_GivensApproach(A);
MatrixObject<T> b_tilde = Q.Transpose()*b;

auto [R_filtered,b_filtered] = generateTruncatedMatrices(R,b_tilde);

return BackwardSubstitution(R_filtered,b_filtered);
}

};
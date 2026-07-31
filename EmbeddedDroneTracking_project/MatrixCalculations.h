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

const bool isLinearIndependent(const MatrixObject<T>& A) const{
MatrixObject<T> G = A;
bool isLinIndep = false;
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
      isLinIndep = true;
      cout << "The matrix has linear dependent columns.";
      break;
   }
}
if(isLinIndep == false){
  cout << "The matrix is linear independent.";
}
return isLinIndep;   
}

const T dot_product(const MatrixObject<T>& v1, const MatrixObject<T>& v2) const{
T summation = 0.0;
for(int i = 0;i < v1.getRows();i++){
   summation += v1(i,v1.getCols()-1)*v2(i,v2.getCols()-1);
}
return summation;   
}

const bool HasOrthogonalColumns(const MatrixObject<T>& A,int c1,int c2)const {
bool is_orth = false;
MatrixObject<T> v1 = A.getColumnVector(c1);
MatrixObject<T> v2 = A.getColumnVector(c2);
T val = dot_product(v1,v2);
cout << "Dot product: " << val;
const double eps = 1e-8;
if(abs(val) < eps){
  is_orth = true; 
}
return is_orth;
}

const bool isOrthogonal(MatrixObject<T>& A) const{
bool orth = false;
int counter = 0;
for(int j1 = 0;j1 < A.getCols()-1;j1++){
   for(int j2 = j1+1;j2 < A.getCols();j2++){
      if(HasOrthogonalColumns(A,j1,j2)){
        counter++;
      }
   }
}   
if(counter == A.getCols()){
   orth = true;
   cout << "The matrix is orthogonal.";
}
return orth;
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
        T denum = sqrt(pivot_val*pivot_val + annihilated_val*annihilated_val);
        T c = pivot_val/denum;
        T s = annihilated_val/denum;
        MatrixObject<T> G = generateIdentityMatrix(A.getRows());
        G(i-1,i-1) = c;
        G(i-1,i) = -s;
        G(i,i-1) = s;
        G(i,i) = c; 
        R = G.Transpose()*R;
        Q = Q*G;
      }
   }
}
return {Q,R};   
}

const MatrixObject<T> LeastSquareSolver_QRDecomposition(const MatrixObject<T>& A, const MatrixObject<T>& b)const{
const double eps = 1e-7;
T min_val = min(A.getRows(),A.getCols());
T differance_val = A.getRows()-min_val;
auto [Q,R] = QRDecomposition_GivensApproach(A);
MatrixObject<T> R_tilde(A.getRows()-differance_val,R.getCols());
MatrixObject<T> b_tilde = Q.Transpose()*b;
MatrixObject<T> b_filtered(A.getRows()-differance_val,b_tilde.getCols());
MatrixObject<T> x(b_filtered.getRows(),1);


for(int i = 0;i < R_tilde.getRows();i++){
   for(int j = 0; j < R_tilde.getCols();j++){
      R_tilde(i,j) = R(i,j);
      }
   
   b_filtered(i,b_filtered.getCols()-1) = b_tilde(i,b_tilde.getCols()-1);
}

for(int i = R_tilde.getRows()-1;i >= 0;i--){
   T summation = 0.0;
   for(int j = i+1;j < R_tilde.getCols();j++){
      summation += R_tilde(i,j)*x(j,x.getCols()-1);
   }
   x(i,x.getCols()-1) = (b_filtered(i,b_filtered.getCols()-1) - summation)/R_tilde(i,i);
}
return x;
}

};
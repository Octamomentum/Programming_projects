//Testing the functionality from MatrixObject.h

#include "MatrixObject.h"
#include <gtest/gtest.h>

TEST(MatrixDimension,DimensionIndexTest){
//Verifies invalid dimensions
EXPECT_THROW({MatrixObject<double> Azero(0,0);},std::runtime_error);
EXPECT_THROW({MatrixObject<double> Anegative(-9999,3);},std::runtime_error);
//Verifies invalid matrix element insertion 
MatrixObject<double> A_wronginsertion(3,2);
EXPECT_THROW({A_wronginsertion(3,0) = 5.3;},std::invalid_argument);
EXPECT_THROW({A_wronginsertion(-2,1) = 0.1;},std::invalid_argument);
MatrixObject<double> A(3,2);
A(0,0) = 2.0;
A(0,1) = 3.0;
A(1,0) = 5.0;
A(1,1) = -3.0;
A(2,0) = 1.1;
A(2,1) = -7.6;
//Verifies the dimensionality is correct
EXPECT_DOUBLE_EQ(A.getRows(),3);
EXPECT_DOUBLE_EQ(A.getCols(),2);
A.print();
}

TEST(MatrixCopy,CopyTest){
//In this test, it verifies the deep copy for an explicit matrix 
MatrixObject<double> A(2,2);
A(0,0) = 2.0;
A(0,1) = 3.0;
A(1,0) = 5.0;
A(1,1) = -3.0;
MatrixObject<double> Acopied = A;
EXPECT_EQ(A,Acopied);
}

TEST(MatrixMove,MoveTest){
//In this test, it verifies that the elements of a matrix is correctly moved to another matrix
MatrixObject<double> A(2,2);
A(0,0) = 2.0;
A(0,1) = 3.0;
A(1,0) = 5.0;
A(1,1) = -3.0;
MatrixObject<double> Amoved = std::move(A);
EXPECT_DOUBLE_EQ(Amoved(1,0), 5.0);
}

TEST(MatrixOperations,CalcTest){
//We test here the fundamental matrix operations such as addition, subtraction and multiplication
MatrixObject<double> A(2,2);
MatrixObject<double> B(3,2);
MatrixObject<double> C(2,2);
MatrixObject<double> D(2,3);
MatrixObject<double> v(2,1);
A(0,0) = 2.0;
A(0,1) = 1.0;
A(1,0) = 5.0;
A(1,1) = 6.0;

B(0,0) = 1.0;
B(0,1) = 1.0;
B(1,0) = 1.0;
B(1,1) = -1.0;
B(2,0) = 0.0;
B(2,1) = 1.0;

C(0,0) = 1.0;
C(0,1) = 1.0;
C(1,0) = 0.0;
C(1,1) = 1.0;

D(0,0) = 0.0;
D(0,1) = 1.0;
D(0,2) = 0.0;
D(1,0) = 1.0;
D(1,1) = 2.0;
D(1,2) = -1.0;

//Verifies that the matrix operations are invalid

EXPECT_THROW({A*B;},std::runtime_error);
EXPECT_THROW({A+B;},std::runtime_error);
EXPECT_THROW({A-B;},std::runtime_error);

MatrixObject<double> result1(2,2);
MatrixObject<double> result2(2,2);
MatrixObject<double> result3(3,3);

result1(0,0) = 3.0;
result1(0,1) = 2.0;
result1(1,0) = 5.0;
result1(1,1) = 7.0;

result2(0,0) = 1.0;
result2(0,1) = 0.0;
result2(1,0) = 5.0;
result2(1,1) = 5.0;


result3(0,0) = 1.0;
result3(0,1) = 3.0;
result3(0,2) = -1.0;
result3(1,0) = -1.0;
result3(1,1) = -1.0;
result3(1,2) = 1.0;
result3(2,0) = 1.0;
result3(2,1) = 2.0;
result3(2,2) = -1.0;

//Verifies that the matrix operations are done correct
EXPECT_EQ(A+C,result1);
EXPECT_EQ(A-C,result2);
EXPECT_EQ(B*D,result3);

//Verify that Matrix Transpose is correct

MatrixObject<double> A_T = A.Transpose();
MatrixObject<double> AT_test(2,2);

AT_test(0,0) = 2.0;
AT_test(0,1) = 5.0;
AT_test(1,0) = 1.0;
AT_test(1,1) = 6.0;
EXPECT_EQ(A_T,AT_test);

//Verifying that a matrix is correctly scaled

double alpha = 0.5;
v(0,0) = 1.0;
v(1,0) = 7.0;

MatrixObject<double> scaled_vector = v.scalarMatrix(alpha);
MatrixObject<double> v_test(2,1);
v_test(0,0) = 0.5;
v_test(1,0) = 3.5;
v.print();
EXPECT_EQ(v_test,scaled_vector);


//Verifying that a row/column of a matrix is correctly generated
MatrixObject<double> row_vec = A.getRow(1);
MatrixObject<double> column_vec = A.getColumn(0);

MatrixObject<double> row_test(2,1);
MatrixObject<double> column_test(2,1);

row_test(0,0) = 5.0;
row_test(1,0) = 6.0;
column_test(0,0) = 2.0;
column_test(1,0) = 5.0;

EXPECT_EQ(row_test,row_vec);
EXPECT_EQ(column_test,column_vec);

}
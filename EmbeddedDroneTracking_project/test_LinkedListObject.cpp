#include "LinkedListObject.h"
#include <iostream>
#include <gtest/gtest.h>

using namespace std;

TEST(LinkedListTest,AddingAndRemovingElements){
   LinkedListObject<double> lst;
   //Checking negative index
   EXPECT_THROW({lst[-1];},runtime_error);
   //Checking out of bounds
   EXPECT_THROW({lst[1];},runtime_error);
   //Checking if removing element from empty list is possible or not
   EXPECT_THROW({lst.removeElement(0);},runtime_error);
   //Adding elements
   lst.addElement(5);
   lst.addElement(9);
   lst.addElement(11);
   lst.addElement(15);
   //Checking if removing element from a list that doesn't exist results to out of bounds
   EXPECT_THROW({lst.removeElement(13);},runtime_error);
   lst.print();
   cout << "List length: " << lst.getLength();
   //Checking the operator[] retrieves its expected value at valid position
   EXPECT_DOUBLE_EQ(lst[2], 11.0);
   //Checking out of bounds
   EXPECT_THROW({lst[32];},invalid_argument);
   //Checking the first and last elements of a list
   EXPECT_DOUBLE_EQ(lst.getFirstElement(),5.0);
   EXPECT_DOUBLE_EQ(lst.getLastElement(),15.0);
   //Checking if it is possible to remove a value which results out of bounds
   EXPECT_THROW({lst.removeElement(99);},runtime_error);
   //Remove the first element
   lst.removeElement(0);
   lst.print();
   EXPECT_DOUBLE_EQ(lst[0],9.0);
   lst.removeElement(lst.getLength()-1);
   EXPECT_DOUBLE_EQ(lst[lst.getLength()-1], 11.0);
}

TEST(LinkedListTest,CopyTest){
    //Initialize an original list and construct a copy to check if they are equal
    LinkedListObject<double> lst;
    LinkedListObject<double> non_equalList;
    lst.addElement(3);
    lst.addElement(7);
    non_equalList.addElement(5);
    non_equalList.addElement(8);
    non_equalList.addElement(13);
    //Checking if two defined lists above are non-equal
    EXPECT_FALSE(lst == non_equalList);
    LinkedListObject<double> copied_lst = lst;
    EXPECT_EQ(lst,copied_lst);
}

TEST(LinkedListTest,MoveTest){
    //Initialize a list and construct a new list to move contents and checking if it is done correctly
    LinkedListObject<double> lst;
    lst.addElement(4);
    lst.addElement(5);
    LinkedListObject<double> moved_lst;
    moved_lst = move(lst);
    EXPECT_EQ(moved_lst[1],5.0);
}
//Testing the functionality from LinkedListObject.h

#include "LinkedListObject.h"
#include <iostream>
#include <gtest/gtest.h>

TEST(LinkedListTest,AddingAndRemovingElements){
   LinkedListObject<double> lst;
   //Checking negative index
   EXPECT_THROW({lst[-1];},std::runtime_error);
   //Checking out of bounds
   EXPECT_THROW({lst[1];},std::runtime_error);
   //Checking if removing element from empty list is possible or not
   EXPECT_THROW({lst.removeElement(0);},std::runtime_error);
   //Adding elements
   lst.addElement(5);
   lst.addElement(9);
   lst.addElement(11);
   lst.addElement(15);
   //Checking if removing element from a index of a list that doesn't exist results out of bounds
   EXPECT_THROW({lst.removeElement(13);},std::runtime_error);
   lst.print();
   std::cout << "List length: " << lst.getLength();
   //Checking the operator[] retrieves its expected value at valid position
   EXPECT_DOUBLE_EQ(lst[2], 11.0);
   //Checking out of bounds
   EXPECT_THROW({lst[32];},std::invalid_argument);
   //Checking the first and last elements of a list
   EXPECT_DOUBLE_EQ(lst.getFirstElement(),5.0);
   EXPECT_DOUBLE_EQ(lst.getLastElement(),15.0);
   //Checking if it is possible to remove a value which results out of bounds
   EXPECT_THROW({lst.removeElement(99);},std::runtime_error);
   //Remove the first element
   lst.removeElement(2);
   lst.print();
   EXPECT_DOUBLE_EQ(lst[1],9.0);
   EXPECT_DOUBLE_EQ(lst[lst.getLength()-1], 15.0);
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
    moved_lst = std::move(lst);
    EXPECT_EQ(moved_lst[1],5.0);
}
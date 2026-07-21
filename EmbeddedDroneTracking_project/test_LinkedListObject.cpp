#include "LinkedListObject.h"
#include <iostream>
#include <gtest/gtest.h>

using namespace std;

TEST(LinkedListTest,testLinkedList){
   LinkedListObject<double> lst;
   EXPECT_THROW({lst[999999];},runtime_error);
   lst.addElement(5);
   lst.addElement(9);
   lst.addElement(11);
   lst.addElement(15);
   lst.print();
   cout << "List length: " << lst.getLength();
   cout << "\nRetrieved element: " << lst[2];
   EXPECT_DOUBLE_EQ(lst[2], 11.0);
   EXPECT_THROW({lst[32];},invalid_argument);
   EXPECT_DOUBLE_EQ(lst.getFirstElement(),5.0);
   EXPECT_DOUBLE_EQ(lst.getLastElement(),15.0);

   EXPECT_THROW({lst.removeElement(99);},runtime_error);
   lst.removeElement(0);
   lst.print();
   EXPECT_DOUBLE_EQ(lst[0],9.0);
   lst.removeElement(lst.getLength()-1);
   EXPECT_DOUBLE_EQ(lst[lst.getLength()-1], 11.0);
}

TEST(LinkedListTest,CopyTest){
    LinkedListObject<double> lst;
    lst.addElement(3);
    lst.addElement(7);
    LinkedListObject<double> copied_lst;
    copied_lst = lst;
    EXPECT_EQ(lst,copied_lst);
}

TEST(LinkedListTest,MoveTest){
    LinkedListObject<double> lst;
    lst.addElement(4);
    lst.addElement(5);
    LinkedListObject<double> moved_lst;
    moved_lst = move(lst);
    EXPECT_EQ(moved_lst[1],5.0);
    EXPECT_EQ(lst.getLength(),0);
}
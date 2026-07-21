#include "LinkedList.h"
#include <iostream>
using namespace std;

int main(){
    LinkedList <int> lst;
    lst.addElement(5);
    lst.addElement(7);
    lst.addElement(21);
    cout << lst[1];
    cout << "List length: " << lst.list_size();
}
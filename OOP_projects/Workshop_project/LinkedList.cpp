#include "LinkedList.h"
#include <iostream>
#include <stdexcept>

using namespace std;

int main(){

LinkedList <double> lst;

lst.addElement(7);
lst.addElement(3);
lst.addElement(-4);
cout << "Number of elements: " << lst.getLength() << "\n";
lst.print();
cout << "\nThis element is retrievable: " << lst[1] << "\n";
try{
cout << lst[1131];
}
catch(const out_of_range e){
cout << e.what();
} 

}
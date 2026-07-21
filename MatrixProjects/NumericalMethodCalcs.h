#include <iostream>
#include <functional>
#include <stdexcept>
#include <tuple>
#include <cmath>
#include <math.h>
using namespace std;

template <typename T>

class LinkedListStructure{

private:

struct Node{

T data;
Node* nextNode;    

};

Node* headNode;
int length;

public:

LinkedListStructure():headNode(nullptr),length(0){};

LinkedListStructure(const LinkedListStructure<T>& copied_list){
this -> length = copied_list.length;
headNode = nullptr;

if(copied_list.headNode == nullptr){
  return;
}

headNode = new Node(copied_list.headNode -> data);

Node* currentCopy = copied_list.headNode -> nextNode;
Node* currentOriginal = headNode;

while(currentCopy != nullptr){
    currentOriginal -> nextNode = new Node(currentCopy -> data);
    currentOriginal = currentOriginal -> nextNode;
    currentCopy = currentCopy -> nextNode;
}

}

~LinkedListStructure(){
Node* currentNode = headNode;
while(currentNode != nullptr){
    Node* newNode = currentNode -> nextNode;
    delete currentNode;
    currentNode = newNode; 
}    
}

void addElement(T val){
Node* newNode = new Node{val,nullptr};
if(headNode == nullptr){
  headNode = newNode;
}  
else{
    Node* tempNode = headNode;
    while(tempNode -> nextNode != nullptr){
        tempNode = tempNode -> nextNode;
    }
    tempNode -> nextNode = newNode;
}
length++;    
}

T& operator[](int ind){
Node* currentNode = headNode;
int counter = 0;

while(currentNode != nullptr){
    if(counter == ind){
      return currentNode -> data;  
    }
    counter++;
    currentNode = currentNode -> nextNode;

}
throw out_of_range("Out of bounds.");   

}

void print(){
cout << "\n";
Node* currentNode = headNode;
while(currentNode != nullptr){
    cout << currentNode -> data << "->";
    currentNode = currentNode -> nextNode;
}
cout << "Null pointer\n";
}

int getLength(){
return length;
}

double diff_calc(double x, double h, function<double (double)> f){
return (f(x+h) - f(x-h))/(2*h);
}

tuple<LinkedListStructure<int>,LinkedListStructure<double>> NR_calc(double x, double h, function<double (double)> f, double eps){

LinkedListStructure<int> iter_list;
LinkedListStructure<double> x_list;

int i = 0;
iter_list.addElement(i);
x_list.addElement(x);

while(!(abs(f(x)) < eps)){
    cout << "Iteration " << i + 1 << "\n";
    double f_diff = diff_calc(x,h,f);
    x -= f(x)/f_diff;
    i++;
    iter_list.addElement(i);
    x_list.addElement(x);
}
cout << "Final solution: " << x << " at Iteration " << i << ".\n";

return {iter_list, x_list};
}

tuple<LinkedListStructure<double>,LinkedListStructure<double>> Euler_calc(tuple<double, double, double> xy_data, double h, function<double (double,double)> f){

LinkedListStructure<double> x_list;
LinkedListStructure<double> y_list;

double x_i = get<0>(xy_data);
double x_f = get<1>(xy_data);
double y_i = get<2>(xy_data);

int i = 0;
x_list.addElement(x_i);
y_list.addElement(y_i);

double x = x_i;
double y = y_i;

while(x < x_f){
    cout << "Iteration " << i + 1 << "\n";
    y += h*f(x,y);
    x += h;
    i++;
    x_list.addElement(x);
    y_list.addElement(y);
}
cout << "Final solution: y-value " << y << " at x-value " << x << ".\n";

return {x_list, y_list};
}

tuple<LinkedListStructure<double>,LinkedListStructure<double>> RK4_calc(tuple<double, double, double> xy_data, double h, function<double (double,double)> f){

LinkedListStructure<double> x_list;
LinkedListStructure<double> y_list;

double x_i = get<0>(xy_data);
double x_f = get<1>(xy_data);
double y_i = get<2>(xy_data);

int i = 0;
x_list.addElement(x_i);
y_list.addElement(y_i);

double x = x_i;
double y = y_i;

while(x < x_f){
    cout << "Iteration " << i + 1 << "\n";
    double k1 = f(x,y);
    double k2 = f(x + h/2,y + (k1*h)/2);
    double k3 = f(x + h/2,y+ (k2*h)/2);
    double k4 = f(x + h,y + h*k3);
    y += (h/6)*(k1 + k4 + 2*(k2 + k3));
    x += h;
    i++;
    x_list.addElement(x);
    y_list.addElement(y);
}
cout << "Final solution: y-value " << y << " at x-value " << x << ".\n";

return {x_list, y_list};
}

tuple<LinkedListStructure<double>,LinkedListStructure<double>> Heun_calc(tuple<double, double, double> xy_data, double h, function<double (double,double)> f){

LinkedListStructure<double> x_list;
LinkedListStructure<double> y_list;

double x_i = get<0>(xy_data);
double x_f = get<1>(xy_data);
double y_i = get<2>(xy_data);

int i = 0;
x_list.addElement(x_i);
y_list.addElement(y_i);

double x = x_i;
double y = y_i;

while(x < x_f){
    cout << "Iteration " << i + 1 << "\n";
    double y_tilde = y + h*f(x,y);
    y += (h/2)*(f(x,y) + f(x,y_tilde));
    x += h;
    i++;
    x_list.addElement(x);
    y_list.addElement(y);
}
cout << "Final solution: y-value " << y << " at x-value " << x << ".\n";

return {x_list, y_list};
}

};
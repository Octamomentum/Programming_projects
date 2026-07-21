#include <iostream>
#pragma once
using namespace std;

template <typename T>


class LinkedList{

private:

struct Node{

T data;
Node* next;
};
Node* head;
int length;

public:

LinkedList():head(nullptr),length(0){}

~LinkedList(){

Node* current = head;
while(current != nullptr){
    Node* newNode = current -> next;
    delete current;
    current = newNode;
}

}

void addElement(T val){
Node* newNode = new Node{val,nullptr};
if(head == nullptr){
   head = newNode;
}
else{
    Node* temp = head;
    while(temp -> next != nullptr){
        temp = temp -> next;
    }
    temp -> next = newNode;
}   
length++;  
}

void print() const{
Node* current = head;
while(current != nullptr){
    cout << current -> data << "->";
    temp = current -> next;
}
cout << "Null pointer\n";
}

T& operator[](int ind){
Node* current = head;
int counter = 0;
while(current != nullptr){
    if(counter == ind){
      return current -> data;  
    }
    counter++;
    current = current -> next;
}
throw out_of_range("Out of bounds.");
}

int list_size(){
return length;
}

};
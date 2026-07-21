#include <iostream>
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

LinkedList(): head(nullptr), length(0){};

~LinkedList(){

Node* current = head;
while(current != nullptr){
    Node* newNode = current -> next;
    delete current;
    current = newNode;
}
}

LinkedList(const LinkedList<T>& copied_list){

this->length = copied_list.length;
head = nullptr;
if(copied_list.head == nullptr){
  return;   
}

head = new Node(copied_list.head -> data);
Node* currentCopy = copied_list.head -> next;
Node* currentOriginal = head;

while(currentCopy != nullptr){
    currentOriginal = new Node(currentCopy -> data);
    currentOriginal = currentOriginal -> next;
    currentCopy = currentCopy -> next;
}
}

void addElement(T val){
Node* newNode = new Node(val,nullptr);
if(head == nullptr){
  head = newNode;  
}    
else{
    Node* temp = head;
    while(temp -> next!= nullptr){
        temp = temp -> next;
    }
    temp -> next = newNode;
}
length++;
}

int getLength() const{
return length;    
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

void print() const{
Node* current = head;
while(current != nullptr){
    cout << current -> data << "->";
    current = current -> next;
} 
cout << "Null pointer\n";
}

};


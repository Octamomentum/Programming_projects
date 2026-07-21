//This data structure is a linked list built in a node based logic that has copy/move assignment operators and
//methods that can add or remove existing elements. Also, one can print its content and
//retrieve its element on a certain index.

#include <iostream>
#include <stdexcept>

using namespace std;

template <typename T>

class LinkedListObject{

private:

struct Node{
T data;
Node* next;
};
Node* head = nullptr;
Node* tail = nullptr;
int length = 0;

public:

//Constructor and destructor

LinkedListObject(){};

void clearList(){
Node* current = head;

while(current != nullptr){
    Node* newNode = current->next;
    delete current;
    current = newNode;
}
head = nullptr;
tail = nullptr; 
}

~LinkedListObject(){
clearList();
}

//Copy constructor and its assignment operator

LinkedListObject(const LinkedListObject<T>& original_list){

if(original_list.head == nullptr){
  head = nullptr;
  tail = nullptr;
  length = 0;  
  return;
}

head = new Node(original_list.head -> data);
length = original_list.length;

Node* currentOriginal = original_list.head -> next;
Node* currentCopy = head;

while(currentOriginal != nullptr){
    currentCopy -> next = new Node(currentOriginal->data);
    currentOriginal = currentOriginal -> next;
    currentCopy = currentCopy -> next;
}
tail = currentCopy;

}

LinkedListObject<T>& operator=(const LinkedListObject<T>& original_list){

if(this != &original_list){
  clearList();   
  if(original_list.head == nullptr){
  head = nullptr;
  tail = nullptr;
  length = 0;  
  }

  head = new Node(original_list.head -> data);
  length = original_list.length;

  Node* currentOriginal = original_list.head -> next;
  Node* currentCopy = head;

  while(currentOriginal != nullptr){
       currentCopy -> next = new Node(currentOriginal->data);
       currentOriginal = currentOriginal -> next;
       currentCopy = currentCopy -> next;
  }
  tail = currentCopy;

}

return *this;
}

//Move constructor and its assigned operator 

LinkedListObject(LinkedListObject<T>&& original_list) noexcept{

if(original_list.head == nullptr){
  head = nullptr;
  tail = nullptr;
  length = 0;
  return;
}

head = original_list.head;
tail = original_list.tail;
length = original_list.length;

original_list.head = nullptr;
original_list.tail = nullptr;
original_list.length = 0;

}

LinkedListObject<T>& operator=(LinkedListObject<T>&& original_list) noexcept{

if(this != &original_list){
  if(original_list.head == nullptr){
     head = nullptr;
     tail = nullptr;
     length = 0;
  }

  head = original_list.head;
  tail = original_list.tail;
  length = original_list.length;

  original_list.head = nullptr;
  original_list.tail = nullptr;
  original_list.length = 0; 
}    
return *this;

}

//Method that adds element to the list

void addElement(const T& val){
Node* newNode = new Node{val,nullptr};
if(head == nullptr){
  head = newNode;  
  tail = newNode;
}
else{
   tail->next = newNode;
   tail = newNode; 
}
length++;    
}

//Method that prints the content of a list

void print(){
Node* current = head;
cout << "\nPrinting list content...\n";
while(current != nullptr){
    cout << current -> data << "->";
    current = current -> next;
}    
}

//An operator that retrieves the element of a list if its exists, otherwise a runtime error
//error is thrown

T& operator[](const int& ind) const{
if(head == nullptr){
   throw runtime_error("The list is empty, add elements."); 
}
else{
   int counter = 0;
   Node* current = head;
   while(current != nullptr){
       if(counter == ind){
         return current -> data;
       }
       counter++;
       current = current -> next;
   }
   throw invalid_argument("Out of bounds."); 
}
}

//Method that removes element given a valid index

void removeElement(const int& ind){
if(head == nullptr){
  throw runtime_error("The list is empty.");
}
else if(ind < 0 || ind >= this -> getLength()){
   throw runtime_error("Out of bounds.");
}
else{
   if(ind == 0){
     Node* temp = head;
     head = head -> next;
     delete temp;
     length--;
     if(head == nullptr){
       tail = nullptr;
       return;
     } 
   }
   else{
      Node* previous = head;
      for(int i = 0;i < ind-1;i++){
         previous = previous -> next;
      }
      Node* deletedNode = previous -> next;
      previous -> next = deletedNode -> next;
      if(deletedNode -> next == nullptr){
        tail = previous;
        tail -> next;
      }
      delete deletedNode;
      length--;
   }
}
} 

//Returns the list length

const int getLength() const{
return length;    
}

//Equal operator that checks if two lists has the same length and identical elements

const bool operator==(LinkedListObject<T> lst) const{
if(this -> getLength() != lst.getLength()){  
  return false;  
}
const double eps = 1e-7;
for(int i = 0;i < this->getLength();i++){
   double val = (*this)[i] - lst[i];
   if(abs(val) > eps){
     return false;
   } 
}
return true;    
}

//Gets the first element of the list

const int getFirstElement() const{
return (*this)[0];  
}

//Gets the last element of the list

const int getLastElement() const{
return (*this)[this->getLength() -1];
}

};
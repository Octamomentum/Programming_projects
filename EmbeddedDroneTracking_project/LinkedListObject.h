//This data structure is a linked list built in a node based logic that has copy/move assignment operators and
//methods that can add or remove existing elements. Also, one can print its content and
//retrieve its element on a certain index.

#include <iostream>
#include <stdexcept>
#include <algorithm>
#include <memory>

using namespace std;

template <typename T>

class LinkedListObject{

private:

struct Node{
T data;
unique_ptr<Node> next = nullptr;
Node(const T& val) : data(val), next(nullptr){}
};
unique_ptr<Node> head = nullptr;
Node* tail = nullptr;
int length = 0;

public:

//Constructor and destructor

LinkedListObject(){};

void clearList(){

while(head != nullptr){
    head = move(head -> next);
}
tail = nullptr; 
length = 0;
}

~LinkedListObject(){
clearList();
}

//Copy constructor and its assignment operator

LinkedListObject(const LinkedListObject<T>& original_list){

if(original_list.head == nullptr){ 
  return;
}

head = make_unique<Node>(original_list.head -> data);
length = original_list.length;

Node* currentOriginal = original_list.head -> next.get();
Node* currentCopy = head.get();

while(currentOriginal != nullptr){
    currentCopy -> next = make_unique<Node>(currentOriginal->data);
    currentOriginal = currentOriginal -> next.get();
    currentCopy = currentCopy -> next.get();
}
tail = currentCopy;

}

//Move constructor and its assigned operator 

LinkedListObject(LinkedListObject<T>&& original_list) noexcept{

head = move(original_list.head);
tail = original_list.tail;
length = original_list.length;

original_list.tail = nullptr;
original_list.length = 0;

}

LinkedListObject<T>& operator=(LinkedListObject<T>&& original_list) noexcept{

swap(head,original_list.head);
swap(tail,original_list.tail);
swap(length,original_list.length);
   
return *this;

}

//Method that adds element to the list

void addElement(const T& val){
auto newNode = make_unique<Node>(val);
if(head == nullptr){
  head = move(newNode);  
  tail = head.get();
}
else{
   Node* newNext = newNode.get();
   tail->next = move(newNode);
   tail = newNext; 
}
length++;    
}

//Method that prints the content of a list

void print(){
Node* current = head.get();
cout << "\nPrinting list content...\n";
while(current != nullptr){
    cout << current -> data << "->";
    current = current -> next.get();
}    
}

//An operator that retrieves the element of a list if its exists, otherwise a runtime error
//error is thrown

T& operator[](const int& ind) const{
if(head == nullptr){
   throw runtime_error("The list is empty, add elements."); 
}
   int counter = 0;
   Node* current = head.get();
   while(current != nullptr){
       if(counter == ind){
         return current -> data;
       }
       counter++;
       current = current -> next.get();
   }
   throw invalid_argument("Out of bounds."); 

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
     head = move(head -> next);
     length--;
     if(head == nullptr){
       tail = nullptr;
     } 
   }
   else{
      Node* previous = head.get();
      for(int i = 0;i < ind-1;i++){
         previous = previous -> next.get();
      }
      unique_ptr<Node> deletedNode = move(previous -> next);
      previous -> next = move(deletedNode -> next);
      if(deletedNode -> next == nullptr){
        tail = previous;
      }
      length--;
   }
}
} 

//Returns the list length

const int getLength() const{
return length;    
}

//Equal operator that checks if two lists has the same length and identical elements

bool operator==(const LinkedListObject<T>& lst) const{
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

const T getFirstElement() const{
return (*this)[0];  
}

//Gets the last element of the list

const T getLastElement() const{
return (*this)[this->getLength() -1];
}

};
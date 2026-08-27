/**
 * @file LinkedListObject.h
 * @brief Flat 1d-array using node structure logic which uses both smart and raw pointers, for low and efficient memory allocation.
 * 
 * Copy constructors and assignment operators perform explicit deep copies of existing linked lists by temporary raw pointer management
 * and constant-time pointer swaps, while move constructors and assignment operators use move and pointer swaps
 * to perform move operations to another linked list. Additionally, adding or removing elements of a list has been implemented to 
 * safely change the linked list.*/ 

#include <iostream>
#include <stdexcept>
#include <algorithm>
#include <memory>

template <typename T>

class LinkedListObject{

private:

struct Node{
T data;
std::unique_ptr<Node> next = nullptr;
Node(const T& val) : data(val), next(nullptr){}
};
std::unique_ptr<Node> head = nullptr;
Node* tail = nullptr;
size_t length = 0;
//A maximal length is set to avoid overflow scenarios
size_t max_length = 1e8;

public:

//Default constructor for the linked list

LinkedListObject(){};

//A help method that destructs a linked list
void clearList(){

while(head != nullptr){
    //Construct a temporary node to avoid recursive chain from causing stack crash
    std::unique_ptr<Node> temp = std::move(head -> next);
    head = std::move(temp);
}
tail = nullptr; 
length = 0;
max_length = 1e8;
}

//Destructor for the linked list
~LinkedListObject(){
clearList();
}

//Copy constructor

LinkedListObject(const LinkedListObject<T>& original_list){
if(original_list.head == nullptr){ 
  return;
}

head = std::make_unique<Node>(original_list.head -> data);
length = original_list.length;
max_length = original_list.max_length;
//Temporary raw pointers management to peek in the next node
Node* currentOriginal = original_list.head -> next.get();
Node* currentCopy = head.get();

while(currentOriginal != nullptr){
    currentCopy -> next = std::make_unique<Node>(currentOriginal->data);
    currentOriginal = currentOriginal -> next.get();
    currentCopy = currentCopy -> next.get();
}
tail = currentCopy;

}

//Copy assignment operator
LinkedListObject<T>& operator=(const LinkedListObject<T>& original_list){

if(this != &original_list){
  LinkedListObject<T> temp(original_list);
  std::swap(*this,temp); 
}

return *this;

}

//Move constructor
LinkedListObject(LinkedListObject<T>&& original_list) noexcept{

head = std::move(original_list.head);
tail = original_list.tail;
length = original_list.length;
max_length = original_list.max_length;

original_list.tail = nullptr;
original_list.length = 0;
original_list.max_length = 1e8;

}

//Move assignment operator

LinkedListObject<T>& operator=(LinkedListObject<T>&& original_list) noexcept{

if(this != &original_list){
  std::swap(head,original_list.head);
  std::swap(tail,original_list.tail);
  std::swap(length,original_list.length);
  std::swap(max_length,original_list.max_length);
}
  
return *this;

}

//Method that adds element to a linked list

void addElement(const T& val){
auto newNode = std::make_unique<Node>(val);
if(head == nullptr){
  head = std::move(newNode);  
  tail = head.get();
}
else{
   Node* newNext = newNode.get();
   tail->next = std::move(newNode);
   tail = newNext; 
}
length++;    
}

//Method that prints the content of a list

void print(){
Node* current = head.get();
std::cout << "\nPrinting list content...\n";
while(current != nullptr){
    std::cout << current -> data << "->";
    current = current -> next.get();
}
std::cout << "nullptr\n";    
}

//An operator that accesses element of a list at a certain index
T& operator[](const size_t& ind) const{
//Ensures that negative inputs are invalid
if(ind >= max_length){
  throw std::runtime_error("Negative indices are invalid.");
}
if(head == nullptr){
   throw std::runtime_error("The list is empty, add elements."); 
}
   int counter = 0;
   Node* current = head.get();
   //Increase the counter if the counter doesn't match with the index and update the current node
   while(current != nullptr){
       if(counter == ind){
         return current -> data;
       }
       counter++;
       current = current -> next.get();
   }
   throw std::invalid_argument("Out of bounds."); 

}

//Method that removes element given a valid index

void removeElement(const size_t& ind){
if(head == nullptr){
  throw std::runtime_error("The list is empty.");
}
//Ensures that negative index or an index larger than the list length are forbidden
else if(ind >= max_length || ind >= this -> getLength()){
   throw std::runtime_error("Out of bounds.");
}
else{
   if(ind == 0){
    //Construct a temporary node to avoid recursive chain from causing stack crash
    std::unique_ptr<Node> temp = std::move(head);
     head = std::move(temp->next);
     length--;
     if(head == nullptr){
       tail = nullptr;
     } 
   }
   else{
      Node* previous = head.get();
      //Update the previous node until the index is reached
      for(int i = 0;i < ind-1;i++){
         previous = previous -> next.get();
      }
      //Construct the targeted node to delete so that the previous node is moved through this index to the next after
      std::unique_ptr<Node> deletedNode = std::move(previous -> next);
      previous -> next = std::move(deletedNode -> next);
      //Last node case
      if(deletedNode -> next == nullptr){
        tail = previous;
      }
      length--;
   }
}
} 

//Getter for the length of a linked list
const size_t getLength() const{
return length;    
}

//Equal operator that checks if two lists have the same length and the same elements
bool operator==(const LinkedListObject<T>& lst) const{
if(this -> getLength() != lst.getLength()){  
  return false;  
}
const double eps = 1e-7;
for(size_t i = 0;i < this->getLength();i++){
   double val = (*this)[i] - lst[i];
   if(std::abs(val) > eps){
     return false;
   } 
}

return true;    
}

//Getters for the first and last element of a linked list

const T getFirstElement() const{
return (*this)[0];  
}

const T getLastElement() const{
return (*this)[this->getLength() -1];
}

};
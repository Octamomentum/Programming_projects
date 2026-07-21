#include <string>
#include <iostream>

using namespace std;

template <typename T>

class LightMap{

private:

struct MappedNode{
string keypart;
T data;
MappedNode* nextNode;
MappedNode(string key, T val): keypart(key), data(val), nextNode(nullptr){};
};

MappedNode* head;
int length;

public:

LightMap(): head(nullptr), length(0){};

~LightMap(){

MappedNode* current = head;
while(current != nullptr){
    MappedNode* newNode = current -> nextNode;
    delete current;
    current = newNode;
}
}

LightMap(const LightMap<T>& copied_map){
this -> length = copied_map.length;
head = nullptr;
if(copied_map.head == nullptr){
  return;  
}
head = new MappedNode(copied_map.head -> keypart, copied_map.head -> data);
MappedNode* currentCopy = copied_map.head -> nextNode;
MappedNode* currentOriginal = head;
while(currentCopy != nullptr){
    currentOriginal -> nextNode = new MappedNode(currentCopy -> keypart, currentCopy -> data);
    currentOriginal = currentOriginal -> nextNode;
    currentCopy = currentCopy -> nextNode;
}
}

T& operator[](string key){
MappedNode* current = head;
MappedNode* last = nullptr;
while(current != nullptr){
    if(current -> keypart == key){
      return current -> data;  
    }
    last = current;
    current = current -> nextNode;
}
length++;
MappedNode* newNode = new MappedNode(key, T{});
if(head == nullptr){
   head = newNode; 
}
else{
   last -> nextNode = newNode;
}
return newNode -> data;
}

int getLength() const{
return length;
}

void print() const{
MappedNode* current = head;
while(current != nullptr){
    cout << "Key-value: " << current -> keypart << " " << current -> data << ", ";
    current = current -> nextNode;
}
cout << "nullptr\n";
}

};
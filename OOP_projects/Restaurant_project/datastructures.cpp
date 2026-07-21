#include "LinkedList.h"
#include "LightMap.h"
#include <iostream>
#include <stdexcept>

int main(){
LightMap <double> lm;
lm["Korv"] = 35;
lm.print();
}
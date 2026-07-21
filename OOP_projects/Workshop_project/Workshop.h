#pragma once
#include "LinkedList.h"
#include "Vehicle.h"

using namespace std;

class Workshop{

private:

LinkedList<Vehicle*> vehicles_to_repair;

public:

~Workshop(){
for(int i = 0; i < vehicles_to_repair.getLength();i++){
   delete vehicles_to_repair[i];
}
}

void addVehicleToRepair(Vehicle* v){
vehicles_to_repair.addElement(v);    
}

void ServiceForAllVehicles(){
cout << "Welcome to the service sector of this workshop.\n";
for(int i = 0;i < vehicles_to_repair.getLength();i++){
   cout << "Reg number: " << vehicles_to_repair[i] -> getRegNumber();
   vehicles_to_repair[i] -> performService(); 
}
}

};
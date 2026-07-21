#pragma once
#include "Vehicle.h"


class Car : public Vehicle{

private:

double months_car;
bool HasTurbo;
double cost_turbo;
double total_cost;
bool repaired;

struct Cost{

double inventory_cost_car;
double cost_turbo;

};

Cost cost;

public:

Car(string reg_param, double fixed_cost, double months, double flag): Vehicle(reg_param), months_car(months), HasTurbo(flag){
cost.inventory_cost_car = fixed_cost;
cost_turbo = 2500;
};

void performService() override{
total_cost = cost.inventory_cost_car*months_car + 1500;
cout << "Vehicle type: Car\n";
if(HasTurbo == false){
cout << "This car has no turbo.\n";
cout << "Total cost: " << total_cost << "\n"; 
}
else{
   total_cost += cost_turbo;
   cout << "Total cost: " << total_cost << "\n"; 
}

repaired = true;
}

};
#pragma once
#include <string>
#include <iostream>
using namespace std;

class Vehicle{

protected:

bool isRepaired;
string reg_param;
double total_cost;

public:

Vehicle(string regNumber): isRepaired(false),reg_param(regNumber){};

virtual ~Vehicle(){
cout << "Removing registration number: " << reg_param;
}

string getRegNumber() const{return reg_param;};
bool CheckRepairStatus() const {return isRepaired;};

virtual void performService() = 0;

};
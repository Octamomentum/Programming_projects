#include "NumericalMethodCalcs.h"
#include <iostream>
#include <functional>
#include <tuple>
#include <fstream>
#include <math.h>
#include <cmath>
using namespace std;
LinkedListStructure<double> num_method;
int main(){

auto f = [](double x){
return x*x - 1;
};

auto f_tilde = [](double x, double y){
return 1/(x*y);
};

tuple<LinkedListStructure<int>,LinkedListStructure<double>> NR_sol = num_method.NR_calc(3.0,1e-3,f,1e-5);
LinkedListStructure<int> iter_list_NR = get<0>(NR_sol);
LinkedListStructure<double> x_list_NR = get<1>(NR_sol);
ofstream csv_file("./NumMethodData_NR.csv");
csv_file << "iterations, x-coordinate\n";
for(int i = 0; i < x_list_NR.getLength();i++){
   csv_file << iter_list_NR[i] << "," << x_list_NR[i] << "\n";  
}
csv_file.close();
cout << "NR file done."; 

tuple<double, double, double> known_data {exp(1),2*exp(1),1};
tuple<LinkedListStructure<double>,LinkedListStructure<double>> Euler_sol = num_method.Euler_calc(known_data, 1e-2, f_tilde);
LinkedListStructure<double> x_list_Euler = get<0>(Euler_sol);
LinkedListStructure<double> y_list_Euler = get<1>(Euler_sol);
ofstream csv_file1("./NumMethodData_Euler.csv");
csv_file1 << "x-value, y-value\n";
for(int i = 0; i < x_list_Euler.getLength();i++){
   csv_file1 << x_list_Euler[i] << "," << y_list_Euler[i] << "\n";  
}
csv_file1.close();
cout << "Euler file done.";  

tuple<LinkedListStructure<double>,LinkedListStructure<double>> RK4_sol = num_method.RK4_calc(known_data, 1e-3, f_tilde);
LinkedListStructure<double> x_list_RK4 = get<0>(RK4_sol);
LinkedListStructure<double> y_list_RK4 = get<1>(RK4_sol);
ofstream csv_file2("./NumMethodData_RK4.csv");
csv_file2 << "x-value, y-value\n";
for(int i = 0; i < x_list_RK4.getLength();i++){
   csv_file2 << x_list_RK4[i] << "," << y_list_RK4[i] << "\n";  
}
csv_file2.close();
cout << "RK4 file done."; 

tuple<LinkedListStructure<double>,LinkedListStructure<double>> Heun_sol = num_method.Heun_calc(known_data, 1e-3, f_tilde);
LinkedListStructure<double> x_list_Heun = get<0>(Heun_sol);
LinkedListStructure<double> y_list_Heun = get<1>(Heun_sol);
ofstream csv_file3("./NumMethodData_Heun.csv");
csv_file3 << "x-value, y-value\n";
for(int i = 0; i < x_list_Heun.getLength();i++){
   csv_file3 << x_list_Heun[i] << "," << y_list_Heun[i] << "\n";  
}
csv_file3.close();
cout << "Heun file done."; 

}
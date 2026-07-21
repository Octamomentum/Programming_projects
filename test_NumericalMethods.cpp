#include "NumericalMethods.h"
#include "NumericalMethods.cpp"
#include <functional>
#include <iostream>
#include <vector>
#include <tuple>
#include <math.h>
#include <cmath>
#include <cassert> 
#include <fstream>
using namespace std;

bool isIdentical(double x, double y, double eps){  
   return abs(x-y) < eps;
}

int main(){
    num_method.setData(100,0,3,-1,1e-6,1e-3);
    tuple <double, double, double, double, double, double> data = num_method.getData();
    double x = get<0>(data);
    double x_i = get<1>(data);
    double x_f = get<2>(data);
    double y_i = get<3>(data);
    double h = get<4>(data);
    double eps = get<5>(data);

    auto f1 = [](double x){
        return x*x*x - 125;
    };
     auto f2 = [](double x, double y){
        return exp(x) + y - x - 9;
    };
    tuple <vector<double>,vector<double>> NR_sol = num_method.NR_calc(x,f1,h,eps);
    vector<double> iteration_list = get<0>(NR_sol);
    vector<double> x_list_NR = get<1>(NR_sol);
    cout << " Solution: x = " << x_list_NR.back() << " at Iteration " << iteration_list.back() << "." << "\n";
    assert(isIdentical(x_list_NR.back(),5,eps));
    ofstream csv_file("./SimData_NR.csv");
    csv_file << "iter, x_NR\n";
    for(int i = 0;i < iteration_list.size();i++){
      csv_file << iteration_list[i] << "," << x_list_NR[i] << "\n";
    }
    csv_file.close();
    cout << "NR file done.";

    tuple <vector<double>, vector<double>> Euler_sol = num_method.Euler_calc(x_i,x_f,y_i,h,f2);
    vector<double> x_list_Euler = get<0>(Euler_sol);
    vector<double> y_list_Euler = get<1>(Euler_sol);
    cout << " Solution: y = " << y_list_Euler.back() << " at x = " << x_list_Euler.back() << ".\n";
    assert(isIdentical(y_list_Euler.back(), -147.6843,eps));
    ofstream csv_file1("SimData_Euler.csv");
    csv_file1 << "x_Euler, y_Euler\n";
    for(int i = 0;i < x_list_Euler.size();i++){
      csv_file1 << x_list_Euler[i] << "," << y_list_Euler[i] << "\n";

   } csv_file1.close();
   cout << "Euler file done.";

   tuple <vector<double>, vector<double>> RK4_sol = num_method.RK4_calc(x_i,x_f,y_i,h,f2);
    vector<double> x_list_RK4 = get<0>(RK4_sol);
    vector<double> y_list_RK4 = get<1>(RK4_sol);
    cout << " Solution: y = " << y_list_RK4.back() << " at x = " << x_list_RK4.back() << ".\n";
    assert(isIdentical(y_list_RK4.back(), -147.6843,eps));
    ofstream csv_file2("SimData_RK4.csv");
    csv_file2 << "x_RK4, y_RK4\n";
    for(int i = 0;i < x_list_RK4.size();i++){
      csv_file2 << x_list_RK4[i] << "," << y_list_RK4[i] << "\n";

   } csv_file2.close();
   cout << "RK4 file done.";

   tuple <vector<double>, vector<double>> Heun_sol = num_method.Heun_calc(x_i,x_f,y_i,h,f2);
    vector<double> x_list_Heun = get<0>(Heun_sol);
    vector<double> y_list_Heun = get<1>(Heun_sol);
    cout << " Solution: y = " << y_list_Heun.back() << " at x = " << x_list_Heun.back() << ".\n";
    assert(isIdentical(y_list_Heun.back(), -147.6843,eps));
    ofstream csv_file3("SimData_Heun.csv");
    csv_file3 << "x_Heun, y_Heun\n";
    for(int i = 0;i < x_list_Heun.size();i++){
      csv_file3 << x_list_Heun[i] << "," << y_list_Heun[i] << "\n";

   } csv_file3.close();
   cout << "Heun file done.";

   cout <<"All tests passed.";
   }

#include "NumericalMethods.h"
#include <functional>
#include <iostream>
#include <vector>
#include <tuple>
#include <math.h>
#include <cmath>
using namespace std;
NumericalMethods num_method;

double NumericalMethods:: diff_f(double x,function <double (double)> f,double h){
    return (f(x+h) - f(x-h))/(2*h);    
}

tuple <vector<double>,vector<double>> NumericalMethods:: NR_calc(double x, function <double (double)> f, double h, double eps){
    vector<double> x_list;
    double i = 0;
    vector<double> iter_list;
    iter_list.push_back(i);
    x_list.push_back(x); 
    if(isnan(f(x))){
      cout << "Undefined at " << x << ". The method cannot apply to this equation.";
    }
    else if(h <= 0){
        cout << "Invalid h.";
    } 
    else if (f(x) == 0){
       cout << "You guessed the solution.";  
       return {iter_list, x_list};
    }
    else{
       while(abs(f(x)) >= eps){
           double f_diff = diff_f(x,f,h);
           if(isinf(f_diff) || abs(f_diff) < eps){
             cout << "The method doesn't converge.";
             break;
           }
           x = x - f(x)/f_diff;
           i += 1;
           iter_list.push_back(i);
           x_list.push_back(x);
       }
       cout << "\n";   
    }
    return {iter_list, x_list}; 
}

tuple <vector<double>,vector<double>> NumericalMethods:: Euler_calc(double x_i, double x_f, double y_i, double h, function <double (double, double)> f){
    vector<double> x_list;
    vector<double> y_list;
    int i = 0;
    if(h <= 0 || eps <= 0){
      cout << "The method cannot apply to the problem since the step size is invalid chosen.";  
    }
    else {
       while(x_i < x_f){
           if(isnan(f(x_i,y_i))){
             cout << "The function is undefined at (" << x_i << "," << y_i << ").";
             break; 
           }
           i += 1;
           x_list.push_back(x_i);
           y_list.push_back(y_i);
           y_i = y_i + h*f(x_i,y_i);
           x_i = x_i + h;

       } 
    } 
    return {x_list,y_list};
}

tuple <vector<double>,vector<double>> NumericalMethods:: RK4_calc(double x_i, double x_f, double y_i, double h, function <double (double, double)> f){
    vector<double> x_list;
    vector<double> y_list;
    int i = 0;
    if(h <= 0 || eps <= 0){
      cout << "The method cannot apply to the problem since the step size is invalid chosen.";  
    }
    else {
       while(x_i < x_f){
           if(isnan(f(x_i,y_i))){
             cout << "The function is undefined at (" << x_i << "," << y_i << ").";
             break; 
           }
           i += 1;
           x_list.push_back(x_i);
           y_list.push_back(y_i);
           double k1 = f(x_i,y_i);
           double k2 = f(x_i + h/2, y_i + (k1*h)/2);
           double k3 = f(x_i + h/2, y_i + (k2*h)/2);
           double k4 = f(x_i + h, y_i + h*k3);
           y_i = y_i + (h/6)*(k1 + k4 + 2*(k2 + k3));
           x_i = x_i + h;
           
       } 
    } 
    return {x_list,y_list};
}

tuple <vector<double>,vector<double>> NumericalMethods:: Heun_calc(double x_i, double x_f, double y_i, double h, function <double (double, double)> f){
    vector<double> x_list;
    vector<double> y_list;
    int i = 0;
    if(h <= 0 || eps <= 0){
      cout << "The method cannot apply to the problem since the step size is invalid chosen.";  
    }
    else {
       while(x_i < x_f){
           if(isnan(f(x_i,y_i))){
             cout << "The function is undefined at (" << x_i << "," << y_i << ").";
             break; 
           }
           i += 1;
           x_list.push_back(x_i);
           y_list.push_back(y_i);
           double y_tilde = y_i + h*f(x_i,y_i);
           y_i = y_i + (h/2)*(f(x_i,y_i) + f(x_i + h, y_tilde));
           x_i = x_i + h;
           
       } 
    } 
    return {x_list,y_list};
}
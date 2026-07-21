#ifndef NUMERICALMETHODS_H_
#define NUMERICALMETHODS_H_
using namespace std;
#include <tuple>
#include <functional>
#include <vector>

class NumericalMethods{

private:

double x;
double x_i;
double x_f;
double y_i;
double h;
double eps;


public:

void setData(double x_val,double xi,double xf, double yi, double step, double threshold){
    x = x_val; 
    x_i = xi;
     x_f = xf;
     y_i = yi;
     h = step;
     eps = threshold;
}

tuple <double,double,double,double,double, double> getData(){
    return {x,x_i,x_f,y_i,h,eps};
}

double diff_f(double, function <double (double)>, double);
tuple <vector<double>, vector<double>> NR_calc(double, function <double (double)>, double, double);
tuple <vector<double>, vector<double>> Euler_calc(double,double,double,double, function <double (double,double)>);
tuple <vector<double>, vector<double>> RK4_calc(double,double,double,double, function <double (double,double)>);
tuple <vector<double>, vector<double>> Heun_calc(double,double,double,double, function <double (double, double)>);

};

#endif
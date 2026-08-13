#include <gtest/gtest.h>
#include "UAVTracking.h"
#include <iostream>
#include <tuple>
#include <functional>

MatrixCalculations<double,int> matcalc;
UAVTracking <double,int> uav_tracking;
using namespace std;

TEST(TrackUAVs,MultipleUAV){   
   uav_tracking.UAVTracking_Solver();
}
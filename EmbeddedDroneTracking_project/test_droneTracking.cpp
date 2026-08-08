#include <gtest/gtest.h>
#include "UAVTracking.h"
#include <iostream>
#include <tuple>
#include <functional>

MatrixCalculations<double> matcalc;
UAVTracking <double,int> uav_tracking;


using namespace std;

TEST(TrackUAVs,OneUAV){
   int numofuavs = 1;
   int numoflms = 30;

   tuple<double,double> axis_range{-10,10};
   int l_seed = 1;
   int ref_seed = 5;
   tuple<int,int> seeds{l_seed,ref_seed};

   MatrixObject<double> uav_guess(3,1); 

   auto [L,uav_ref] = uav_tracking.generateRadar(numoflms,axis_range,seeds);

   uav_guess(0,0) = -10000.0;
   uav_guess(1,0) = -120000.0;
   uav_guess(2,0) = -103200.0;

   cout << "\nLandmarks, locations\n";
   L.print();
   cout << "\nReference point, locations\n";
   uav_ref.print();
   MatrixObject<double> d_truth = uav_tracking.distanceMetric(L,uav_ref);
   cout << "\nReference point, distances to landmarks\n";
   d_truth.print();  
   cout << "\nUAV, Initial guess\n";
   uav_guess.print();
   MatrixObject<double> d_meas = uav_tracking.distanceMetric(L,uav_guess);
   cout << "\nEstimated distances\n";
   d_meas.print();

   uav_tracking.UAVTracking_Solver(numofuavs,L,d_truth,uav_guess);
}
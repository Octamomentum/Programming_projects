#include <gtest/gtest.h>
#include "UAVTracking.h"
#include <iostream>
#include <tuple>
#include <functional>

MatrixCalculations<double> matcalc;
UAVTracking <double,int> uav_tracking;


using namespace std;

TEST(TrackUAVs,MultipleUAVs){
   int numofuavs = 7;
   MatrixObject<double> L(4,3);
   MatrixObject<double> dist_meas(4,1);
   MatrixObject<double> uav_sensors(3,1);
   int time_limit = 21600;
   tuple<MatrixObject<double>,MatrixObject<double>> landmarkParams {L,dist_meas};

   L(0,0) = 1.0;
   L(0,1) = 2.0;
   L(0,2) = 3.0;
   L(1,0) = 4.0;
   L(1,1) = -8.0;
   L(1,2) = -9.0;
   L(2,0) = 1.0;
   L(2,1) = -1.0;
   L(2,2) = 1.0;
   L(3,0) = 1.0;
   L(3,1) = -1.0;
   L(3,2) = 3.0;

   dist_meas(0,0) = 0.5;
   dist_meas(1,0) = 1.0;
   dist_meas(2,0) = 2.0;
   dist_meas(3,0) = 1.0;

   uav_sensors(0,0) = -3.0;
   uav_sensors(1,0) = -2.0;
   uav_sensors(1,0) = 1.0;

   uav_tracking.UAVTracking_Solver(numofuavs,L,dist_meas,uav_sensors,time_limit);
}

TEST(TrackUAVs,OneUAV){
   int numofuavs = 1;
   MatrixObject<double> L(3,3);
   MatrixObject<double> dist_meas(4,1);
   MatrixObject<double> uav_sensors(3,1);
   MatrixObject<double> uav_guess(3,1); 
   int time_limit = 1000000;
   tuple<MatrixObject<double>,MatrixObject<double>> landmarkParams {L,dist_meas};

   L(0,0) = 1.0;
   L(0,1) = 0.0;
   L(0,2) = 0.0;
   L(1,0) = -1.0;
   L(1,1) = 1.0;
   L(1,2) = 1.0;
   L(2,0) = 0.0;
   L(2,1) = 0.0;
   L(2,2) = 1.0;

   uav_guess(0,0) = 1.0;
   uav_guess(1,0) = 0.0;
   uav_guess(2,0) = 1.0;

   uav_sensors(0,0) = 1.0;
   uav_sensors(1,0) = 0.0;
   uav_sensors(2,0) = 0.0;

   MatrixObject<double> d_meas = uav_tracking.distanceMetric(L,uav_sensors);
   MatrixObject<double> d_init = uav_tracking.distanceMetric(L,uav_guess);
   cout << "\n Distances, Ground Truth locations: \n";
   d_meas.print();
   cout << "\nDistances, Initial guess: \n";
   d_init.print();
   uav_tracking.UAVTracking_Solver(numofuavs,L,dist_meas,uav_guess,time_limit);
}
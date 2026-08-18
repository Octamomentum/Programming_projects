#include <gtest/gtest.h>
#include "UAVTracking.h"

UAVTracking <double,int> uav_tracking;

TEST(TrackUAVs,MultipleUAV){   
   uav_tracking.UAVTracking_Solver();
}
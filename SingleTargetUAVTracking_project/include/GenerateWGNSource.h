/**
 * @file GenerateWGNSource.h
 * @brief Handles the method that generates White Gaussian Noise to true distances of the UAV positions relative to the landmarks.

 * */ 

#pragma once

#include "MatrixObject.h"
#include <random>
#include <string>

template<typename T>
using Matrix = MatrixObject<T>;

template <typename T>
class GenerateWGNSource{

public:



//Add WGN to the true distances of UAV positions relative to the landmarks

void AddingWGNToTrueDistances(Matrix<T>& dist_truth, const size_t& var_ind,const T& variance)const{

std::normal_distribution wgn(0.0,std::sqrt(variance));
std::mt19937 gen(var_ind);

for(size_t i = 0;i < dist_truth.getRows();i++){
   dist_truth(i,0) += wgn(gen);
}

}

};


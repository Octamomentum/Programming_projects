/**
 * @file GenerateWGNSource.h
 * @brief Handles the method that generates White Gaussian Noise.

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

//Generate WGN to true distances of the radar anchor points relative to landmark locations

void GenerateWGN(Matrix<T>& dist,const size_t& var_ind, const T& variance)const {
std::mt19937 gen(var_ind);
std::normal_distribution wgn(0.0,std::sqrt(variance));
for(size_t i = 0;i < dist.getRows();i++){
   dist(i,0) += wgn(gen);
}
}

};


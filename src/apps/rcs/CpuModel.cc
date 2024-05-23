/*
 * CpuModel.cpp
 *
 *  Created on: May 6, 2024
 *      Author: veins
 */
#include "CpuModel.h"
#include <vector>
#include <random>

CpuModel::CpuModel(int numCores) {
    init(numCores);
}

CpuModel::CpuModel() {
    CpuModel(1);
}

void CpuModel::init(int numCores) {
    this->numCores = numCores;
    coreLoads = std::vector<double>(numCores);
}

double CpuModel::randomGaussian(double mean, double stddev) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<float> d(mean, stddev);
    double sample = d(gen);
    return sample;
}

double CpuModel::getLatency(double currentTime, double mean, double std) {
    int selectCoreId = -1;
    double earliestTime = 1000000.0;
    for (int i = 0; i < coreLoads.size(); i++) {
        if (coreLoads[i] < currentTime) coreLoads[i] = currentTime;
        if (coreLoads[i] < earliestTime) {
            earliestTime = coreLoads[i];
            selectCoreId = i;
        }
    }
    coreLoads[selectCoreId] = coreLoads[selectCoreId] + randomGaussian(mean, std);
    return coreLoads[selectCoreId] - currentTime;
}



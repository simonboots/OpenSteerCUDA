#include "SteerForPursuit.h"
#include <cuda_runtime.h>
#include "OpenSteer/VehicleData.h"
#include "OpenSteer/PursuitDataProvider.h"
#include "CUDAKernelOptions.cu"
#include <iostream>

using namespace OpenSteer;
using namespace std;

__global__ void
steerForPursuitKernel(VehicleData *vehicleData, float3 *quarryPosition, float3 *quarryVelocity, float3 *steeringVectors, float maxPredictionTime, float weight, kernel_options options);

OpenSteer::SteerForPursuit::SteerForPursuit(PursuitDataProvider* pursuitDataProvider, float maxPredictionTime, float weight, kernel_options options)
{
    threadsPerBlock = 128;
    this->weight = weight;
    this->options = options;
    this->pursuitDataProvider = pursuitDataProvider;
    this->maxPredictionTime = maxPredictionTime;
}

OpenSteer::SteerForPursuit::~SteerForPursuit() {}

void OpenSteer::SteerForPursuit::init()
{
    const float h_timeFactorTable[9] = {2.f, 4.f, 0.85f, 2.f, 0.8f, 1.8f, 0.5f, 1.f, 4.f};
    cudaMemcpyToSymbol("timeFactorTable", h_timeFactorTable, sizeof(float) * 9, 0, cudaMemcpyHostToDevice);
}

void OpenSteer::SteerForPursuit::run()
{
    steerForPursuitKernel<<<gridDim(), blockDim()>>>(getVehicleData(), pursuitDataProvider->getPursuitPosition(), pursuitDataProvider->getPursuitVelocity(), getSteeringVectors(), maxPredictionTime, weight, options);
}

void OpenSteer::SteerForPursuit::close()
{
    // nothing to do
}

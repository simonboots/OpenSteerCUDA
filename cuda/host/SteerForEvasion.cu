#include "SteerForEvasion.h"
#include <cuda_runtime.h>
#include "OpenSteer/VehicleData.h"
#include "OpenSteer/PursuitDataProvider.h"
#include "CUDAKernelOptions.cu"
#include <iostream>

using namespace OpenSteer;
using namespace std;

__global__ void
steerForEvasionKernel(VehicleData *vehicleData, float3 *menacePosition, float3 *menaceVelocity, float3 *steeringVectors, float maxPredictionTime, float weight, kernel_options options);

OpenSteer::SteerForEvasion::SteerForEvasion(PursuitDataProvider* pursuitDataProvider, float maxPredictionTime, float weight, kernel_options options)
{
    threadsPerBlock = 128;
    this->weight = weight;
    this->options = options;
    this->pursuitDataProvider = pursuitDataProvider;
    this->maxPredictionTime = maxPredictionTime;
}

OpenSteer::SteerForEvasion::~SteerForEvasion() {}

void OpenSteer::SteerForEvasion::init()
{
    // nothing to do
}

void OpenSteer::SteerForEvasion::run()
{
    steerForEvasionKernel<<<gridDim(), blockDim()>>>(getVehicleData(), pursuitDataProvider->getPursuitPosition(), pursuitDataProvider->getPursuitVelocity(), getSteeringVectors(), maxPredictionTime, weight, options);
}

void OpenSteer::SteerForEvasion::close()
{
    // nothing to do
}

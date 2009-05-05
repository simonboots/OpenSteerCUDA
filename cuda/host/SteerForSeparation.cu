#include "SteerForSeparation.h"
#include <cuda_runtime.h>
#include "OpenSteer/VehicleData.h"
#include "OpenSteer/NeighborDataProvider.h"
#include "CUDAKernelOptions.cu"
#include <iostream>

using namespace OpenSteer;
using namespace std;

__global__ void
steerForSeparationKernel(VehicleData *vehicleData, VehicleConst *vehicleConst, float3 *steeringVectors, float maxDistance, float cosMaxAngle, NeighborData* neighborData, float weight, kernel_options options);

OpenSteer::SteerForSeparation::SteerForSeparation(NeighborDataProvider* pursuitDataProvider, float maxDistance, float cosMaxAngle, float weight, kernel_options options)
{
    threadsPerBlock = 128;
    this->weight = weight;
    this->options = options;
    this->neighborDataProvider = neighborDataProvider;
    this->maxDistance = maxDistance;
    this->cosMaxAngle = cosMaxAngle;
}

OpenSteer::SteerForSeparation::~SteerForSeparation() {}

void OpenSteer::SteerForSeparation::init()
{
    // nothing to do
}

void OpenSteer::SteerForSeparation::run()
{
    steerForSeparationKernel<<<gridDim(), blockDim()>>>(getVehicleData(), getVehicleConst(), getSteeringVectors(), maxDistance, cosMaxAngle, neighborDataProvider->getNeighborData(), weight, options);
}

void OpenSteer::SteerForSeparation::close()
{
    // nothing to do
}

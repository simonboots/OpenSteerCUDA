#include "SteerToAvoidCloseNeighbors.h"
#include <cuda_runtime.h>
#include "OpenSteer/VehicleData.h"
#include "OpenSteer/NeighborData.h"
#include "CUDAKernelOptions.cu"
#include <iostream>

using namespace OpenSteer;
using namespace std;

__global__ void
steerToAvoidCloseNeighbors(VehicleData *vehicleData, VehicleConst *vehicleConst, float3 *steeringVectors, NeighborData *neighbors, float minSeparationDistance, float weight, kernel_options options);

OpenSteer::SteerToAvoidCloseNeighbors::SteerToAvoidCloseNeighbors(NeighborDataProvider *neighborDataProvider, float minSeparationDistance, float weight, kernel_options options)
{
    threadsPerBlock = 128;
    this->weight = weight;
    this->options = options;
    this->neighborDataProvider = neighborDataProvider;
    this->minSeparationDistance = minSeparationDistance;
}

OpenSteer::SteerToAvoidCloseNeighbors::~SteerToAvoidCloseNeighbors() {}

void OpenSteer::SteerToAvoidCloseNeighbors::init()
{
    // nothing to do
}

void OpenSteer::SteerToAvoidCloseNeighbors::run()
{
    steerToAvoidCloseNeighbors<<<gridDim(), blockDim()>>>(getVehicleData(), getVehicleConst(), getSteeringVectors(), neighborDataProvider->getNeighborData(), minSeparationDistance, weight, options);
}

void OpenSteer::SteerToAvoidCloseNeighbors::close()
{
    // nothing to do
}


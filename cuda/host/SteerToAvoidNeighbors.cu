#include "SteerToAvoidNeighbors.h"
#include <cuda_runtime.h>
#include "OpenSteer/VehicleData.h"
#include "OpenSteer/NeighborData.h"
#include "CUDAKernelOptions.cu"
#include <iostream>

using namespace OpenSteer;
using namespace std;

__global__ void
steerToAvoidNeighbors(VehicleData *vehicleData, VehicleConst *vehicleConst, float3 *steeringVectors, NeighborData *neighbors, float minTimeToCollision, float weight, kernel_options options);

OpenSteer::SteerToAvoidNeighbors::SteerToAvoidNeighbors(NeighborDataProvider *neighborDataProvider, float minTimeToCollision, float weight, kernel_options options)
{
    threadsPerBlock = 128;
    this->weight = weight;
    this->options = options;
    this->neighborDataProvider = neighborDataProvider;
    this->minTimeToCollision = minTimeToCollision;
}

OpenSteer::SteerToAvoidNeighbors::~SteerToAvoidNeighbors() {}

void OpenSteer::SteerToAvoidNeighbors::init()
{
    // nothing to do
}

void OpenSteer::SteerToAvoidNeighbors::run()
{
    steerToAvoidNeighbors<<<gridDim(), blockDim()>>>(getVehicleData(), getVehicleConst(), getSteeringVectors(), neighborDataProvider->getNeighborData(), minTimeToCollision, weight, options);
}

void OpenSteer::SteerToAvoidNeighbors::close()
{
    // nothing to do
}


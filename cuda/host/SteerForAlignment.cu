#include "SteerForAlignment.h"
#include <cuda_runtime.h>
#include "OpenSteer/VehicleData.h"
#include "OpenSteer/NeighborDataProvider.h"
#include "CUDAKernelOptions.cu"
#include <iostream>

using namespace OpenSteer;
using namespace std;

__global__ void
steerForAlignmentKernel(VehicleData *vehicleData, VehicleConst *vehicleConst, float3 *steeringVectors, float maxDistance, float cosMaxAngle, NeighborData* neighborData, float weight, kernel_options options);

OpenSteer::SteerForAlignment::SteerForAlignment(NeighborDataProvider*neighborDataProvider, float maxDistance, float cosMaxAngle, float weight, kernel_options options)
{
    threadsPerBlock = 128;
    this->weight = weight;
    this->options = options;
    this->neighborDataProvider = neighborDataProvider;
    this->maxDistance = maxDistance;
    this->cosMaxAngle = cosMaxAngle;
}

OpenSteer::SteerForAlignment::~SteerForAlignment() {}

void OpenSteer::SteerForAlignment::init()
{
    // nothing to do
}

void OpenSteer::SteerForAlignment::run()
{
    steerForAlignmentKernel<<<gridDim(), blockDim()>>>(getVehicleData(), getVehicleConst(), getSteeringVectors(), maxDistance, cosMaxAngle, neighborDataProvider->getNeighborData(), weight, options);
}

void OpenSteer::SteerForAlignment::close()
{
    // nothing to do
}

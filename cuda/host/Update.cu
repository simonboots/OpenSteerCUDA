#include "Update.h"
#include <cuda_runtime.h>
#include "OpenSteer/VehicleData.h"
#include "CUDAKernelOptions.cu"
#include <iostream>

using namespace OpenSteer;
using namespace std;

__global__ void
updateKernel(VehicleData *vehicleData, VehicleConst *vehicleConst, float3 *steeringVectors, float elapsedTime, kernel_options options);


OpenSteer::Update::Update(kernel_options options)
{
    threadsPerBlock = 128;
    this->options = options;
}

OpenSteer::Update::~Update() {}

void OpenSteer::Update::init()
{
    // nothing to do
}

void OpenSteer::Update::run()
{
    updateKernel<<<gridDim(), blockDim()>>>(getVehicleData(), getVehicleConst(), getSteeringVectors(), getElapsedTime(), options);
}

void OpenSteer::Update::close()
{
    // nothing to do
}

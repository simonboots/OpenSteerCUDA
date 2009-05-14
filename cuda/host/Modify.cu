#include "Modify.h"
#include <cuda_runtime.h>
#include "OpenSteer/VehicleData.h"
#include "CUDAKernelOptions.cu"
#include <iostream>

using namespace OpenSteer;
using namespace std;

__global__ void
modifyKernel(VehicleData *vehicleData, VehicleConst *vehicleConst, float3 *steeringVectors, float elapsedTime, kernel_options options);


OpenSteer::Modify::Modify(kernel_options options)
{
    threadsPerBlock = 128;
    this->options = options;
}

OpenSteer::Modify::~Modify() {}

void OpenSteer::Modify::init()
{
    // nothing to do
}

void OpenSteer::Modify::run()
{
    modifyKernel<<<gridDim(), blockDim()>>>(getVehicleData(), getVehicleConst(), getSteeringVectors(), getElapsedTime(), options);
}

void OpenSteer::Modify::close()
{
    // nothing to do
}

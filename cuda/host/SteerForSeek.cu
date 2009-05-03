#include "SteerForSeek.h"
#include <cuda_runtime.h>
#include "OpenSteer/VehicleData.h"
#include "OpenSteer/SeekVectorProviderCUDAKernel.h"
#include "CUDAKernelOptions.cu"
#include <iostream>

using namespace OpenSteer;
using namespace std;

__global__ void
steerForSeekKernel(VehicleData *vehicleData, float3 *seekVectors, float3 *steeringVectors, float weight, kernel_options options);

OpenSteer::SteerForSeek::SteerForSeek(SeekVectorProviderCUDAKernel* seekVectorProvider, float weight, kernel_options options)
{
    threadsPerBlock = 128;
    this->seekVectorProvider = seekVectorProvider;
    this->weight = weight;
    this->options = options;
}

OpenSteer::SteerForSeek::~SteerForSeek() {}

void OpenSteer::SteerForSeek::init()
{
    // nothing to do
}

void OpenSteer::SteerForSeek::run()
{
    steerForSeekKernel<<<gridDim(), blockDim()>>>(getVehicleData(), seekVectorProvider->getSeekVectors(), getSteeringVectors(), weight, options);
}

void OpenSteer::SteerForSeek::close()
{
    // nothing to do
}

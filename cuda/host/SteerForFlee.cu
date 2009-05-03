#include "SteerForFlee.h"
#include <cuda_runtime.h>
#include "OpenSteer/VehicleData.h"
#include "OpenSteer/SeekVectorProviderCUDAKernel.h"
#include "CUDAKernelOptions.cu"
#include <iostream>

using namespace OpenSteer;
using namespace std;

__global__ void
steerForFleeKernel(VehicleData *vehicleData, float3 *fleeVectors, float3 *steeringVectors, float weight, kernel_options options);

OpenSteer::SteerForFlee::SteerForFlee(SeekVectorProviderCUDAKernel* seekVectorProvider, float weight, kernel_options options)
{
    threadsPerBlock = 128;
    this->seekVectorProvider = seekVectorProvider;
    this->weight = weight;
    this->options = options;
}

OpenSteer::SteerForFlee::~SteerForFlee() {}

void OpenSteer::SteerForFlee::init()
{
    // nothing to do
}

void OpenSteer::SteerForFlee::run()
{
    steerForFleeKernel<<<gridDim(), blockDim()>>>(getVehicleData(), seekVectorProvider->getSeekVectors(), getSteeringVectors(), weight, options);
}

void OpenSteer::SteerForFlee::close()
{
    // nothing to do
}

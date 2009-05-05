#include "SteerForTargetSpeed.h"
#include <cuda_runtime.h>
#include "OpenSteer/VehicleData.h"
#include "OpenSteer/SeekVectorProvider.h"
#include "CUDAKernelOptions.cu"
#include <iostream>

using namespace OpenSteer;
using namespace std;

__global__ void
steerForTargetSpeedKernel(VehicleData *vehicleData, VehicleConst *vehicleConst, float *targetSpeeds, float3 *steeringVectors, float weight, kernel_options options);

OpenSteer::SteerForTargetSpeed::SteerForTargetSpeed(TargetSpeedProvider* targetSpeedProvider, float weight, kernel_options options)
{
    threadsPerBlock = 128;
    this->targetSpeedProvider = targetSpeedProvider;
    this->weight = weight;
    this->options = options;
}

OpenSteer::SteerForTargetSpeed::~SteerForTargetSpeed() {}

void OpenSteer::SteerForTargetSpeed::init()
{
    // nothing to do
}

void OpenSteer::SteerForTargetSpeed::run()
{
    steerForTargetSpeedKernel<<<gridDim(), blockDim()>>>(getVehicleData(), getVehicleConst(), targetSpeedProvider->getTargetSpeeds(), getSteeringVectors(), weight, options);
}

void OpenSteer::SteerForTargetSpeed::close()
{
    // nothing to do
}

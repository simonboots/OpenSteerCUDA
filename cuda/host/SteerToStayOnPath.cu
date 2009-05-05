#include "SteerToStayOnPath.h"
#include <cuda_runtime.h>
#include "OpenSteer/VehicleData.h"
#include "OpenSteer/PathwayData.h"
#include "CUDAKernelOptions.cu"
#include <iostream>

using namespace OpenSteer;
using namespace std;

__global__ void
steerToStayOnPathKernel(VehicleData *vehicleData, float3 *steeringVectors, float predictionTime, float weight, kernel_options options);

OpenSteer::SteerToStayOnPath::SteerToStayOnPath(float predictionTime, float weight, kernel_options options)
{
    threadsPerBlock = 128;
    this->weight = weight;
    this->options = options;
    this->predictionTime = predictionTime;
}

OpenSteer::SteerToStayOnPath::~SteerToStayOnPath() {}

void OpenSteer::SteerToStayOnPath::init()
{
    // fill pathwayData with empty pathway
    PathwayData *pwData = new PathwayData();
    pwData->numElements = 0;
    pwData->isCyclic = 0;
    pwData->radius = 1.f;
    
    cudaMemcpyToSymbol("pathway", pwData, sizeof(PathwayData), 0, cudaMemcpyHostToDevice);
    
    delete pwData;
}

void OpenSteer::SteerToStayOnPath::run()
{
    steerToStayOnPathKernel<<<gridDim(), blockDim()>>>(getVehicleData(), getSteeringVectors(), predictionTime, weight, options);
}

void OpenSteer::SteerToStayOnPath::close()
{
    // nothing to do
}

void OpenSteer::SteerToStayOnPath::setPathwayData(PathwayData* pathwayData)
{
    cudaMemcpyToSymbol("pathway", pathwayData, sizeof(PathwayData), 0, cudaMemcpyHostToDevice);
}


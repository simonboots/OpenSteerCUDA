#include "SteerToFollowPath.h"
#include <cuda_runtime.h>
#include "OpenSteer/VehicleData.h"
#include "OpenSteer/PathwayData.h"
#include "OpenSteer/PathwayDataFunc.h"
#include "CUDAKernelOptions.cu"
#include <iostream>

using namespace OpenSteer;
using namespace std;

__global__ void
steerToFollowPathKernel(VehicleData *vehicleData, float3 *steeringVectors, int *direction, float predictionTime, float weight, kernel_options options);

OpenSteer::SteerToFollowPath::SteerToFollowPath(float predictionTime, float weight, kernel_options options)
{
    d_directions = NULL;
    threadsPerBlock = 128;
    this->weight = weight;
    this->options = options;
    this->predictionTime = predictionTime;
}

OpenSteer::SteerToFollowPath::~SteerToFollowPath() {}

void OpenSteer::SteerToFollowPath::init()
{
    // fill pathwayData with empty pathway
    PathwayData *pwData = new PathwayData();
    pwData->numElements = 0;
    pwData->isCyclic = 0;
    pwData->radius = 1.f;
    
    cudaMemcpyToSymbol("followPathway", pwData, sizeof(PathwayData), 0, cudaMemcpyHostToDevice);
    
    delete pwData;
    
    // device memory for directions
    mem_size_directions = getNumberOfAgents()*sizeof(int);
    cudaError_t retval = cudaMalloc((void **)&d_directions, mem_size_directions);
    if (retval != cudaSuccess)
        cout << "Error while allocating d_seekVectors memory: " << cudaGetErrorString(retval) << endl;
    
    // seems to be buggy? please use setDirections(int) to set directions
    retval = cudaMemset(d_directions, 1, mem_size_directions);
}

void OpenSteer::SteerToFollowPath::run()
{
    steerToFollowPathKernel<<<gridDim(), blockDim()>>>(getVehicleData(), getSteeringVectors(), d_directions, predictionTime, weight, options);
}

void OpenSteer::SteerToFollowPath::close()
{
    // nothing to do
}

void OpenSteer::SteerToFollowPath::setPathwayData(PolylinePathway& pathway)
{
    PathwayData *pwData = transformPathway(pathway);
    cudaMemcpyToSymbol("followPathway", pwData, sizeof(PathwayData), 0, cudaMemcpyHostToDevice);
    delete pwData;
}

void OpenSteer::SteerToFollowPath::setDirections(int *directions)
{
    cudaMemcpy(d_directions, directions, mem_size_directions, cudaMemcpyHostToDevice);
}


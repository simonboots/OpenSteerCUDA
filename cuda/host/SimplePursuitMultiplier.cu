#include "SimplePursuitMultiplier.h"
#include <cuda_runtime.h>
#include "OpenSteer/VehicleData.h"
#include "CUDAKernelOptions.cu"
#include <iostream>

using namespace OpenSteer;
using namespace std;

__global__ void
simplePursuitMultiplierKernel(float3 position, float3 velocity, float3 *positions, float3 *velocities);

OpenSteer::SimplePursuitMultiplier::SimplePursuitMultiplier()
{
    d_pursuitVelocity = NULL;
    d_pursuitPosition = NULL;
    threadsPerBlock = 128;
    change = true;
}

OpenSteer::SimplePursuitMultiplier::~SimplePursuitMultiplier() {}

void OpenSteer::SimplePursuitMultiplier::init()
{
    // device memory for position data
    mem_size_position = getNumberOfAgents()*sizeof(float3);
    cudaError_t retval = cudaMalloc((void **)&d_pursuitPosition, mem_size_position);
    if (retval != cudaSuccess)
        cout << "Error while allocating d_pursuitPosition memory: " << cudaGetErrorString(retval) << endl;
    
    // device memory for velocity data
    mem_size_velocity = getNumberOfAgents()*sizeof(float3);
    retval = cudaMalloc((void **)&d_pursuitVelocity, mem_size_velocity);
    if (retval != cudaSuccess)
        cout << "Error while allocating d_pursuitVelocity memory: " << cudaGetErrorString(retval) << endl;
}

void OpenSteer::SimplePursuitMultiplier::run()
{
    if (change) {
        simplePursuitMultiplierKernel<<<gridDim(), blockDim()>>>(newPosition, newVelocity, d_pursuitPosition, d_pursuitVelocity);
        change = false;
    }
}

void OpenSteer::SimplePursuitMultiplier::close()
{
    if (d_pursuitVelocity != NULL) {
        cudaFree(d_pursuitVelocity);
        d_pursuitVelocity = NULL;        
    }
    
    if (d_pursuitPosition != NULL) {
        cudaFree(d_pursuitPosition);
        d_pursuitPosition = NULL;
    }
}

void OpenSteer::SimplePursuitMultiplier::setNewPosition(float3 newPosition)
{
    this->newPosition = newPosition;
    change = true;
}

void OpenSteer::SimplePursuitMultiplier::setNewVelocity(float3 newVelocity)
{
    this->newVelocity= newVelocity;
    change = true;
}
#include "FindFollower.h"
#include <cuda_runtime.h>
#include "OpenSteer/VehicleData.h"
#include "CUDAKernelOptions.cu"
#include <iostream>

using namespace OpenSteer;
using namespace std;

__global__ void
findFollowerKernel(VehicleData *vehicleData, float3 *seekVectors);

OpenSteer::FindFollower::FindFollower()
{
    d_seekVectors = NULL;
    threadsPerBlock = 128;
}

OpenSteer::FindFollower::~FindFollower() {}

void OpenSteer::FindFollower::init()
{
    // device memory for seek vector
    mem_size_seek_vectors = getNumberOfAgents()*sizeof(float3);
    cudaError_t retval = cudaMalloc((void **)&d_seekVectors, mem_size_seek_vectors);
    if (retval != cudaSuccess)
        cout << "Error while allocating d_seekVectors memory: " << cudaGetErrorString(retval) << endl;
}

void OpenSteer::FindFollower::run()
{    
    findFollowerKernel<<<gridDim(), blockDim()>>>(getVehicleData(), d_seekVectors);
}

void OpenSteer::FindFollower::close()
{
    if (d_seekVectors != NULL) {
        cudaFree(d_seekVectors);
        d_seekVectors = NULL;        
    }
}

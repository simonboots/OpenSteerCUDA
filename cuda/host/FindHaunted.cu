#include "FindHaunted.h"
#include <cuda_runtime.h>
#include "OpenSteer/VehicleData.h"
#include "CUDAKernelOptions.cu"
#include <iostream>

using namespace OpenSteer;
using namespace std;

__global__ void
findHauntedKernel(VehicleData *vehicleData, float3 *seekVectors, unsigned int stride);

OpenSteer::FindHaunted::FindHaunted(unsigned int stride)
{
    d_seekVectors = NULL;
    threadsPerBlock = 128;
    this->stride = stride;
}

OpenSteer::FindHaunted::~FindHaunted() {}

void OpenSteer::FindHaunted::init()
{
    // device memory for seek vector
    mem_size_seek_vectors = getNumberOfAgents()*sizeof(float3);
    cudaError_t retval = cudaMalloc((void **)&d_seekVectors, mem_size_seek_vectors);
    if (retval != cudaSuccess)
        cout << "Error while allocating d_seekVectors memory: " << cudaGetErrorString(retval) << endl;
}

void OpenSteer::FindHaunted::run()
{    
    findHauntedKernel<<<gridDim(), blockDim()>>>(getVehicleData(), d_seekVectors, stride);
}

void OpenSteer::FindHaunted::close()
{
    if (d_seekVectors != NULL) {
        cudaFree(d_seekVectors);
        d_seekVectors = NULL;        
    }
}

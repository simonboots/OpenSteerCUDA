#ifndef _RESET_ON_COLLISION_KERNEL_H_
#define _RESET_ON_COLLISION_KERNEL_H_

#include "VehicleData.h"
#include "CUDAKernelOptions.cu"

#define CHECK_BANK_CONFLICTS 0
#if CHECK_BANK_CONFLICTS
#define P_F(i) (CUT_BANK_CHECKER(((float*)position), i))
#define P(i) (CUT_BANK_CHECKER(position, i))
#define R(i) (CUT_BANK_CHECKER(random, i))
#else
#define P_F(i) ((float*)position)[i]
#define P(i) position[i]
#define R(i) random[i]
#endif

__global__ void
resetOnCollisionKernel(VehicleData *vehicleData, float *random, float3 position, float radius)
{
    int id = (blockIdx.x * blockDim.x + threadIdx.x);
    int blockOffset = (blockDim.x * blockIdx.x * 3);
    int numOfAgents = gridDim.x * blockDim.x;

    // shared memory for position
    __shared__ float3 position[TPB];
    
    // shared memory for random numbers
    __shared__ float random[TPB];
    __shared__ int rindex[TPB];
    
    rindex[threadIdx.x] = threadIdx.x; // init random indices
    
    // copy position data from global memory (coalesced)
    P_F(threadIdx.x) = ((float*)(*vehicleData).position)[blockOffset + threadIdx.x];
    P_F(threadIdx.x + blockDim.x) = ((float*)(*vehicleData).position)[blockOffset + threadIdx.x + blockDim.x];
    P_F(threadIdx.x + 2*blockDim.x) = ((float*)(*vehicleData).position)[blockOffset + threadIdx.x + 2*blockDim.x];
    
    // copy random numbers from global memory (coalesced)
    R(threadIdx.x) = random[id];
    
    float d = float3Distance(P(threadIdx.x), position);
    float r = radius + (*vehicleData).radius[id];
    
    if (d < r) {
        // 
    }
    
}   

#endif // _RESET_ON_COLLISION_KERNEL_H_
#ifndef _STEER_FOR_FLEE_KERNEL_CU_
#define _STEER_FOR_FLEE_KERNEL_CU_

#include "VehicleData.h"
#include "CUDAVectorUtilities.cu"
#include "CUDAKernelOptions.cu"

#define CHECK_BANK_CONFLICTS 0
#if CHECK_BANK_CONFLICTS
#define F_F(i) (CUT_BANK_CHECKER(((float*)forward), i))
#define P_F(i) (CUT_BANK_CHECKER(((float*)position), i))
#define FL_F(i) (CUT_BANK_CHECKER(((float*)flee), i))
#define F(i) (CUT_BANK_CHECKER(forward, i))
#define P(i) (CUT_BANK_CHECKER(position, i))
#define FL(i) (CUT_BANK_CHECKER(flee, i))
#else
#define F_F(i) ((float*)forward)[i]
#define P_F(i) ((float*)position)[i]
#define FL_F(i) ((float*)flee)[i]
#define F(i) forward[i]
#define P(i) position[i]
#define FL(i) flee[i]
#endif

__global__ void
steerForFleeKernel(VehicleData *vehicleData, float3 *fleeVectors, float3 *steeringVectors, float weight, kernel_options options)
{
    int id = (blockIdx.x * blockDim.x + threadIdx.x);    
    int blockOffset = (blockDim.x * blockIdx.x * 3);
    
    // shmem for forward vector (TPB * 12 bytes)
    __shared__ float3 forward[TPB];
    
    // shmem for position (TPB * 12 bytes)
    __shared__ float3 position[TPB];
    
    // shmem for flee vectors (TPB * 12 bytes)
    __shared__ float3 flee[TPB];
    
    // load forward vector from global memory (coalesced)
    F_F(threadIdx.x) = ((float*)(*vehicleData).forward)[blockOffset + threadIdx.x];
    F_F(threadIdx.x + blockDim.x) = ((float*)(*vehicleData).forward)[blockOffset + threadIdx.x + blockDim.x]; 
    F_F(threadIdx.x + 2*blockDim.x) = ((float*)(*vehicleData).forward)[blockOffset + threadIdx.x + 2*blockDim.x];
    
    // load speed data from global memory (coalesced)
    float speed = (*vehicleData).speed[id];
    
    // load position vector from global memory (coalesced)
    P_F(threadIdx.x) = ((float*)(*vehicleData).position)[blockOffset + threadIdx.x];
    P_F(threadIdx.x + blockDim.x) = ((float*)(*vehicleData).position)[blockOffset + threadIdx.x + blockDim.x];
    P_F(threadIdx.x + 2*blockDim.x) = ((float*)(*vehicleData).position)[blockOffset + threadIdx.x + 2*blockDim.x];
    
    // load seek vector from global memory (coalesced)
    FL_F(threadIdx.x) = ((float*)fleeVectors)[blockOffset + threadIdx.x];
    FL_F(threadIdx.x + blockDim.x) = ((float*)fleeVectors)[blockOffset + threadIdx.x + blockDim.x];
    FL_F(threadIdx.x + 2*blockDim.x) = ((float*)fleeVectors)[blockOffset + threadIdx.x + 2*blockDim.x];
    
    __syncthreads();
    
    // calculate steering vectors (using seek shmem)
    FL(threadIdx.x).x = (P(threadIdx.x).x - FL(threadIdx.x).x) - (F(threadIdx.x).x * speed);
    FL(threadIdx.x).y = (P(threadIdx.x).y - FL(threadIdx.x).y) - (F(threadIdx.x).y * speed);
    FL(threadIdx.x).z = (P(threadIdx.x).z - FL(threadIdx.x).z) - (F(threadIdx.x).z * speed);
    
    __syncthreads();
    
    // multiply by weight
    FL(threadIdx.x) = float3Mul(FL(threadIdx.x), weight);
    
    if ((options & IGNORE_UNLESS_ZERO) != 0
        && (steeringVectors[id].x != 0.f
         || steeringVectors[id].y != 0.f
         || steeringVectors[id].z != 0.f))
    {
        FL(threadIdx.x) = steeringVectors[id];
    } else {
        FL(threadIdx.x) = float3Add(FL(threadIdx.x), steeringVectors[id]);
    }
    
    __syncthreads();
    
    // copy steering vectors to global memory (coalesced)
    ((float*)steeringVectors)[blockOffset + threadIdx.x] = FL_F(threadIdx.x);
    ((float*)steeringVectors)[blockOffset + threadIdx.x + blockDim.x] = FL_F(threadIdx.x + blockDim.x);
    ((float*)steeringVectors)[blockOffset + threadIdx.x + 2*blockDim.x] = FL_F(threadIdx.x + 2*blockDim.x);
}

#endif // _STEER_FOR_FLEE_KERNEL_CU_
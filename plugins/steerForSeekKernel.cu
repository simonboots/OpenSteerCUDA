#ifndef _STEER_FOR_SEEK_KERNEL_CU_
#define _STEER_FOR_SEEK_KERNEL_CU_

#include "VehicleData.h"
#include "CUDAVectorUtilities.cu"
#include "CUDAKernelOptions.cu"

#define CHECK_BANK_CONFLICTS 0
#if CHECK_BANK_CONFLICTS
#define F_F(i) (CUT_BANK_CHECKER(((float*)forward), i))
#define P_F(i) (CUT_BANK_CHECKER(((float*)position), i))
#define SK_F(i) (CUT_BANK_CHECKER(((float*)seek), i))
#define F(i) (CUT_BANK_CHECKER(forward, i))
#define P(i) (CUT_BANK_CHECKER(position, i))
#define SK(i) (CUT_BANK_CHECKER(seek, i))
#else
#define F_F(i) ((float*)forward)[i]
#define P_F(i) ((float*)position)[i]
#define SK_F(i) ((float*)seek)[i]
#define F(i) forward[i]
#define P(i) position[i]
#define SK(i) seek[i]
#endif

__global__ void
steerForSeekKernel(VehicleData *vehicleData, float3 *seekVectors, float3 *steeringVectors, float weight, kernel_options options)
{
    int id = (blockIdx.x * blockDim.x + threadIdx.x);    
    int blockOffset = (blockDim.x * blockIdx.x * 3);
    
    // shmem for forward vector (TPB * 12 bytes)
    __shared__ float3 forward[TPB];
    
    // shmem for position (TPB * 12 bytes)
    __shared__ float3 position[TPB];
    
    // shmem for seek vectors (TPB * 12 bytes)
    __shared__ float3 seek[TPB];
    
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
    SK_F(threadIdx.x) = ((float*)seekVectors)[blockOffset + threadIdx.x];
    SK_F(threadIdx.x + blockDim.x) = ((float*)seekVectors)[blockOffset + threadIdx.x + blockDim.x];
    SK_F(threadIdx.x + 2*blockDim.x) = ((float*)seekVectors)[blockOffset + threadIdx.x + 2*blockDim.x];
    
    __syncthreads();
    
    // calculate steering vectors (using seek shmem)
    SK(threadIdx.x).x = (SK(threadIdx.x).x - P(threadIdx.x).x) - (F(threadIdx.x).x * speed);
    SK(threadIdx.x).y = (SK(threadIdx.x).y - P(threadIdx.x).y) - (F(threadIdx.x).y * speed);
    SK(threadIdx.x).z = (SK(threadIdx.x).z - P(threadIdx.x).z) - (F(threadIdx.x).z * speed);

    __syncthreads();
    
    // multiply by weight
    SK(threadIdx.x) = float3Mul(SK(threadIdx.x), weight);
    
    if ((options & IGNORE_UNLESS_ZERO) != 0
        && (steeringVectors[id].x != 0.f
         || steeringVectors[id].y != 0.f
         || steeringVectors[id].z != 0.f))
    {
        SK(threadIdx.x) = steeringVectors[id];
    } else {
        SK(threadIdx.x) = float3Add(SK(threadIdx.x), steeringVectors[id]);
    }
    
    __syncthreads();
    
    // copy steering vectors to global memory (coalesced)
    ((float*)steeringVectors)[blockOffset + threadIdx.x] = SK_F(threadIdx.x);
    ((float*)steeringVectors)[blockOffset + threadIdx.x + blockDim.x] = SK_F(threadIdx.x + blockDim.x);
    ((float*)steeringVectors)[blockOffset + threadIdx.x + 2*blockDim.x] = SK_F(threadIdx.x + 2*blockDim.x);
}

#endif // _STEER_FOR_SEEK_KERNEL_CU_
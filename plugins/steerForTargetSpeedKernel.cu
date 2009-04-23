#ifndef _STEER_FOR_TARGET_SPPED_KERNEL_CU_
#define _STEER_FOR_TARGET_SPEED_KERNEL_CU_

#include <cutil.h>
#include "CUDAVectorUtilities.cu"
#include "VehicleData.h"
#include "CUDAKernelOptions.cu"

#define CHECK_BANK_CONFLICTS 1
#if CHECK_BANK_CONFLICTS
#define F_F(i) (CUT_BANK_CHECKER(((float*)forward), i))
#define F(i) (CUT_BANK_CHECKER(forward, i))
#define SP(i) (CUT_BANK_CHECKER(speed, i))
#define MF(i) (CUT_BANK_CHECKER(maxForce, i))
#else
#define F_F(i) ((float*)forward)[i]
#define F(i) forward[i]
#define SP(i) speed[i]
#define MF(i) maxForce[i]
#endif

__global__ void
steerForTargetSpeedKernel(VehicleData *vehicleData, VehicleConst *vehicleConst, float *targetSpeeds, float3 *steeringVectors)
{
    int id = (blockIdx.x * blockDim.x + threadIdx.x);
    int blockOffset = (blockDim.x * blockIdx.x * 3);
    
    // shared memory for forward vector
    __shared__ float3 forward[TPB];
    
    // shared memory for speed data
    __shared__ float speed[TPB];
    
    // shared memory for maxForce
    __shared__ float maxForce[TPB];
    
    // copy speed data from global memory (coalesced)
    SP(threadIdx.x) = (*vehicleData).speed[id];
    
    // copy forward (later velocity) data from global memory (coalesced)
    F_F(threadIdx.x) = ((float*)(*vehicleData).forward)[blockOffset + threadIdx.x];
    F_F(threadIdx.x + blockDim.x) = ((float*)(*vehicleData).forward)[blockOffset + threadIdx.x + blockDim.x]; 
    F_F(threadIdx.x + 2*blockDim.x) = ((float*)(*vehicleData).forward)[blockOffset + threadIdx.x + 2*blockDim.x];
    
    // copy maxForce data from global memory (coalesced)
    MF(threadIdx.x) = (*vehicleConst).maxForce[id];
    
    __syncthreads();
    
    float speedError = clip(targetSpeeds[id] - SP(threadIdx.x), -MF(threadIdx.x), MF(threadIdx.x));
    
    F(threadIdx.x) = float3Mul(F(threadIdx.x), speedError);
    
    __syncthreads();
    
    // writing back to global memory (coalesced)
    ((float*)steeringVectors)[blockOffset + threadIdx.x] = F_F(threadIdx.x);
    ((float*)steeringVectors)[blockOffset + threadIdx.x + blockDim.x] = F_F(threadIdx.x + blockDim.x);
    ((float*)steeringVectors)[blockOffset + threadIdx.x + 2*blockDim.x] = F_F(threadIdx.x + 2*blockDim.x);    
}

#endif // _STEER_FOR_TARGET_SPEED_KERNEL_H_
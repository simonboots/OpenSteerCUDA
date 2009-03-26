#ifndef _STEER_FOR_TARGET_SPPED_KERNEL_H_
#define _STEER_FOR_TARGET_SPEED_KERNEL_H_

#include "CUDAVectorUtilities.cu"
#include "vehicle_t.h"
#include <cutil.h>

#define CHECK_BANK_CONFLICTS 1
#if CHECK_BANK_CONFLICTS
#define V(i) (CUT_BANK_CHECKER(velocity, i))
#else
#define V(i) velocity[i]
#endif

__global__ __device__ void
steerForTargetSpeedKernel(vehicle_t *vehicleData, float *targetSpeeds, float2 *steeringVectors)
{
    int id = (blockIdx.x * blockDim.x + threadIdx.x);
    
    // shared memory for velocity vector
    __shared__ float3 velocity[TPB];
    
    // copy velocity data from global memory to shared memory
    V(threadIdx.x) = make_float3((*vehicleData).velocity[id].x, 0.f, (*vehicleData).velocity[id].y); 
    
    float speed = float3Length(V(threadIdx.x));
    float speedError = clip(targetSpeeds[id] - speed, -MAX_FORCE, MAX_FORCE);
    
    float2 forwardVector = make_float2(V(threadIdx.x).x / speed, V(threadIdx.x).z / speed);
    
    steeringVectors[id].x = (forwardVector.x * speedError);
    steeringVectors[id].y = (forwardVector.y * speedError);
}

#endif // _STEER_FOR_TARGET_SPEED_KERNEL_H_
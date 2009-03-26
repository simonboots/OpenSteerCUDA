#ifndef _STEER_FOR_WANDER_KERNEL_H_
#define _STEER_FOR_WANDER_KERNEL_H_

// Doesn't work!

#include "vehicle_t.h"
#include "CUDAFloatUtilities.cu"

__device__ float
scalarRandomWalk(float walkspeed, float min, float max);

__global__ __device__ void
steerForWanderKernel(vehicle_t *vehicleData, float dt, float2 *steeringVectors)
{
    int id = (blockIdx.x * blockDim.x + threadIdx.x);
    
    float speed = 12 * dt;
    
    float wanderSide = scalarRandomWalk(speed, -1, +1);
    steeringVectors[id].x = ((*vehicleData).side[id].x * wanderSide);
    steeringVectors[id].y = ((*vehicleData).side[id].y * wanderSide);
}

__device__ float
scalarRandomWalk(float walkspeed, float min, float max)
{
    float wander = (((frandom01() * 2) - 1) * walkspeed);
    return clip(wander, min, max);
}

#endif // _STEER_FOR_FLEE_KERNEL_H_
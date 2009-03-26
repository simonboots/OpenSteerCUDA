#ifndef _STEER_FOR_FLEE_KERNEL_H_
#define _STEER_FOR_FLEE_KERNEL_H_

#include "vehicle_t.h"

__global__ __device__ void
steerForFleeKernel(vehicle_t *vehicleData, float2 *fleeVectors, float2 *steeringVectors)
{
    int id = (blockIdx.x * blockDim.x + threadIdx.x);
    
    steeringVectors[id].x = ((*vehicleData).position[id].x - fleeVectors[id].x) - (*vehicleData).velocity[id].x;
    steeringVectors[id].y = ((*vehicleData).position[id].y - fleeVectors[id].y) - (*vehicleData).velocity[id].y;
}

#endif // _STEER_FOR_FLEE_KERNEL_H_
#ifndef _STEER_FOR_SEEK_KERNEL_H_
#define _STEER_FOR_SEEK_KERNEL_H_

#include "vehicle_t.h"

__global__ void
steerForSeekKernel(vehicle_t *vehicleData, float2 *seekVectors, float2 *steeringVectors)
{
    int id = (blockIdx.x * blockDim.x + threadIdx.x);
    
    steeringVectors[id].x = (seekVectors[id].x - (*vehicleData).position[id].x) - (*vehicleData).velocity[id].x;
    steeringVectors[id].y = (seekVectors[id].y - (*vehicleData).position[id].y) - (*vehicleData).velocity[id].y;
}

#endif // _STEER_FOR_SEEK_KERNEL_H_
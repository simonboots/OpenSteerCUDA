#ifndef _STEER_FOR_SEEK_KERNEL_H_
#define _STEER_FOR_SEEK_KERNEL_H_

#include "VehicleData.h"

__global__ void
steerForSeekKernel(VehicleData *vehicleData, float3 *seekVectors, float3 *steeringVectors)
{
    int id = (blockIdx.x * blockDim.x + threadIdx.x);
    float speed = (*vehicleData).speed[id];
    float3 velocity = make_float3((*vehicleData).forward[id].x * speed,
                                  (*vehicleData).forward[id].y * speed,
                                  (*vehicleData).forward[id].z * speed);
    
    steeringVectors[id].x = (seekVectors[id].x - (*vehicleData).position[id].x) - velocity.x;
    steeringVectors[id].y = (seekVectors[id].y - (*vehicleData).position[id].y) - velocity.y;
    steeringVectors[id].z = (seekVectors[id].z - (*vehicleData).position[id].z) - velocity.z;
}   

#endif // _STEER_FOR_SEEK_KERNEL_H_
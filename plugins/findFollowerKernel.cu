#ifndef _FIND_FOLLOWER_KERNEL_H_
#define _FIND_FOLLOWER_KERNEL_H_

#include "VehicleData.h"

__global__ void
findFollowerKernel(VehicleData *vehicleData, float3 *seekVectors)
{
    int id = (blockIdx.x * blockDim.x + threadIdx.x);
    int follower = (id + 16) % (gridDim.x * blockDim.x);
    
    seekVectors[id] = (*vehicleData).position[follower];
}   

#endif // _STEER_FOR_SEEK_KERNEL_H_
#ifndef _FIND_FOLLOWER_KERNEL_H_
#define _FIND_FOLLOWER_KERNEL_H_

#include "VehicleData.h"

__global__ void
findFollowerKernel(VehicleData *vehicleData, float3 *seekVectors)
{
    int id = (blockIdx.x * blockDim.x + threadIdx.x);
    int numOfAgents = gridDim.x * blockDim.x;
    int numOfFields = 3 * numOfAgents;
    int follower = id + 3 * 16;
    
    // coalesced memory access
    (((float*)seekVectors)[id]) = (((float*)(*vehicleData).position)[follower%numOfFields]);
    (((float*)seekVectors)[id+numOfAgents]) = (((float*)(*vehicleData).position)[(follower+numOfAgents)%numOfFields]);
    (((float*)seekVectors)[id+2*numOfAgents]) = (((float*)(*vehicleData).position)[(follower+2*numOfAgents)%numOfFields]);
}   

#endif // _FIND_FOLLOWER_KERNEL_H_
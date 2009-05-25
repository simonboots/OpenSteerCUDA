#ifndef _FIND_HAUNTED_KERNEL_CU_
#define _FIND_HAUNTED_KERNEL_CU_

#include "OpenSteer/VehicleData.h"

__global__ void
findHauntedKernel(VehicleData *vehicleData, float3 *seekVectors, unsigned int stride)
{
    int id = (blockIdx.x * blockDim.x + threadIdx.x);
    int numOfAgents = gridDim.x * blockDim.x;
    int numOfFields = 3 * numOfAgents;
    int haunted = id + 3 * stride;
    
    // coalesced memory access
    (((float*)seekVectors)[id]) = (((float*)(*vehicleData).position)[haunted%numOfFields]);
    (((float*)seekVectors)[id+numOfAgents]) = (((float*)(*vehicleData).position)[(haunted+numOfAgents)%numOfFields]);
    (((float*)seekVectors)[id+2*numOfAgents]) = (((float*)(*vehicleData).position)[(haunted+2*numOfAgents)%numOfFields]);
}   

#endif // _FIND_HAUNTED_KERNEL_CU_
#ifndef _STEER_FOR_SEPARATION_CU_
#define _STEER_FOR_SEPARATION_CU_

#include <stdio.h>
#include <cutil.h>
#include "CUDAVectorUtilities.cu"
#include "CUDANeighborUtilities.cu"
#include "OpenSteer/VehicleData.h"
#include "OpenSteer/NeighborData.h"
#include "CUDAKernelOptions.cu"

#define CHECK_BANK_CONFLICTS 0
#if CHECK_BANK_CONFLICTS
#define P_F(i) (CUT_BANK_CHECKER(((float*)position), i))
#define S_F(i) (CUT_BANK_CHECKER(((float*)steering), i))
#define P(i) (CUT_BANK_CHECKER(position, i))
#define S(i) (CUT_BANK_CHECKER(steering, i))
#else
#define P_F(i) ((float*)position)[i]
#define S_F(i) ((float*)steering)[i]
#define P(i) position[i]
#define S(i) steering[i]
#endif


__global__ void
steerForSeparationKernel(VehicleData *vehicleData, VehicleConst *vehicleConst, float3 *steeringVectors, float maxDistance, float cosMaxAngle, NeighborData* neighborData, float weight, kernel_options options)
{
    int id = (blockIdx.x * blockDim.x + threadIdx.x);
    int blockOffset = (blockDim.x * blockIdx.x * 3);
    
    // shared memory for position vector
    __shared__ float3 position[TPB];
    
    // shared memory for steering vector
    __shared__ float3 steering[TPB];
    
    S(threadIdx.x) = make_float3(0.f, 0.f, 0.f);
    
    
    // copy position vector from global memory (coalesced)
    P_F(threadIdx.x) = ((float*)(*vehicleData).position)[blockOffset + threadIdx.x];
    P_F(threadIdx.x + blockDim.x) = ((float*)(*vehicleData).position)[blockOffset + threadIdx.x + blockDim.x];
    P_F(threadIdx.x + 2*blockDim.x) = ((float*)(*vehicleData).position)[blockOffset + threadIdx.x + 2*blockDim.x];
    
    __syncthreads();
    
    
    int neighbors = 0;
    
    int i = 0;
    for (; i < neighborData[id].numOfNeighbors; i++) {
        int idOfNeighbor = neighborData[id].idsOfNeighbors[i];
        if (inNeighborhood(P(threadIdx.x), (*vehicleData).forward[id], (*vehicleData).position[idOfNeighbor], (*vehicleConst).radius[id] * 3, maxDistance, cosMaxAngle) == 1) {
            float3 offset = float3Sub((*vehicleData).position[idOfNeighbor], P(threadIdx.x));
            float distanceSquared = float3Dot(offset, offset);
            S(threadIdx.x) = float3Add(S(threadIdx.x), float3Div(offset, -distanceSquared));
            
            neighbors++;
        }
    }
    
    if (neighbors > 0) S(threadIdx.x) = float3Normalize(float3Div(S(threadIdx.x), (float)neighbors));
    
    S(threadIdx.x) = float3Mul(S(threadIdx.x), weight);

    if ((options & IGNORE_UNLESS_ZERO) != 0
        && (steeringVectors[id].x != 0.f
         || steeringVectors[id].y != 0.f
         || steeringVectors[id].z != 0.f))
    {
        S(threadIdx.x) = steeringVectors[id];
    } else {
        S(threadIdx.x) = float3Add(S(threadIdx.x), steeringVectors[id]);
    }
    
    __syncthreads();
    
    // writing back to global memory (coalesced)
    ((float*)steeringVectors)[blockOffset + threadIdx.x] = S_F(threadIdx.x);
    ((float*)steeringVectors)[blockOffset + threadIdx.x + blockDim.x] = S_F(threadIdx.x + blockDim.x);
    ((float*)steeringVectors)[blockOffset + threadIdx.x + 2*blockDim.x] = S_F(threadIdx.x + 2*blockDim.x);
}


#endif // _STEER_FOR_SEPARATION_CU_
#ifndef _STEER_TO_AVOID_CLOSE_NEIGHBORS_CU_
#define _STEER_TO_AVOID_CLOSE_NEIGHBORS_CU_

#include <cuda_runtime.h>
#include "OpenSteer/VehicleData.h"
#include "OpenSteer/NeighborData.h"
#include "CUDAKernelOptions.cu"
#include "CUDAVectorUtilities.cu"

#define CHECK_BANK_CONFLICTS 0
#if CHECK_BANK_CONFLICTS
#define N_I(i) (CUT_BANK_CHECKER(((int*)neighbor), i))
#define F_F(i) (CUT_BANK_CHECKER(((float*)forward), i))
#define P_F(i) (CUT_BANK_CHECKER(((float*)position), i))
#define S_F(i) (CUT_BANK_CHECKER(((float*)steering), i))
#define N(i) (CUT_BANK_CHECKER(neighbor, i))
#define F(i) (CUT_BANK_CHECKER(forward, i))
#define P(i) (CUT_BANK_CHECKER(position, i))
#define S(i) (CUT_BANK_CHECKER(steering, i))
#define SP(i) (CUT_BANK_CHECKER(speed, i))
#else
#define N_I(i) ((int*)neighbor)[i]
#define F_F(i) ((float*)forward)[i]
#define P_F(i) ((float*)position)[i]
#define S_F(i) ((float*)steering)[i]
#define N(i) neighbor[i]
#define F(i) forward[i]
#define P(i) position[i]
#define S(i) steering[i]
#define SP(i) speed[i]
#endif

__global__ void
steerToAvoidCloseNeighbors(VehicleData *vehicleData, VehicleConst *vehicleConst, float3 *steeringVectors, NeighborData *neighbors, float minSeparationDistance, float weight, kernel_options options)
{
    int id = (blockIdx.x * blockDim.x + threadIdx.x);
    int blockOffset = (blockDim.x * blockIdx.x * 3);
    int blockOffsetNeighbors = (blockDim.x * blockIdx.x * (MAX_NEIGHBORS + 1));
    
    // shared memory for NeighborData
    __shared__ NeighborData neighbor[TPB];
    
    // shared memory for steering vectors
    __shared__ float3 steering[TPB];
    
    S(threadIdx.x) = make_float3(0.f, 0.f, 0.f);
    
    __syncthreads();
    
    // copy neighbor data from global memory (coalesced)
    int i;    
    for (i = 0; i < (sizeof(NeighborData) / sizeof(int)); i++) {
        N_I(threadIdx.x + i*blockDim.x) = ((int*)neighbors)[blockOffsetNeighbors + threadIdx.x + i*blockDim.x];
    }
    
    __syncthreads();
    
    // for each of the other vehicles...
    for (i = 0; i < N(threadIdx.x).numOfNeighbors; i++)
    {
        int idOfNeighbor = N(threadIdx.x).idsOfNeighbors[i];
        
        float sumOfRadii = (*vehicleConst).radius[id] + (*vehicleConst).radius[idOfNeighbor];
        float minCenterToCenter = minSeparationDistance + sumOfRadii;
        float3 offset = float3Sub((*vehicleData).position[idOfNeighbor], (*vehicleData).position[id]);
        float currentDistance = float3Length(offset);
            
        if (currentDistance < minCenterToCenter)
        {
            offset = float3Mul(offset, -1.f);
            S(threadIdx.x) = float3PerpendicularComponent(offset, (*vehicleData).forward[id]);
            
            break;
        }
    }
    
    // multiply by weight
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

#endif // _STEER_TO_AVOID_CLOSE_NEIGHBORS_CU_
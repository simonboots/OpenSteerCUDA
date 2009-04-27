#ifndef _STEER_TO_AVOID_NEIGHBORS_CU_
#define _STEER_TO_AVOID_NEIGHBORS_CU_

#include <cuda_runtime.h>
#include "VehicleData.h"
#include "NeighborData.h"
#include "CUDAKernelOptions.cu"
#include "CUDAVectorUtilities.cu"
#include "CUDANeighborUtilities.cu"

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
steerToAvoidNeighbors(VehicleData *vehicleData, VehicleConst *vehicleConst, float3 *steeringVectors, NeighborData *neighbors, float minTimeToCollision, float weight, kernel_options options)
{
    int id = (blockIdx.x * blockDim.x + threadIdx.x);
    int blockOffset = (blockDim.x * blockIdx.x * 3);
    int blockOffsetNeighbors = (blockDim.x * blockIdx.x * (MAX_NEIGHBORS + 1));
    
    // shared memory for NeighborData
    __shared__ NeighborData neighbor[TPB];
    
    // shared memory for steering vectors
    __shared__ float3 steering[TPB];
    
    // copy side vector from global memory (coalesced)
    S_F(threadIdx.x) = ((float*)(*vehicleData).side)[blockOffset + threadIdx.x];
    S_F(threadIdx.x + blockDim.x) = ((float*)(*vehicleData).side)[blockOffset + threadIdx.x + blockDim.x];
    S_F(threadIdx.x + 2*blockDim.x) = ((float*)(*vehicleData).side)[blockOffset + threadIdx.x + 2*blockDim.x];
    
    __syncthreads();
    
    // copy neighbor data from global memory (coalesced)
    int i;    
    for (i = 0; i < (sizeof(NeighborData) / sizeof(int)); i++) {
        N_I(threadIdx.x + i*blockDim.x) = ((int*)neighbors)[blockOffsetNeighbors + threadIdx.x + i*blockDim.x];
    }
    
    __syncthreads();
    
    // considering potential future collisions
    float steer = 0;
    int idOfThreat = -1;
    int otherID = 0;
    
    // Time (in seconds) until the most immediate collision threat found
    // so far.  Initial value is a threshold: don't look more than this
    // many frames into the future.
    float minTime = minTimeToCollision;
    
    // avoid when future positions are this close (or less)
    float collisionDangerThreshold = (*vehicleConst).radius[id];
    
    float3 threatPositionAtNearestApproach;
    
    // for each of the other vehicles, determine which (if any)
    // pose the most immediate threat of collision.
    for (i = 0; i < N(threadIdx.x).numOfNeighbors; i++) {
        otherID = N(threadIdx.x).idsOfNeighbors[i];
        
        // predicted time until nearest approach of "this" and "other"
        float time = predictNearestApproachTime(vehicleData, id, otherID);
        
        // If the time is in the future, sooner than any other
        // threatened collision...
        if ((time >= 0) && (time < minTime)) {
            // if the two will be close enough to collide,
            // make a note of it
            if (computeNearestApproachPositions(vehicleData, id, otherID, time, &threatPositionAtNearestApproach) < collisionDangerThreshold) {
                minTime = time;
                idOfThreat = otherID;
            }
        }
    }
    
     // if a potential collision was found, compute steering to avoid
    if (idOfThreat != -1) {
        float parallelness = float3Dot((*vehicleData).forward[id], (*vehicleData).forward[idOfThreat]);
        float angle = 0.707f;
        
        if (parallelness < -angle) {
            // anti-parallel "head on" paths:
            // steer away from future threat position
            float3 offset = float3Sub(threatPositionAtNearestApproach, (*vehicleData).position[id]);
            float sideDot = float3Dot(offset, (*vehicleData).side[id]);
            steer = (sideDot > 0) ? -1.0f : 1.0f;
            
        } else {
            if (parallelness > angle) {
                float3 offset = float3Sub((*vehicleData).position[idOfThreat], (*vehicleData).position[id]);
                float sideDot = float3Dot(offset, (*vehicleData).side[id]);
                steer = (sideDot > 0) ? -1.0f : 1.0f;
            } else {
                // perpendicular paths: steer behind threat
                // (only the slower of the two does this)
                if ((*vehicleData).speed[idOfThreat] <= (*vehicleData).speed[id])
                {
                    float sideDot = float3Dot((*vehicleData).side[id], float3Mul((*vehicleData).forward[idOfThreat], (*vehicleData).speed[idOfThreat]));
                    steer = (sideDot > 0) ? -1.0f : 1.0f;
                }
            }
        }
    }
    
    // multiply by weight and steer
    S(threadIdx.x) = float3Mul(S(threadIdx.x), steer * weight);
    
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



#endif // _STEER_TO_AVOID_NEIGHBORS_CU_
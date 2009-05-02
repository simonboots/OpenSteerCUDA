#ifndef _STEER_FOR_EVASION_KERNEL_CU_
#define _STEER_FOR_EVASION_KERNEL_CU_

#include <stdio.h>
#include <cutil.h>
#include "CUDAVectorUtilities.cu"
#include "OpenSteer/VehicleData.h"
#include "CUDAKernelOptions.cu"

#define CHECK_BANK_CONFLICTS 0
#if CHECK_BANK_CONFLICTS
#define V_F(i) (CUT_BANK_CHECKER(((float*)velocity), i))
#define F_F(i) (CUT_BANK_CHECKER(((float*)forward), i))
#define P_F(i) (CUT_BANK_CHECKER(((float*)position), i))
#define S_F(i) (CUT_BANK_CHECKER(((float*)steering), i))
#define V(i) (CUT_BANK_CHECKER(velocity, i))
#define F(i) (CUT_BANK_CHECKER(forward, i))
#define P(i) (CUT_BANK_CHECKER(position, i))
#define S(i) (CUT_BANK_CHECKER(steering, i))
#define SP(i) (CUT_BANK_CHECKER(speed, i))
#else
#define V_F(i) ((float*)velocity)[i]
#define F_F(i) ((float*)forward)[i]
#define P_F(i) ((float*)position)[i]
#define S_F(i) ((float*)steering)[i]
#define V(i) velocity[i]
#define F(i) forward[i]
#define P(i) position[i]
#define S(i) steering[i]
#define SP(i) speed[i]
#endif

// prototype
__device__ void
steerForFleeKernelSingle(float3 position, float3 velocity, float3 seekVector, float3 *steeringVectors, float weight, kernel_options options);

__global__ void
steerForEvasionKernel(VehicleData *vehicleData, float3 menacePosition, float3 menaceVelocity, float3 *steeringVectors, float maxPredictionTime, float weight, kernel_options options)
{
    int id = blockIdx.x * blockDim.x + threadIdx.x;
    int blockOffset = blockDim.x * blockIdx.x * 3;
    
    // shared memory for velocity vector
    __shared__ float3 velocity[TPB];
    
    // shared memory for position vector
    __shared__ float3 position[TPB];
    
    // shared memory for speed
    __shared__ float speed[TPB];
    
    // copy forward vector from global memory (coalesced)
    V_F(threadIdx.x) = ((float*)(*vehicleData).forward)[blockOffset + threadIdx.x];
    V_F(threadIdx.x + blockDim.x) = ((float*)(*vehicleData).forward)[blockOffset + threadIdx.x + blockDim.x];
    V_F(threadIdx.x + 2*blockDim.x) = ((float*)(*vehicleData).forward)[blockOffset + threadIdx.x + 2*blockDim.x];
    
    // copy position vector from global memory (coalesced)
    P_F(threadIdx.x) = ((float*)(*vehicleData).position)[blockOffset + threadIdx.x];
    P_F(threadIdx.x + blockDim.x) = ((float*)(*vehicleData).position)[blockOffset + threadIdx.x + blockDim.x];
    P_F(threadIdx.x + 2*blockDim.x) = ((float*)(*vehicleData).position)[blockOffset + threadIdx.x + 2*blockDim.x];
    
    // copy speed from gloabl memory (coalesced)
    SP(threadIdx.x) = (*vehicleData).speed[id];
    __syncthreads();
    
    // calculate velocity vector
    V(threadIdx.x) = float3Mul(V(threadIdx.x), SP(threadIdx.x));
    __syncthreads();
    
    // offset from self to menace
    float3 offset = float3Sub(menacePosition, P(threadIdx.x));
    float distance = float3Length(offset);
        
    float roughTime = distance / float3Length(menaceVelocity);
    float predictionTime = ((roughTime > maxPredictionTime) ?
                            maxPredictionTime :
                            roughTime);
    
    float3 target = float3PredictFuturePosition(menacePosition, menaceVelocity, predictionTime);

    steerForFleeKernelSingle(P(threadIdx.x), V(threadIdx.x), target, steeringVectors, weight, options);
}

__device__ void
steerForFleeKernelSingle(float3 position, float3 velocity, float3 fleeVector, float3 *steeringVectors, float weight, kernel_options options)
{
    int id = (blockIdx.x * blockDim.x + threadIdx.x);
    int blockOffset = (blockDim.x * blockIdx.x * 3);
    
    // shared memory for steering vectors
    __shared__ float3 steering[TPB];
    
    S(threadIdx.x).x = (position.x - fleeVector.x) - velocity.x;
    S(threadIdx.x).y = (position.y - fleeVector.y) - velocity.y;
    S(threadIdx.x).z = (position.z - fleeVector.z) - velocity.z;
    
    __syncthreads();
    
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
    
    ((float*)steeringVectors)[blockOffset + threadIdx.x] = S_F(threadIdx.x);
    ((float*)steeringVectors)[blockOffset + threadIdx.x + blockDim.x] = S_F(threadIdx.x + blockDim.x);
    ((float*)steeringVectors)[blockOffset + threadIdx.x + 2*blockDim.x] = S_F(threadIdx.x + 2*blockDim.x);
} 

#endif // _STEER_FOR_EVASION_KERNEL_CU_
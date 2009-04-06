#ifndef _STEER_FOR_PURSUIT_KERNEL_H_
#define _STEER_FOR_PURSUIT_KERNEL_H_

#include <cutil.h>
#include "CUDAVectorUtilities.cu"
#include "VehicleData.h"
#include "MultiplePursuitCUDADefines.h"
#include <stdio.h>

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
#else
#define V_F(i) ((float*)velocity)[i]
#define F_F(i) ((float*)forward)[i]
#define P_F(i) ((float*)position)[i]
#define S_F(i) ((float*)steering)[i]
#define V(i) velocity[i]
#define F(i) forward[i]
#define P(i) position[i]
#define S(i) steering[i]
#endif

// prototype
__device__ void
steerForSeekKernelSingle(float3 position, float3 velocity, float3 seekVector, float3 *steeringVectors);

// constant memory (timeFactorTable)
__constant__ float timeFactorTable[9];

__global__ void
steerForPursuitKernel(VehicleData *vehicleData, float3 wandererPosition, float3 wandererVelocity, float3 *steeringVectors, float maxPredictionTime)
{
    int id = (blockIdx.x * blockDim.x + threadIdx.x);
    int blockOffset = (blockDim.x * blockIdx.x * 3);
 
    // shared memory for velocity vector
    __shared__ float3 velocity[TPB];
    
    // shared memory for velocity vector
    __shared__ float3 forward[TPB];
    
    // shared memory for position vector
    __shared__ float3 position[TPB];

    
    // copy velocity data from global memory (coalesced)
    float speed = (*vehicleData).speed[id];
    
    // copy forward vector from global memory (coalesced)
    F_F(threadIdx.x) = ((float*)(*vehicleData).forward)[blockOffset + threadIdx.x];
    F_F(threadIdx.x + blockDim.x) = ((float*)(*vehicleData).forward)[blockOffset + threadIdx.x + blockDim.x];
    F_F(threadIdx.x + 2*blockDim.x) = ((float*)(*vehicleData).forward)[blockOffset + threadIdx.x + 2*blockDim.x];

    // calculating velocity vector
    V(threadIdx.x) = float3Mul(F(threadIdx.x), speed);
    
    // copy position vector from global memory (coalesced)
    P_F(threadIdx.x) = ((float*)(*vehicleData).position)[blockOffset + threadIdx.x];
    P_F(threadIdx.x + blockDim.x) = ((float*)(*vehicleData).position)[blockOffset + threadIdx.x + blockDim.x];
    P_F(threadIdx.x + 2*blockDim.x) = ((float*)(*vehicleData).position)[blockOffset + threadIdx.x + 2*blockDim.x];
    
    __syncthreads();
    
    // offset from this to quarry, that distance, unit vector toward quarry
    float3 offset = make_float3(wandererPosition.x - P(threadIdx.x).x, 0.f, wandererPosition.z - P(threadIdx.x).z);
    float distance = float3Length(offset);
    float3 unitOffset = float3Div(offset, distance);
    
    float wandererSpeed = float3Length(wandererVelocity);
    float3 wandererForward = float3Div(wandererVelocity, wandererSpeed);

    
    // how parallel are the paths of "this" and the quarry
    // (1 means parallel, 0 is pependicular, -1 is anti-parallel)
    float parallelness = float3Dot(F(threadIdx.x), wandererForward);
    
    // how "forward" is the direction to the quarry
    // (1 means dead ahead, 0 is directly to the side, -1 is straight back)
    float forwardness = float3Dot(F(threadIdx.x), unitOffset);
    //printf("forwardness is: %f\n", forwardness);

    float directTravelTime = distance / speed;
    int f = intervalComparison (forwardness,  -0.707f, 0.707f);
    int p = intervalComparison (parallelness, -0.707f, 0.707f);
    
    float timeFactor = timeFactorTable[(f+1) + (p+1)*3]; // to be filled in below
    
    //printf("timeFactor is: %f\n", timeFactor);
    //Vec3 color;           // to be filled in below (xxx just for debugging)
    
    // estimated time until intercept of quarry
    float et = directTravelTime * timeFactor;
    
    // xxx experiment, if kept, this limit should be an argument
    float etl = (et > maxPredictionTime) ? maxPredictionTime : et;
    
    // estimated position of quarry at intercept
    //printf("wandererPos: (%f, %f, %f)\n", wandererPosition.x, wandererPosition.y, wandererPosition.z);
    //printf("wandererVel: (%f, %f, %f)\n", wandererVelocity.x, wandererVelocity.y, wandererVelocity.z);
    float3 target = float3PredictFuturePosition(wandererPosition, wandererVelocity, etl);
    //printf("target: (%f, %f, %f)\n", target.x, target.y, target.z);

    
    // annotation
//    annotationLine (position(),
//                    target,
//                    gaudyPursuitAnnotation ? color : gGray40);
    
    steerForSeekKernelSingle(P(threadIdx.x), V(threadIdx.x), target, steeringVectors);
}

__device__ void
steerForSeekKernelSingle(float3 position, float3 velocity, float3 seekVector, float3 *steeringVectors)
{
    int id = (blockIdx.x * blockDim.x + threadIdx.x);
    int blockOffset = (blockDim.x * blockIdx.x * 3);

    //printf("target: (%f, %f, %f)\n", seekVector.x, seekVector.y, seekVector.z);
    //printf("1: sv.x: %f, sv.y: %f\n", steeringVectors[id].x, steeringVectors[id].y);
    
    // shared memory for steering vectors
    __shared__ float3 steering[TPB];
    
    S(threadIdx.x).x = (seekVector.x - position.x) - velocity.x;
    S(threadIdx.x).y = (seekVector.y - position.y) - velocity.y;
    S(threadIdx.x).z = (seekVector.z - position.z) - velocity.z;
    
    ((float*)steeringVectors)[blockOffset + threadIdx.x] = S_F(threadIdx.x);
    ((float*)steeringVectors)[blockOffset + threadIdx.x + blockDim.x] = S_F(threadIdx.x + blockDim.x);
    ((float*)steeringVectors)[blockOffset + threadIdx.x + 2*blockDim.x] = S_F(threadIdx.x + 2*blockDim.x);
    
    //printf("2: sv.x: %f, sv.y: %f\n", steeringVectors[id].x, steeringVectors[id].y);
} 

#endif // _STEER_FOR_TARGET_SPEED_KERNEL_H_
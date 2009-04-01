#ifndef _STEER_FOR_PURSUIT_KERNEL_H_
#define _STEER_FOR_PURSUIT_KERNEL_H_

#include <cutil.h>
#include "CUDAVectorUtilities.cu"
#include "VehicleData.h"
#include "MultiplePursuitCUDADefines.h"
#include <stdio.h>

#define CHECK_BANK_CONFLICTS 0
#if CHECK_BANK_CONFLICTS
#define V(i) (CUT_BANK_CHECKER(velocity, i))
#define F(i) (CUT_BANK_CHECKER(forward, i))
#define P(i) (CUT_BANK_CHECKER(position, i))
#else
#define V(i) velocity[i]
#define F(i) forward[i]
#define P(i) position[i]
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
 
    // shared memory for velocity vector
    __shared__ float3 velocity[TPB];
    
    // shared memory for velocity vector
    __shared__ float3 forward[TPB];
    
    // shared memory for position vector
    __shared__ float3 position[TPB];

    
    // copy velocity data from global memory to shared memory
    float speed = (*vehicleData).speed[id];
    F(threadIdx.x) = (*vehicleData).forward[id];
    __syncthreads();
    V(threadIdx.x) = float3Mul(F(threadIdx.x), speed);
    
    //printf("V: (%f, %f, %f)\n", V(threadIdx.x).x, V(threadIdx.x).y, V(threadIdx.x).z);
    
    // copy position vector to shared memory
    P(threadIdx.x) = (*vehicleData).position[id];
    
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
    
    // Break the pursuit into nine cases, the cross product of the
    // quarry being [ahead, aside, or behind] us and heading
    // [parallel, perpendicular, or anti-parallel] to us.
//    switch (f)
//    {
//        case +1:
//            switch (p)
//        {
//            case +1:          // ahead, parallel
//                timeFactor = 4;
//                color = gBlack;
//                break;
//            case 0:           // ahead, perpendicular
//                timeFactor = 1.8f;
//                color = gGray50;
//                break;
//            case -1:          // ahead, anti-parallel
//                timeFactor = 0.85f;
//                color = gWhite;
//                break;
//        }
//            break;
//        case 0:
//            switch (p)
//        {
//            case +1:          // aside, parallel
//                timeFactor = 1;
//                color = gRed;
//                break;
//            case 0:           // aside, perpendicular
//                timeFactor = 0.8f;
//                color = gYellow;
//                break;
//            case -1:          // aside, anti-parallel
//                timeFactor = 4;
//                color = gGreen;
//                break;
//        }
//            break;
//        case -1:
//            switch (p)
//        {
//            case +1:          // behind, parallel
//                timeFactor = 0.5f;
//                color= gCyan;
//                break;
//            case 0:           // behind, perpendicular
//                timeFactor = 2;
//                color= gBlue;
//                break;
//            case -1:          // behind, anti-parallel
//                timeFactor = 2;
//                color = gMagenta;
//                break;
//        }
//            break;
//    }
    
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

    //printf("target: (%f, %f, %f)\n", seekVector.x, seekVector.y, seekVector.z);
    //printf("1: sv.x: %f, sv.y: %f\n", steeringVectors[id].x, steeringVectors[id].y);
    
    steeringVectors[id].x = (seekVector.x - position.x) - velocity.x;
    steeringVectors[id].y = (seekVector.y - position.y) - velocity.y;
    steeringVectors[id].z = (seekVector.z - position.z) - velocity.z;
    
    //printf("2: sv.x: %f, sv.y: %f\n", steeringVectors[id].x, steeringVectors[id].y);
} 

#endif // _STEER_FOR_TARGET_SPEED_KERNEL_H_
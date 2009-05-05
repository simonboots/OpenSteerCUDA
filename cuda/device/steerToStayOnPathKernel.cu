#ifndef _STEER_TO_STAY_ON_PATH_H_
#define _STEER_TO_STAY_ON_PATH_H_

#include <cutil.h>
#include "OpenSteer/VehicleData.h"
#include "OpenSteer/PathwayData.h"
#include "CUDAFloatUtilities.cu"
#include "CUDAVectorUtilities.cu"
#include "CUDAPathwayUtilities.cu"
#include "CUDAKernelOptions.cu"

#define CHECK_BANK_CONFLICTS 0
#if CHECK_BANK_CONFLICTS
#define V_F(i) (CUT_BANK_CHECKER(((float*)velocity), i))
#define P_F(i) (CUT_BANK_CHECKER(((float*)position), i))
#define S_F(i) (CUT_BANK_CHECKER(((float*)steering), i))
#define V(i) (CUT_BANK_CHECKER(velocity, i))
#define P(i) (CUT_BANK_CHECKER(position, i))
#define S(i) (CUT_BANK_CHECKER(steering, i))
#define SP(i) (CUT_BANK_CHECKER(speed, i))
#else
#define V_F(i) ((float*)velocity)[i]
#define P_F(i) ((float*)position)[i]
#define S_F(i) ((float*)steering)[i]
#define V(i) velocity[i]
#define P(i) position[i]
#define S(i) steering[i]
#define SP(i) speed[i]
#endif

// Pathway data
__constant__ PathwayData pathway;

__device__ void
steerForSeekKernelSingle(float3 position, float3 velocity, float3 seekVector, float3 *steeringVectors, int ignore, float weight, kernel_options options);

__global__ void
steerToStayOnPathKernel(VehicleData *vehicleData, float3 *steeringVectors, float predictionTime, float weight, kernel_options options)
{    
    int id = (blockIdx.x * blockDim.x + threadIdx.x);
    int blockOffset = (blockDim.x * blockIdx.x * 3);
    
    // shared memory for velocity vector
    __shared__ float3 velocity[TPB];
    
    // shared memory for position vector
    __shared__ float3 position[TPB];
    
    // shared memory for speed
    __shared__ float speed[TPB];
    
    // copy speed data from global memory (coalesced)
    SP(threadIdx.x) = (*vehicleData).speed[id];
    __syncthreads();
    
    // copy velocity data from global memory (coalesced)
    V_F(threadIdx.x) = ((float*)(*vehicleData).forward)[blockOffset + threadIdx.x];
    V_F(threadIdx.x + blockDim.x) = ((float*)(*vehicleData).forward)[blockOffset + threadIdx.x + blockDim.x];
    V_F(threadIdx.x + 2*blockDim.x) = ((float*)(*vehicleData).forward)[blockOffset + threadIdx.x + 2*blockDim.x];
    __syncthreads();
    V(threadIdx.x) = float3Mul(V(threadIdx.x), SP(threadIdx.x));
    
    // copy position data from global memory (coalesced)
    P_F(threadIdx.x) = ((float*)(*vehicleData).position)[blockOffset + threadIdx.x];
    P_F(threadIdx.x + blockDim.x) = ((float*)(*vehicleData).position)[blockOffset + threadIdx.x + blockDim.x];
    P_F(threadIdx.x + 2*blockDim.x) = ((float*)(*vehicleData).position)[blockOffset + threadIdx.x + 2*blockDim.x];
    
    __syncthreads();
    
    // predict our future position
    float3 futurePosition = float3PredictFuturePosition(P(threadIdx.x), V(threadIdx.x), predictionTime);
    
    // find the point on the path nearest the predicted future position
    float3 tangent;
    float outside;
    float3 onPath = mapPointToPath(pathway.points, pathway.numElements, pathway.radius, futurePosition, &tangent, &outside);
    
    float3 target;
    int ignore;
    
    if (outside < 0) {
        // our predicted future position was in the path,
        // return zero steering.
        target = make_float3(0, 0, 0);
        ignore = 1;
    } else {
        // our predicted future position was outside the path, need to
        // steer towards it.  Use onPath projection of futurePosition
        // as seek target
        target = onPath;
        ignore = 0;
    }
    
    steerForSeekKernelSingle(P(threadIdx.x), V(threadIdx.x), target, steeringVectors, ignore, weight, options);
}

__device__ void
steerForSeekKernelSingle(float3 position, float3 velocity, float3 seekVector, float3 *steeringVectors, int ignore, float weight, kernel_options options)
{
    int id = (blockIdx.x * blockDim.x + threadIdx.x);
    int blockOffset = (blockDim.x * blockIdx.x * 3);
    
    // shared memory for steering vectors
    __shared__ float3 steering[TPB];
    
    if (ignore != 1) {
        S(threadIdx.x).x = (seekVector.x - position.x) - velocity.x;
        S(threadIdx.x).y = (seekVector.y - position.y) - velocity.y;
        S(threadIdx.x).z = (seekVector.z - position.z) - velocity.z;        
    } else {
        S(threadIdx.x).x = seekVector.x;
        S(threadIdx.x).y = seekVector.y;
        S(threadIdx.x).z = seekVector.z;
    }
    
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
    
    // writing back to global memory (coalesced)
    ((float*)steeringVectors)[blockOffset + threadIdx.x] = S_F(threadIdx.x);
    ((float*)steeringVectors)[blockOffset + threadIdx.x + blockDim.x] = S_F(threadIdx.x + blockDim.x);
    ((float*)steeringVectors)[blockOffset + threadIdx.x + 2*blockDim.x] = S_F(threadIdx.x + 2*blockDim.x);
} 
#endif // _STEER_TO_STAY_ON_PATH_H_
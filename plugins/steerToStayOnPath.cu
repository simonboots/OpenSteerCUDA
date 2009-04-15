#ifndef _STEER_TO_STAY_ON_PATH_H_
#define _STEER_TO_STAY_ON_PATH_H_

#include <cutil.h>
#include "VehicleData.h"
#include "PathwayData.h"
#include "CUDAFloatUtilities.cu"
#include "CUDAVectorUtilities.cu"
#include "CUDAPathwayUtilities.cu"
#include "FollowPathCUDADefines.h"

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

// Pathway data
__constant__ PathwayData pathway;

__device__ void
steerForSeekKernelSingle(float3 position, float3 velocity, float3 seekVector, float3 *steeringVectors, int ignore);

__global__ void
steerToStayOnPathKernel(VehicleData *vehicleData, float3 *steeringVectors, float predictionTime)
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
    
    if (outside < 0) {
        // our predicted future position was in the path,
        // return zero steering.
        steerForSeekKernelSingle(P(threadIdx.x), V(threadIdx.x), make_float3(0, 0, 0), steeringVectors, 1);
    } else {
        // our predicted future position was outside the path, need to
        // steer towards it.  Use onPath projection of futurePosition
        // as seek target
        steerForSeekKernelSingle(P(threadIdx.x), V(threadIdx.x), onPath, steeringVectors, 0);
    }
}

__device__ void
steerForSeekKernelSingle(float3 position, float3 velocity, float3 seekVector, float3 *steeringVectors, int ignore)
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
    
    // writing back to global memory (coalesced)
    ((float*)steeringVectors)[blockOffset + threadIdx.x] = S_F(threadIdx.x);
    ((float*)steeringVectors)[blockOffset + threadIdx.x + blockDim.x] = S_F(threadIdx.x + blockDim.x);
    ((float*)steeringVectors)[blockOffset + threadIdx.x + 2*blockDim.x] = S_F(threadIdx.x + 2*blockDim.x);
} 
#endif // _STEER_TO_STAY_ON_PATH_H_
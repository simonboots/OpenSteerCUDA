#ifndef _STEER_TO_FOLLOW_PATH_H_
#define _STEER_TO_FOLLOW_PATH_H_

#include <cutil.h>
#include "VehicleData.h"
#include "PathwayData.h"
#include "CUDAFloatUtilities.cu"
#include "CUDAVectorUtilities.cu"
#include "CUDAPathwayUtilities.cu"
//#include "FollowPath CUDADefines.h"

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

// Pathway data
__constant__ PathwayData pathway;

__device__ void
steerForSeekKernelSingle(float3 position, float3 velocity, float3 seekVector, float3 *steeringVectors);

__global__ void
steerToFollowPathKernel(VehicleData* vehicleData, float3* steeringVectors, int* direction, float predictionTime)
{
    int id = (blockIdx.x * blockDim.x + threadIdx.x);
    
    // our goal will be offset from our path distance by this amount
    float pathDistanceOffset = direction[id] * predictionTime * (*vehicleData).speed[id];
    
    // predict our future position
    float3 futurePosition = float3PredictFuturePosition((*vehicleData).position[id], (*vehicleData).velocity[id], predictionTime);

    // measure distance along path of our current and predicted positions
    float nowPathDistance = mapPointToPathDistance(pathway.points, pathway.numElements, (*vehicleData).position[id]);
    float futurePathDistance = mapPointToPathDistance(pathway.points, pathway.numElements, futurePosition);
    
    // are we facing in the correction direction?
    int rightway = ((pathDistanceOffset > 0) ?
                    (nowPathDistance < futurePathDistance) :
                    (nowPathDistance > futurePathDistance));
                     

    // find the point on the path nearest the predicted future position
    // XXX need to improve calling sequence, maybe change to return a
    // XXX special path-defined object which includes two Vec3s and a 
    // XXX bool (onPath,tangent (ignored), withinPath)
    float3 tangent;
    float outside;
    float3 onPath = mapPointToPath(pathway.points, pathway.numElements, pathway.radius, futurePosition, &tangent, &outside);
    
    // no steering is required if (a) our future position is inside
    // the path tube and (b) we are facing in the correct direction
    if ((outside < 0) && rightway) {
        steeringVectors[id] = make_float3(0, 0, 0);
        return;
    } else {
        // otherwise we need to steer towards a target point obtained
        // by adding pathDistanceOffset to our current path position
        float targetPathDistance = nowPathDistance + pathDistanceOffset;
        float3 target = mapPathDistanceToPoint(pathway.points, pathway.numElements, pathway.isCyclic, targetPathDistance);
        steerForSeekKernelSingle((*vehicleData).position[id], (*vehicleData).velocity[id], target, steeringVectors);
    }
}

__device__ void
steerForSeekKernelSingle(float3 position, float3 velocity, float3 seekVector, float3 *steeringVectors)
{
    int id = (blockIdx.x * blockDim.x + threadIdx.x);
    int blockOffset = (blockDim.x * blockIdx.x * 3);
    
    // shared memory for steering vectors
    __shared__ float3 steering[TPB];
    
    S(threadIdx.x).x = (seekVector.x - position.x) - velocity.x;
    S(threadIdx.x).y = (seekVector.y - position.y) - velocity.y;
    S(threadIdx.x).z = (seekVector.z - position.z) - velocity.z;
    
    ((float*)steeringVectors)[blockOffset + threadIdx.x] = S_F(threadIdx.x);
    ((float*)steeringVectors)[blockOffset + threadIdx.x + blockDim.x] = S_F(threadIdx.x + blockDim.x);
    ((float*)steeringVectors)[blockOffset + threadIdx.x + 2*blockDim.x] = S_F(threadIdx.x + 2*blockDim.x);
} 
#endif // _STEER_TO_FOLLOW_PATH_H_
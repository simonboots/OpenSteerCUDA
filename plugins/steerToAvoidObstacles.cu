#ifndef _STEER_TO_AVOID_OBSTACLES_
#define _STEER_TO_AVOID_OBSTACLES_

#include <cutil.h>
#include "VehicleData.h"
#include "ObstacleData.h"
#include "CUDAFloatUtilities.cu"
#include "CUDAVectorUtilities.cu"
#include "WanderAroundCUDADefines.h"

#define CHECK_BANK_CONFLICTS 0
#if CHECK_BANK_CONFLICTS
#define V_F(i) (CUT_BANK_CHECKER(((float*)velocity), i))
#define F_F(i) (CUT_BANK_CHECKER(((float*)forward), i))
#define P_F(i) (CUT_BANK_CHECKER(((float*)position), i))
#define S_F(i) (CUT_BANK_CHECKER(((float*)steering), i))
#define A_F(i) (CUT_BANK_CHECKER(((float*)avoidance), i))
#define LC_F(i) (CUT_BANK_CHECKER(((float*)localcenter), i))
#define V(i) (CUT_BANK_CHECKER(velocity, i))
#define F(i) (CUT_BANK_CHECKER(forward, i))
#define P(i) (CUT_BANK_CHECKER(position, i))
#define S(i) (CUT_BANK_CHECKER(steering, i))
#define SP(i) (CUT_BANK_CHECKER(speed, i))
#define A(i) (CUT_BANK_CHECKER(avoidance, i))
#define LC(i) (CUT_BANK_CHECKER(localcenter, i))
#else
#define V_F(i) ((float*)velocity)[i]
#define F_F(i) ((float*)forward)[i]
#define P_F(i) ((float*)position)[i]
#define S_F(i) ((float*)steering)[i]
#define A_F(i) ((float*)avoidance)[i]
#define LC_F(i) ((float*)localcenter)[i]
#define V(i) velocity[i]
#define F(i) forward[i]
#define P(i) position[i]
#define S(i) steering[i]
#define A(i) avoidance[i]
#define SP(i) speed[i]
#define LC(i) localcenter[i]
#endif


// Obstacle Data
__constant__ ObstacleData obstacles[NUM_OF_OBSTACLES];
__constant__ int numOfObstacles;

__global__ void
steerToAvoidObstacles(VehicleData* vehicleData, float3 *steeringVectors)
{
    int id = (blockIdx.x * blockDim.x + threadIdx.x);
    int blockOffset = (blockDim.x * blockIdx.x * 3);
    
    // shared memory for avoidance vector
    __shared__ float3 avoidance[TPB];
    
    // shared memory for local center
    //__shared__ float3 localcenter[TPB];
    
    A(threadIdx.x) = make_float3(0, 0, 0);
    
    int intersectionFound = 0;
    int nearestIntersectionID = 0;
    float nearestIntersectionDistance = MAXFLOAT;
    
    float minDistanceToCollision = (*vehicleData).speed[id] * 10.f;
    
    int i = 0;
    
    // Find nearest obstacle
    for (; i < numOfObstacles; i++) {
        // find next intersection with sphere
        float b, c, d, p, q, s;
        float intersectionDistance = 0.f;
        
        float3 lc;
        
        // find local center
        // -----------------
        
//        LC(threadIdx.x).x = obstacles[i].center.x - (*vehicleData).position[id].x;
//        LC(threadIdx.x).y = obstacles[i].center.y - (*vehicleData).position[id].y;
//        LC(threadIdx.x).z = obstacles[i].center.z - (*vehicleData).position[id].z;

        lc.x = obstacles[i].center.x - (*vehicleData).position[id].x;
        lc.y = obstacles[i].center.y - (*vehicleData).position[id].y;
        lc.z = obstacles[i].center.z - (*vehicleData).position[id].z;
        
        
        lc = make_float3(float3Dot(lc, (*vehicleData).side[id]),
                                      float3Dot(lc, (*vehicleData).up[id]),
                                      float3Dot(lc, (*vehicleData).forward[id]));
        
        // compute line-sphere intersection parameters
        b = -2*lc.z;
        c = lc.x*lc.x + lc.y*lc.y + lc.z*lc.z - (obstacles[i].radius + (*vehicleData).radius[id])*(obstacles[i].radius + (*vehicleData).radius[id]);
        d = (b * b) - (4 * c);
        
        // path does not intersect sphere
        if (d < 0) continue;
        
        s = sqrt(d);
        p = (-b + s) / 2;
        q = (-b - s) / 2;
        
        // both intersection behind us - no potential collisions
        if ((p < 0) && (q < 0)) continue;
        
        intersectionFound = 1;
        intersectionDistance = 
            ((p > 0) && (q < 0)) ?
            // both intersections in front of us, find nearest one
            ((p < q) ? p : q) :
            // otherwise only one intersection in front of us, select it
            ((p > 0) ? p : q);
        
        if (intersectionDistance < nearestIntersectionDistance) {
            nearestIntersectionDistance = intersectionDistance;
            nearestIntersectionID = i;
        }
    }
    
    if (intersectionFound == 1 && nearestIntersectionDistance < minDistanceToCollision) {
        float3 offset = float3Sub((*vehicleData).position[id], obstacles[nearestIntersectionID].center);
        A(threadIdx.x) = float3PerpendicularComponent(offset, (*vehicleData).forward[id]);
        A(threadIdx.x) = float3Normalize(A(threadIdx.x));
        A(threadIdx.x) = float3Mul(A(threadIdx.x), (*vehicleData).maxForce[id]);
        A(threadIdx.x) = float3Add(A(threadIdx.x), float3Mul((*vehicleData).forward[id], (*vehicleData).maxForce[id] * 0.75));
    }
    
    __syncthreads();
    
    // writing back to global memory (coalesced)
    ((float*)steeringVectors)[blockOffset + threadIdx.x] = A_F(threadIdx.x);
    ((float*)steeringVectors)[blockOffset + threadIdx.x + blockDim.x] = A_F(threadIdx.x + blockDim.x);
    ((float*)steeringVectors)[blockOffset + threadIdx.x + 2*blockDim.x] = A_F(threadIdx.x + 2*blockDim.x);
}

#endif // _STEER_TO_AVOID_OBSTACLES_
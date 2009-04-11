#ifndef _STEER_TO_AVOID_OBSTACLES_
#define _STEER_TO_AVOID_OBSTACLES_

#include <cutil.h>
#include "VehicleData.h"
#include "ObstacleData.h"
#include "CUDAFloatUtilities.cu"
#include "CUDAVectorUtilities.cu"
#include "WanderAroundCUDADefines.h"

// Obstacle Data
__constant__ ObstacleData obstacles[NUM_OF_OBSTACLES];

__global__ void
steerToAvoidObstacles(VehicleData* vehicleData, float3 *steeringVectors)
{
    int id = (blockIdx.x * blockDim.x + threadIdx.x);
    
    int intersectionFound = 0;
    int nearestIntersectionID = 0;
    float nearestIntersectionDistance = MAXFLOAT;
    
    float minDistanceToCollision = (*vehicleData).speed[id] * 10.f;
    
    float3 avoidance = make_float3(0, 0, 0);
    
    int i = 0;
    
    // Find nearest obstacle
    for (; i < NUM_OF_OBSTACLES; i++) {
        // find next intersection with sphere
        float b, c, d, p, q, s;
        float intersectionDistance = 0.f;
        float3 lc;
        
        // find local center
        // -----------------
        
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
        avoidance = float3PerpendicularComponent(offset, (*vehicleData).forward[id]);
        avoidance = float3Normalize(avoidance);
        avoidance = float3Mul(avoidance, (*vehicleData).maxForce[id]);
        avoidance = float3Add(avoidance, float3Mul((*vehicleData).forward[id], (*vehicleData).maxForce[id] * 0.75));
    }
    
    steeringVectors[id] = avoidance;
}

#endif // _STEER_TO_AVOID_OBSTACLES_
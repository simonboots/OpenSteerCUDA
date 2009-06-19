#ifndef _CUDA_NEIGHBOR_UTILITIES_CU_
#define _CUDA_NEIGHBOR_UTILITIES_CU_

#include "OpenSteer/NeighborData.h"
#include "OpenSteer/VehicleData.h"
#include "CUDAVectorUtilities.cu"
#include <stdio.h>

#define WORLD_SIZE 100
#define CELL_SIZE 5

__device__ int
index(float _x, float _y, float _z)
{
    int cellsPerDimension = 2*WORLD_SIZE/CELL_SIZE;
    
    int x = (_x + WORLD_SIZE) / CELL_SIZE;
    int y = (_y + WORLD_SIZE) / CELL_SIZE;
    int z = (_z + WORLD_SIZE) / CELL_SIZE;
    
    return x +
    y * cellsPerDimension +
    z * cellsPerDimension * cellsPerDimension;
}

__device__ int
indexByCellIndex(int3 cell)
{
    int cellsPerDimension = 2*WORLD_SIZE/CELL_SIZE;
    
    return cell.x +
    cell.y * cellsPerDimension +
    cell.z * cellsPerDimension * cellsPerDimension;
}

__device__ int3
cellIndex(float3 position)
{
    int3 index;
    index.x = (position.x + WORLD_SIZE) / CELL_SIZE;
    index.y = (position.y + WORLD_SIZE) / CELL_SIZE;
    index.z = (position.z + WORLD_SIZE) / CELL_SIZE;
    
    return index;
}

__device__ int
isValidCell(int3 cell)
{
    int cellsPerDimension = 2*WORLD_SIZE/CELL_SIZE;
    if (cell.x >= cellsPerDimension ||
        cell.y >= cellsPerDimension ||
        cell.z >= cellsPerDimension) {
        return 0;
    }
    return 1;
}

__device__ void
addNeighbor(NeighborData *neighbor, float radius, VehicleData *vehicleData, int vehicleIndex, int myID)
{
    float distance = float3Distance((*vehicleData).position[myID], (*vehicleData).position[vehicleIndex]);
    
    if (distance > radius) return;
    
    if (neighbor[threadIdx.x].numOfNeighbors < MAX_NEIGHBORS) {
        // just add to neighbors
        neighbor[threadIdx.x].idsOfNeighbors[neighbor[threadIdx.x].numOfNeighbors++] = vehicleIndex;
    } else {
        // replace neighbor with longest distance
        float maxDistance = 0.f;
        int neighborToReplace = -1;
        int i;
        for (i = 0; i < neighbor[threadIdx.x].numOfNeighbors; i++) {
            float testDistance = float3Distance((*vehicleData).position[myID], (*vehicleData).position[neighbor[threadIdx.x].idsOfNeighbors[i]]);
            if (testDistance > maxDistance) {
                maxDistance = testDistance;
                neighborToReplace = i;
            }
        }
        
        if (distance < maxDistance) {
            neighbor[threadIdx.x].idsOfNeighbors[neighborToReplace] = vehicleIndex;
        }
    }
}

__device__ int
inNeighborhood(float3 myPosition, float3 myForward, float3 otherPosition, float minDistance, float maxDistance, float cosMaxAngle)
{
    float3 offset = float3Sub(otherPosition, myPosition);
    float distance = float3Length(offset);
    
    // definitely in neighborhood if inside minDistance sphere
    if (distance < minDistance)
    {
        return 1; // true
    }
    else
    {
        // definitely not in neighborhood if outside maxDistance sphere
        if (distance > maxDistance)
        {
            return 0; // false
        }
        else
        {
            if (distance != 0.f)
            {
                // otherwise, test angular offset from forward axis
                float3 unitOffset = float3Div(offset, distance);
                float forwardness = float3Dot(myForward, unitOffset);
                return forwardness > cosMaxAngle;                
            } else {
                return 0;
            }
        }
    }
}

__device__ float3
sphericalWrapAround (float3 myPosition, float3 center)
{
    // [3 FLOPS]
    float3 offset = float3Sub(myPosition, center);
    // [6 FLOPS]
    float r = float3Length(offset);
    // [1 FLOPS]
    if (r > WORLD_SIZE)
        // [3 4 3 FLOPS]
        return float3Add(myPosition, float3Mul(float3Div(offset, r), WORLD_SIZE * -2));
        //return *this + ((offset/r) * radius * -2);
    else
        return myPosition;
}

__device__ float
predictNearestApproachTime(VehicleData *vehicleData, int myID, int neighborID)
{
    // imagine we are at the origin with no velocity,
    // compute the relative velocity of the other vehicle
    float3 myVelocity = float3Mul((*vehicleData).forward[myID], (*vehicleData).speed[myID]);
    float3 otherVelocity = float3Mul((*vehicleData).forward[neighborID], (*vehicleData).speed[neighborID]);
    float3 relVelocity = float3Sub(otherVelocity, myVelocity);
    float relSpeed = float3Length(relVelocity);
    
    // for parallel paths, the vehicles will always be at the same distance,
    // so return 0 (aka "now") since "there is no time like the present"
    if (relSpeed == 0) return 0;
    
    // Now consider the path of the other vehicle in this relative
    // space, a line defined by the relative position and velocity.
    // The distance from the origin (our vehicle) to that line is
    // the nearest approach.
    
    // Take the unit tangent along the other vehicle's path
    float3 relTangent = float3Div(relVelocity, relSpeed);
    
    // find distance from its path to origin (compute offset from
    // other to us, find length of projection onto path)
    float3 relPosition = float3Sub((*vehicleData).position[myID], (*vehicleData).position[neighborID]);
    float projection = float3Dot(relTangent, relPosition);
    
    return projection / relSpeed;
}

__device__ float
computeNearestApproachPositions(VehicleData *vehicleData, int myID, int neighborID, float time, float3 *threatPositionAtNearestApproach)
{
    float3 myTravel = float3Mul((*vehicleData).forward[myID], (*vehicleData).speed[myID] * time);
    float3 otherTravel = float3Mul((*vehicleData).forward[neighborID], (*vehicleData).speed[neighborID] * time);
    
    float3 myFinal = float3Add((*vehicleData).position[myID], myTravel);
    float3 otherFinal = float3Add((*vehicleData).position[neighborID], otherTravel);
    
    return float3Distance(myFinal, otherFinal);
}

#endif // _CUDA_NEIGHBOR_UTILITIES_CU_
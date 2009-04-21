#ifndef _CUDA_NEIGHBOR_UTILITIES_CU_
#define _CUDA_NEIGHBOR_UTILITIES_CU_

#include "NeighborData.h"
#include "VehicleData.h"
#include "CUDAVectorUtilities.cu"

#define WORLD_SIZE 100
#define CELL_SIZE 5

__device__ int
index(float _x, float _y, float _z)
{
    int cellsPerDimension = WORLD_SIZE/CELL_SIZE;
    
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
    int cellsPerDimension = WORLD_SIZE/CELL_SIZE;
    
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
    int cellsPerDimension = WORLD_SIZE/CELL_SIZE;
    if (cell.x >= cellsPerDimension ||
        cell.y >= cellsPerDimension ||
        cell.z >= cellsPerDimension) {
        return 0;
    }
    return 1;
}

__device__ void
addNeighbor(NeighborData neighbor, float radius, VehicleData *vehicleData, int vehicleIndex, int myID)
{
    float distance = float3Distance((*vehicleData).position[myID], (*vehicleData).position[vehicleIndex]);

    if (distance > radius) return;
    
    if (neighbor.numOfNeighbors <= MAX_NEIGHBORS) {
        // just add to neighbors
        neighbor.idsOfNeighbors[neighbor.numOfNeighbors++] = vehicleIndex;
    } else {
        // replace neighbor with longest distance
        float maxDistance = 0.f;
        int neighborToReplace = -1;
        int i;
        for (i = 0; i < neighbor.numOfNeighbors; i++) {
            float testDistance = float3Distance((*vehicleData).position[myID], (*vehicleData).position[neighbor.idsOfNeighbors[i]]);
            if (testDistance > maxDistance) {
                maxDistance = testDistance;
                neighborToReplace = i;
            }
        }
        
        if (distance < maxDistance) {
            neighbor.idsOfNeighbors[neighborToReplace] = vehicleIndex;
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
            // otherwise, test angular offset from forward axis
            float3 unitOffset = float3Div(offset, distance);
            float forwardness = float3Dot(myForward, unitOffset);
            return forwardness > cosMaxAngle;
        }
    }
}

__device__ float3
sphericalWrapAround (float3 myPosition, float3 center)
{
    float3 offset = float3Sub(myPosition, center);
    float r = float3Length(offset);
    if (r > WORLD_SIZE/2)
        return float3Add(myPosition, float3Mul(float3Div(offset, r), WORLD_SIZE * -1));      \
        //return *this + ((offset/r) * radius * -2);
    else
        return myPosition;
}

#endif // _CUDA_NEIGHBOR_UTILITIES_CU_
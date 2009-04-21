#ifndef _FIND_NEIGHBORS_KERNEL_CU_
#define _FIND_NEIGHBORS_KERNEL_CU_

#include "VehicleData.h"
#include "NeighborData.h"
#include "CUDANeighborUtilities.cu"
#include "CUDAKernelOptions.cu"

__global__ void
findNeighborsKernel(VehicleData* vehicleData, int* indices, int* agents, NeighborData* neighbours, float radius)
{
    int id = (blockIdx.x * blockDim.x + threadIdx.x);
    int numOfAgents = (gridDim.x * blockDim.x);
    
    // reset numbers of neighbors
    neighbours[id].numOfNeighbors = 0;
    
    int3 my_grid_cell = cellIndex((*vehicleData).position[id]);
    int cpd = floor(radius / CELL_SIZE + 1);
    
    for (int x = -cpd; x<=cpd; ++x) {
        for (int y = -cpd; y<=cpd; ++y) {
            for (int z = -cpd; z<=cpd; ++z) {
                
                int3 offset = make_int3(x, y, z);
                int3 cell = make_int3(my_grid_cell.x + offset.x,
                                      my_grid_cell.y + offset.y,
                                      my_grid_cell.z + offset.z);
                
                if (isValidCell(cell) == 0) {
                    continue;
                }
                
                // TODO: Check if cell is within search radius
                
                int cell_index = indexByCellIndex(cell);
                int startID = indices[cell_index];
                int totalCells = (WORLD_SIZE / CELL_SIZE) * (WORLD_SIZE / CELL_SIZE) * (WORLD_SIZE / CELL_SIZE);
                int endID = (cell_index + 1) == totalCells ? numOfAgents-1 : indices[cell_index+1];

                int i = startID;
                for (; i < endID; i++) {
                    if (agents[i] != id)
                        addNeighbor(neighbours[id], radius, vehicleData, agents[i], id);
                }
            }
        }
    }
}

#endif // _FIND_NEIGHBORS_KERNEL_CU_
#ifndef _FIND_NEIGHBORS_KERNEL_CU_
#define _FIND_NEIGHBORS_KERNEL_CU_

#include "OpenSteer/VehicleData.h"
#include "OpenSteer/NeighborData.h"
#include "CUDANeighborUtilities.cu"
#include "CUDAKernelOptions.cu"
#include <stdio.h>

#define CHECK_BANK_CONFLICTS 0
#if CHECK_BANK_CONFLICTS
#define N_I(i) (CUT_BANK_CHECKER(((int*)neighbor), i))
#define N(i) (CUT_BANK_CHECKER(neighbor, i))
#else
#define N_I(i) ((int*)neighbor)[i]
#define N(i) neighbor[i]
#endif

__global__ void
findNeighborsKernel(VehicleData* vehicleData, int* indices, int* agents, NeighborData* neighbors, float radius)
{
    int id = (blockIdx.x * blockDim.x + threadIdx.x);
    int blockOffsetNeighbors = (blockDim.x * blockIdx.x * (MAX_NEIGHBORS + 1));
    int numOfAgents = (gridDim.x * blockDim.x);
    
    // shared memort for neighbor data
    __shared__ NeighborData neighbor[TPB];
    
    //printf("handling id %d\n", id);
    
    // reset numbers of neighbors
    N(threadIdx.x).numOfNeighbors = 0;
    
    int3 my_grid_cell = cellIndex((*vehicleData).position[id]);
    int cpd = floor(radius / CELL_SIZE + 1);
    
    //printf("cpd is %d\n", cpd);
    
    for (int x = -cpd; x<=cpd; ++x) {
        for (int y = -cpd; y<=cpd; ++y) {
            for (int z = -cpd; z<=cpd; ++z) {
                
                int3 offset = make_int3(x, y, z);
                int3 cell = make_int3(my_grid_cell.x + offset.x,
                                      my_grid_cell.y + offset.y,
                                      my_grid_cell.z + offset.z);
                
                //printf("x: %d, y:%d, z:%d\n", x, y, z);
             
                if (isValidCell(cell) == 0) {
                    continue;
                }
                
                // TODO: Check if cell is within search radius
                
                int cell_index = indexByCellIndex(cell);
                //printf("cell_index is %d\n", cell_index);
                int startID = indices[cell_index];
                int totalCells = (2 * WORLD_SIZE / CELL_SIZE) * (2 * WORLD_SIZE / CELL_SIZE) * (2 * WORLD_SIZE / CELL_SIZE);
                int endID = (cell_index + 1) == totalCells ? numOfAgents-1 : indices[cell_index+1];
                
                //printf("startID: %d, endID: %d, totalCells: %d\n", startID, endID, totalCells);
                
                int i = startID;
                for (; i < endID; i++) {
                    if (agents[i] != id)
                        addNeighbor(neighbor, radius, vehicleData, agents[i], id);
                }
            }
        }
    }

    // copy neighbor data back to global memory
    __syncthreads();

    int i;    
    for (i = 0; i < (sizeof(NeighborData) / sizeof(int)); i++) {
        ((int*)neighbors)[blockOffsetNeighbors + threadIdx.x + i*blockDim.x] = N_I(threadIdx.x + i*blockDim.x);
    }
    
    __syncthreads();
}

#endif // _FIND_NEIGHBORS_KERNEL_CU_
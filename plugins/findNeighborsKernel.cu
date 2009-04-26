#ifndef _FIND_NEIGHBORS_KERNEL_CU_
#define _FIND_NEIGHBORS_KERNEL_CU_

#include "VehicleData.h"
#include "NeighborData.h"
#include "CUDANeighborUtilities.cu"
#include "CUDAKernelOptions.cu"
#include <stdio.h>

#define CHECK_BANK_CONFLICTS 0
#if CHECK_BANK_CONFLICTS
#define N_I(i) (CUT_BANK_CHECKER(((int*)neighbor), i))
#define F_F(i) (CUT_BANK_CHECKER(((float*)forward), i))
#define P_F(i) (CUT_BANK_CHECKER(((float*)position), i))
#define S_F(i) (CUT_BANK_CHECKER(((float*)steering), i))
#define N(i) (CUT_BANK_CHECKER(neighbor, i))
#define F(i) (CUT_BANK_CHECKER(forward, i))
#define P(i) (CUT_BANK_CHECKER(position, i))
#define S(i) (CUT_BANK_CHECKER(steering, i))
#define SP(i) (CUT_BANK_CHECKER(speed, i))
#else
#define N_I(i) ((int*)neighbor)[i]
#define F_F(i) ((float*)forward)[i]
#define P_F(i) ((float*)position)[i]
#define S_F(i) ((float*)steering)[i]
#define N(i) neighbor[i]
#define F(i) forward[i]
#define P(i) position[i]
#define S(i) steering[i]
#define SP(i) speed[i]
#endif

__global__ void
findNeighborsKernel(VehicleData* vehicleData, int* indices, int* agents, NeighborData* neighbors, float radius)
{
    int id = (blockIdx.x * blockDim.x + threadIdx.x);
    int blockOffsetNeighbors = (blockDim.x * blockIdx.x * (MAX_NEIGHBORS + 1));
    int numOfAgents = (gridDim.x * blockDim.x);
    
    // shared memort for neighbor data
    __shared__ NeighborData neighbor[TPB];
    
    // shared memory for position data
    __shared__ float3 positions[TPB][MAX_NEIGHBORS];
    
    //printf("handling id %d\n", id);
    
    // reset numbers of neighbors
    neighbor[threadIdx.x].numOfNeighbors = 0;
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
                        addNeighbor(neighbor, radius, vehicleData, agents[i], id, (float3**)positions);
                }
            }
        }
    }
    
    __syncthreads();
    int i;    
    // copy neighbor data back to global memory
    for (i = 0; i < (sizeof(NeighborData) / sizeof(int)); i++) {
        ((int*)neighbors)[blockOffsetNeighbors + threadIdx.x + i*blockDim.x] = N_I(threadIdx.x + i*blockDim.x);
    }
    //neighbors[id] = neighbor[threadIdx.x];
    
    __syncthreads();
//    if (neighbors[id].numOfNeighbors > 0) 
//        printf("content is: NUM: %d VALUES (%d, %d, %d, %d, %d, %d, %d)\n", neighbors[id].numOfNeighbors, neighbors[id].idsOfNeighbors[0], neighbors[id].idsOfNeighbors[1], neighbors[id].idsOfNeighbors[2], neighbors[id].idsOfNeighbors[3], neighbors[id].idsOfNeighbors[4], neighbors[id].idsOfNeighbors[5], neighbors[id].idsOfNeighbors[6]);
    
}

#endif // _FIND_NEIGHBORS_KERNEL_CU_
#include "FindNeighbors.h"
#include <cuda_runtime.h>
#include "OpenSteer/VehicleData.h"
#include "OpenSteer/NeighborData.h"
#include "OpenSteer/MemoryBackend.h"
#include "CUDAKernelOptions.cu"
#include <iostream>

using namespace OpenSteer;
using namespace std;

__global__ void
findNeighborsKernel(VehicleData* vehicleData, int* indices, int* vehicles, NeighborData* neighbors, float radius);

OpenSteer::FindNeighbors::FindNeighbors(float radius)
{
    grid = NULL;
    d_neighborData = NULL;
    d_indices = NULL;
    d_vehicles = NULL;
    threadsPerBlock = 128;
    this->radius = radius;
}

OpenSteer::FindNeighbors::~FindNeighbors() {}

void OpenSteer::FindNeighbors::init()
{
    grid = new Grid();
    
    // device memory for neighbor data
    mem_size_neighbor_data = getNumberOfAgents()*sizeof(NeighborData);
    cudaError_t retval = cudaMalloc((void **)&d_neighborData, mem_size_neighbor_data);
    if (retval != cudaSuccess)
        cout << "Error while allocating d_neighborData memory: " << cudaGetErrorString(retval) << endl;
    
    // device memory for neighbor indices
    mem_size_neighbor_indices = grid->numOfCells()*sizeof(int);
    retval = cudaMalloc((void **)&d_indices, mem_size_neighbor_indices);
    if (retval != cudaSuccess)
        cout << "Error while allocating d_indices memory: " << cudaGetErrorString(retval) << endl;
    
    // device memory for neighbor agents
    mem_size_neighbor_vehicles = getNumberOfAgents()*sizeof(int);
    retval = cudaMalloc((void **)&d_vehicles, mem_size_neighbor_vehicles);
    if (retval != cudaSuccess)
        cout << "Error while allocating d_agents memory: " << cudaGetErrorString(retval) << endl;
}

void OpenSteer::FindNeighbors::run()
{    
    MemoryBackend *mb = getMemoryBackend();
    
    for (int i = 0; i < getNumberOfAgents(); i++) {
        grid->save(mb->position(i).x, mb->position(i).y, mb->position(i).z, i);
    }
    
    cudaMemcpy(d_indices, grid->getIndices(), grid->numOfCells()*sizeof(int), cudaMemcpyHostToDevice);
    cudaMemcpy(d_vehicles, grid->getAgents(), grid->numOfAgents()*sizeof(int), cudaMemcpyHostToDevice);
    
    findNeighborsKernel<<<gridDim(), blockDim()>>>(getVehicleData(), d_indices, d_vehicles, d_neighborData, radius);
    
    grid->clear();
}

void OpenSteer::FindNeighbors::close()
{
    delete grid;
    
    if (d_neighborData != NULL) {
        cudaFree(d_neighborData);
        d_neighborData = NULL;        
    }
    
    if (d_indices != NULL) {
        cudaFree(d_indices);
        d_indices = NULL;
    }
    
    if (d_vehicles != NULL) {
        cudaFree(d_vehicles);
        d_vehicles = NULL;
    }
}

#include <cuda_runtime.h>
#include <cutil.h>
#include <iostream>
#include "OpenSteer/VehicleData.h"
#include "OpenSteer/ObstacleData.h"
#include "OpenSteer/NeighborData.h"
#include "OpenSteer/RandomizedVector.h"
#include "CUDAKernelOptions.cu"


__global__ void
findNeighborsKernel(VehicleData* vehicleData, int* indices, int* agents, NeighborData* neighbours, float radius);

__global__ void
steerToAvoidCloseNeighbors(VehicleData *vehicleData, VehicleConst *vehicleConst, float3 *steeringVectors, NeighborData *neighbors, float minSeparationDistance, float weight, kernel_options options);

__global__ void
steerToAvoidNeighbors(VehicleData *vehicleData, VehicleConst *vehicleConst, float3 *steeringVectors, NeighborData *neighbors, float minTimeToCollision, float weight, kernel_options options);

__global__ void
steerToAvoidObstacles(VehicleData* vehicleData, VehicleConst* vehicleConst, float3 *steeringVectors, float weight, kernel_options options);

__global__ __device__ void
steerForWander2DKernel(VehicleData *vehicleData, float *random, float dt, float3 *steeringVectors, float2 *wanderData, float weight, kernel_options options);

__global__ void
updateKernel(VehicleData *vehicleData, VehicleConst *vehicleConst, float3 *steeringVectors, float elapsedTime, kernel_options options);

using namespace OpenSteer;
using namespace std;


// device memory objects
static VehicleData* d_vehicleData = NULL;
static VehicleConst* d_vehicleConst = NULL;
static float3* d_steeringVectors = NULL;
static float* d_randomNumbers = NULL;
static float2* d_wanderData = NULL;
static NeighborData* d_neighborData = NULL;
static int* d_neighborIndices = NULL;
static int* d_neighborAgents = NULL;
static int first_run = 1;

// host memory objects
static RandomizedVector* rVec = NULL;

void runWanderAroundKernel(VehicleData *h_vehicleData, VehicleConst *h_vehicleConst, int numOfVehicles, ObstacleData *h_obstacleData, int numOfObstacles, int* h_neighborIndices, int numOfNIndices, int* h_neighborAgents, int numOfNAgents, float elapsedTime)
{
    // init GPU
    int gpu_count;
    cudaGetDeviceCount(&gpu_count);
    if (gpu_count < 1) {
        return;
    }
    
    cudaSetDevice(0);
    
    dim3 grid(numOfVehicles/TPB,1,1);
    dim3 threads(TPB,1,1);
    
    // allocate device memory
    const unsigned int mem_size_vehicle = sizeof(VehicleData);
    if (d_vehicleData == NULL) {
        cudaMalloc((void **)&d_vehicleData, mem_size_vehicle);
        cudaMemcpy(d_vehicleData, h_vehicleData, mem_size_vehicle, cudaMemcpyHostToDevice);
    }
    
    const unsigned int mem_size_vehicle_const = sizeof(VehicleConst);
    if (d_vehicleConst == NULL) {
        cudaMalloc((void **)&d_vehicleConst, mem_size_vehicle_const);
        cudaMemcpy(d_vehicleConst, h_vehicleConst, mem_size_vehicle_const, cudaMemcpyHostToDevice);
    }
    
    const unsigned int mem_size_steering = sizeof(float3) * numOfVehicles;
    if (d_steeringVectors == NULL) {
        cudaMalloc((void **)&d_steeringVectors, mem_size_steering);
    }
    
    cudaMemset(d_steeringVectors, 0, mem_size_steering);
    
    if (first_run == 1) {
        rVec = new RandomizedVector(2*numOfVehicles);
    }
    
    const unsigned int mem_size_random = rVec->size() * sizeof(float);
    if (d_randomNumbers == NULL) {
        cudaMalloc((void **)&d_randomNumbers, mem_size_random);
    }
    
    const unsigned int mem_size_wander = numOfVehicles * sizeof(float2);
    if (d_wanderData == NULL) {
        cudaMalloc((void **)&d_wanderData, mem_size_wander);
        cudaMemset(d_wanderData, 0, mem_size_wander);
    }
    
    // first run initializations
    if (first_run == 1) {
        cudaMemcpyToSymbol("d_obstacles", h_obstacleData, sizeof(ObstacleData) * numOfObstacles, 0, cudaMemcpyHostToDevice);
        cudaMemcpyToSymbol("d_numOfObstacles", &numOfObstacles, sizeof(int), 0, cudaMemcpyHostToDevice);
    }
    
    // renew random numbers
    rVec->renew();
    cudaMemcpy(d_randomNumbers, rVec->getVector(), mem_size_random, cudaMemcpyHostToDevice);
    
    // neighbors
    const unsigned int mem_size_neighbor_indices = sizeof(int) * numOfNIndices;
    const unsigned int mem_size_neighbor_agents = sizeof(int) * numOfNAgents;
    const unsigned int mem_size_neighbor_data = sizeof(NeighborData) * numOfVehicles;
    
    if (d_neighborData == NULL) {
        const unsigned int mem_size_neighbor_data = sizeof(NeighborData) * numOfVehicles;
        cudaMalloc((void **)&d_neighborData, mem_size_neighbor_data);
        cudaMalloc((void **)&d_neighborIndices, mem_size_neighbor_indices);
        cudaMalloc((void **)&d_neighborAgents, mem_size_neighbor_agents);
    }
    
    // copy neighbor raw data to global memory
    cudaMemcpy(d_neighborIndices, h_neighborIndices, mem_size_neighbor_indices, cudaMemcpyHostToDevice);
    cudaMemcpy(d_neighborAgents, h_neighborAgents, mem_size_neighbor_agents, cudaMemcpyHostToDevice);
    
    // run find neighbors kernel
    findNeighborsKernel<<<grid, threads>>>(d_vehicleData, d_neighborIndices, d_neighborAgents, d_neighborData, 4.24f);
    
    // start avoidObstacle kernel
    steerToAvoidObstacles<<<grid, threads>>>(d_vehicleData, d_vehicleConst, d_steeringVectors, 1.f, NONE);
    
    // avoid close neighbors
    steerToAvoidCloseNeighbors<<<grid, threads>>>(d_vehicleData, d_vehicleConst, d_steeringVectors, d_neighborData, 0.f, 8.f, IGNORE_UNLESS_ZERO);
    
    // avoid neighbors
    steerToAvoidNeighbors<<<grid, threads>>>(d_vehicleData, d_vehicleConst, d_steeringVectors, d_neighborData, 3.f, 8.f, IGNORE_UNLESS_ZERO);
    
    // start wander kernel
    steerForWander2DKernel<<<grid, threads>>>(d_vehicleData, d_randomNumbers, elapsedTime, d_steeringVectors, d_wanderData, 1.f, IGNORE_UNLESS_ZERO);
    //CUT_CHECK_ERROR("kernel execution failed!");
    
    // start update kernel
    updateKernel<<<grid, threads>>>(d_vehicleData, d_vehicleConst, d_steeringVectors, elapsedTime, SPHERICAL_WRAP_AROUND);
    //CUT_CHECK_ERROR("kernel execution failed!");
    
    cudaMemcpy(h_vehicleData, d_vehicleData, mem_size_vehicle, cudaMemcpyDeviceToHost);
    
    first_run = 0;
}

void endWanderAround(void)
{
    cudaFree(d_vehicleData);
    cudaFree(d_vehicleConst);
    cudaFree(d_steeringVectors);
    cudaFree(d_randomNumbers);
    cudaFree(d_wanderData);
    cudaFree(d_neighborIndices);
    cudaFree(d_neighborAgents);
    cudaFree(d_neighborData);

    d_vehicleData = NULL;
    d_vehicleConst = NULL;
    d_steeringVectors = NULL;
    d_randomNumbers = NULL;
    d_wanderData = NULL;
    d_neighborIndices = NULL;
    d_neighborAgents = NULL;
    d_neighborData = NULL;
    
    delete rVec;
    rVec = NULL;
    
    first_run = 1;
}
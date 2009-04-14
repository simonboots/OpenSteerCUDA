#include <cuda_runtime.h>
#include <cutil.h>
#include <iostream>
#include "VehicleData.h"
#include "ObstacleData.h"
#include "RandomizedVector.h"
#include "WanderAroundCUDADefines.h"
#include "CUDAKernelOptions.cu"


__global__ void
steerToAvoidObstacles(VehicleData* vehicleData, float3 *steeringVectors);

__global__ __device__ void
steerForWander2DKernel(VehicleData *vehicleData, float *random, float dt, float3 *steeringVectors, float2 *wanderData, float blendFactor, kernel_options options);

__global__ void
updateKernel(VehicleData *vehicleData, float3 *steeringVectors, float elapsedTime);

using namespace OpenSteer;
using namespace std;


// device memory objects
static VehicleData* d_vehicleData = NULL;
static float3* d_steeringVectors = NULL;
static float* d_randomNumbers = NULL;
static float2* d_wanderData = NULL;
static int first_run = 1;

// host memory objects
static RandomizedVector* rVec = new RandomizedVector(2*NUM_OF_AGENTS);

void runWanderAroundKernel(VehicleData *h_vehicleData, ObstacleData *h_obstacleData, float elapsedTime)
{
    // init GPU
    int gpu_count;
    cudaGetDeviceCount(&gpu_count);
    if (gpu_count < 1) {
        return;
    }
    
    cudaSetDevice(0);
    
    dim3 grid(NUM_OF_AGENTS/TPB,1,1);
    dim3 threads(TPB,1,1);
    
    // allocate device memory
    const unsigned int mem_size_vehicle = sizeof(VehicleData);
    if (d_vehicleData == NULL) {
        cudaMalloc((void **)&d_vehicleData, mem_size_vehicle);
        cudaMemcpy(d_vehicleData, h_vehicleData, mem_size_vehicle, cudaMemcpyHostToDevice);
    }
    
    if (d_steeringVectors == NULL) {
        const unsigned int mem_size_steering = sizeof(float3) * NUM_OF_AGENTS;
        cudaMalloc((void **)&d_steeringVectors, mem_size_steering);
    }
    
    const unsigned int mem_size_random = rVec->size() * sizeof(float);
    if (d_randomNumbers == NULL) {
        cudaMalloc((void **)&d_randomNumbers, mem_size_random);
    }
    
    const unsigned int mem_size_wander = NUM_OF_AGENTS * sizeof(float2);
    if (d_wanderData == NULL) {
        cudaMalloc((void **)&d_wanderData, mem_size_wander);
        cudaMemset(d_wanderData, 0, mem_size_wander);
    }
    
    // first run initializations
    if (first_run == 1) {
        cudaMemcpyToSymbol("obstacles", h_obstacleData, sizeof(ObstacleData) * NUM_OF_OBSTACLES, 0, cudaMemcpyHostToDevice);
    }
    
    // renew random numbers
    rVec->renew();
    cudaMemcpy(d_randomNumbers, rVec->getVector(), mem_size_random, cudaMemcpyHostToDevice);
    
    // start avoidObstacle kernel
    steerToAvoidObstacles<<<grid, threads>>>(d_vehicleData, d_steeringVectors);
    
    // start wander kernel
    steerForWander2DKernel<<<grid, threads>>>(d_vehicleData, d_randomNumbers, elapsedTime, d_steeringVectors, d_wanderData, 1.f, IGNORE_UNLESS_ZERO);
    //CUT_CHECK_ERROR("kernel execution failed!");
    
    // start update kernel
    updateKernel<<<grid, threads>>>(d_vehicleData, d_steeringVectors, elapsedTime);
    //CUT_CHECK_ERROR("kernel execution failed!");
    
    cudaMemcpy(h_vehicleData, d_vehicleData, mem_size_vehicle, cudaMemcpyDeviceToHost);
    
    first_run = 0;
}

void endWanderAround(void)
{
    cudaFree(d_vehicleData);
    cudaFree(d_steeringVectors);
    cudaFree(d_randomNumbers);
    
    d_vehicleData = NULL;
    d_steeringVectors = NULL;
    d_randomNumbers = NULL;
}
#include <cuda_runtime.h>
#include <cutil.h>
#include <iostream>
#include "VehicleData.h"
#include "ObstacleData.h"
#include "RandomizedVector.h"
#include "CUDAKernelOptions.cu"


__global__ void
steerToAvoidObstacles(VehicleData* vehicleData, VehicleConst* vehicleConst, float3 *steeringVectors);

__global__ __device__ void
steerForWander2DKernel(VehicleData *vehicleData, float *random, float dt, float3 *steeringVectors, float2 *wanderData, float blendFactor, kernel_options options);

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
static int first_run = 1;

// host memory objects
static RandomizedVector* rVec = NULL;

void runWanderAroundKernel(VehicleData *h_vehicleData, VehicleConst *h_vehicleConst, int numOfVehicles, ObstacleData *h_obstacleData, int numOfObstacles, float elapsedTime)
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
    
    if (d_steeringVectors == NULL) {
        const unsigned int mem_size_steering = sizeof(float3) * numOfVehicles;
        cudaMalloc((void **)&d_steeringVectors, mem_size_steering);
    }
    
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
    
    // start avoidObstacle kernel
    steerToAvoidObstacles<<<grid, threads>>>(d_vehicleData, d_vehicleConst, d_steeringVectors);
    
    // start wander kernel
    steerForWander2DKernel<<<grid, threads>>>(d_vehicleData, d_randomNumbers, elapsedTime, d_steeringVectors, d_wanderData, 1.f, IGNORE_UNLESS_ZERO);
    //CUT_CHECK_ERROR("kernel execution failed!");
    
    // start update kernel
    updateKernel<<<grid, threads>>>(d_vehicleData, d_vehicleConst, d_steeringVectors, elapsedTime, NONE);
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

    d_vehicleData = NULL;
    d_vehicleConst = NULL;
    d_steeringVectors = NULL;
    d_randomNumbers = NULL;
    d_wanderData = NULL;
    
    delete rVec;
    rVec = NULL;
    
    first_run = 1;
}
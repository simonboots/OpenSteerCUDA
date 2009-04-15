#include <cuda_runtime.h>
#include <cutil.h>
#include <stdio.h>
#include "VehicleData.h"
#include "PathwayData.h"
#include "CUDAKernelOptions.cu"

__global__ void
steerToFollowPathKernel(VehicleData *vehicleData, float3 *steeringVectors, int *direction, float predictionTime);

__global__ void
steerToStayOnPathKernel(VehicleData *vehicleData, float3 *steeringVectors, float predictionTime);

__global__ void
updateKernel(VehicleData *vehicleData, float3 *steeringVectors, float elapsedTime);

// device memory objects
static float3 *d_steeringVectors = NULL;
static VehicleData *d_vehicleData = NULL;
static int *d_directions = NULL;

static int first_run = 1;

void runFollowPathKernel(VehicleData *h_vehicleData, int numOfVehicles, PathwayData *h_pathwayData, int *h_directions, float elapsedTime)
{
    // init GPU
    int gpu_count;
    cudaGetDeviceCount(&gpu_count);
    if (gpu_count < 1) {
        return;
    }
    
    cudaSetDevice(0);
    
    dim3 grid(numOfVehicles/TPB,1,1);
    dim3 threads(TPB, 1, 1);
    
    // allocate device memory
    const unsigned int mem_size_vehicle = sizeof(VehicleData);
    if (d_vehicleData == NULL) {
        cudaMalloc((void **)&d_vehicleData, mem_size_vehicle);
        cudaMemcpy(d_vehicleData, h_vehicleData, mem_size_vehicle, cudaMemcpyHostToDevice);
    }
    
    if (d_steeringVectors == NULL) {
        const unsigned int mem_size_steering = sizeof(float3) * numOfVehicles;
        cudaMalloc((void **)&d_steeringVectors, mem_size_steering);
    }
    
    if (d_directions == NULL) {
        const unsigned int mem_size_directions = sizeof(int) * numOfVehicles;
        cudaMalloc((void **)&d_directions, mem_size_directions);
        cudaMemcpy(d_directions, h_directions, mem_size_directions, cudaMemcpyHostToDevice);
    }
    
    // first run initializations
    if (first_run == 1) {
        cudaMemcpyToSymbol("pathway", h_pathwayData, sizeof(PathwayData), 0, cudaMemcpyHostToDevice);
        //CUT_CHECK_ERROR("cudaMemcpyToSymbol failed");
    }
    
    // start followPath kernel
    steerToFollowPathKernel<<<grid, threads>>>(d_vehicleData, d_steeringVectors, d_directions, 3.f);
    //steerToStayOnPathKernel<<<grid, threads>>>(d_vehicleData, d_steeringVectors, 3.f);
    //CUT_CHECK_ERROR("steerToFollowPathKernel execution failed");
    
    // start update kernel
    updateKernel<<<grid, threads>>>(d_vehicleData, d_steeringVectors, elapsedTime);
    //CUT_CHECK_ERROR("updateKernel execution failed");
    
    // copy vehicle data back to host memory
    cudaMemcpy(h_vehicleData, d_vehicleData, mem_size_vehicle, cudaMemcpyDeviceToHost);
    
    first_run = 0;
}

void endFollowPath(void)
{
    cudaFree(d_vehicleData);
    cudaFree(d_steeringVectors);
    cudaFree(d_directions);
    
    d_vehicleData = NULL;
    d_steeringVectors = NULL;
    d_directions = NULL;
}
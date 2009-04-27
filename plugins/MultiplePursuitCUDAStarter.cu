#include <cuda_runtime.h>
#include <cutil.h>
#include <stdio.h>
#include "VehicleData.h"
#include "RandomizedVector.h"
#include "CUDAKernelOptions.cu"

__global__ __device__ void
steerForPursuitKernel(VehicleData *vehicleData, float3 wandererPosition, float3 wandererVelocity, float3 *steeringVectors, float maxPredictionTime, float weight, kernel_options options);

__global__ void
updateKernel(VehicleData *vehicleData, VehicleConst *vehicleConst, float3 *steeringVectors, float elapsedTime, kernel_options options);

static float3 *d_steeringVectors = NULL;
static VehicleData *d_vehicleData = NULL;
static VehicleConst *d_vehicleConst = NULL;
static int first_time = 1;

void runMultiplePursuitKernel(VehicleData *h_vehicleData, VehicleConst *h_vehicleConst, int numOfVehicles, float3 wandererPosition, float3 wandererVelocity, float elapsedTime, int copy_vehicle_data)
{
    const float h_timeFactorTable[9] = {2.f, 4.f, 0.85f, 2.f, 0.8f, 1.8f, 0.5f, 1.f, 4.f};
    
    int gpu_count;
    cudaGetDeviceCount(&gpu_count);
    if (gpu_count < 1) {
        return;
    }
    
    cudaSetDevice(0);
    
    // copy time factor table
    if (first_time == 1) {
        cudaMemcpyToSymbol("timeFactorTable", h_timeFactorTable, sizeof(float) * 9, 0, cudaMemcpyHostToDevice);
    }

    dim3 grid(numOfVehicles/TPB,1,1);
    dim3 threads(TPB,1,1);
        
    // prepare memory for steeringVectors
    const unsigned int mem_size_steering = sizeof(float3) * numOfVehicles;
    if (d_steeringVectors == NULL) {
        cudaMalloc((void **) &d_steeringVectors, mem_size_steering);
    }
    
    cudaMemset(d_steeringVectors, 0, mem_size_steering);
    
    // prepare vehicle data
    const unsigned int mem_size_vehicle = sizeof(VehicleData);
    
    if (d_vehicleData == NULL || copy_vehicle_data == 1) {
        if (d_vehicleData == NULL)
            cudaMalloc((void **) &d_vehicleData, mem_size_vehicle);
        
        cudaMemcpy(d_vehicleData, h_vehicleData, mem_size_vehicle, cudaMemcpyHostToDevice);
    }
    
    const unsigned int mem_size_vehicle_const = sizeof(VehicleConst);
    if (d_vehicleConst == NULL) {
        cudaMalloc((void **) &d_vehicleConst, mem_size_vehicle_const);
        cudaMemcpy(d_vehicleConst, h_vehicleConst, mem_size_vehicle_const, cudaMemcpyHostToDevice);
    }
            
    // create and start timer
//    unsigned int timer = 0;
//    CUT_SAFE_CALL(cutCreateTimer(&timer));
//    CUT_SAFE_CALL(cutStartTimer(timer));
    
    // call steerForSeekKernel
    steerForPursuitKernel<<<grid, threads>>>(d_vehicleData, wandererPosition, wandererVelocity, d_steeringVectors, 20.f, 1.f, NONE);
    //CUT_CHECK_ERROR("Kernel execution failed");
    
    // stop and destroy timer
//    CUT_SAFE_CALL(cutStopTimer(timer));
//    printf("Raw processing time (steerForPursuitKernel): %f (ms) \n", cutGetTimerValue(timer));
//    CUT_SAFE_CALL(cutDeleteTimer(timer));
    
    // create and start timer
//    CUT_SAFE_CALL(cutCreateTimer(&timer));
//    CUT_SAFE_CALL(cutStartTimer(timer));
    
    // call updateKernel
    updateKernel<<<grid, threads>>>(d_vehicleData, d_vehicleConst, d_steeringVectors, elapsedTime, NONE);
    //CUT_CHECK_ERROR("Kernel execution failed");
        
    // stop and destroy timer
//    CUT_SAFE_CALL(cutStopTimer(timer));
//    printf("Raw processing time (updateKernel): %f (ms) \n", cutGetTimerValue(timer));
//    CUT_SAFE_CALL(cutDeleteTimer(timer));
//    CUT_SAFE_CALL(cutCreateTimer(&timer));
//    CUT_SAFE_CALL(cutStartTimer(timer));
    
    cudaMemcpy(h_vehicleData, d_vehicleData, mem_size_vehicle, cudaMemcpyDeviceToHost);
    
    // stop and destroy timer
//    CUT_SAFE_CALL(cutStopTimer(timer));
//    printf("Memcpy time: %f (ms) \n", cutGetTimerValue(timer));
//    CUT_SAFE_CALL(cutDeleteTimer(timer));    
    
    
    //cudaFree(vehicleData);
    
    first_time = 0;
}

void endMultiplePursuit(void)
{
    cudaFree(d_vehicleData);
    cudaFree(d_vehicleConst);
    cudaFree(d_steeringVectors);
    
    d_vehicleData = NULL;
    d_vehicleConst = NULL;
    d_steeringVectors = NULL;
}

#include <cuda_runtime.h>
#include <cutil.h>
#include <stdio.h>
#include "OpenSteer/VehicleData.h"
#include "CUDAKernelOptions.cu"

__global__ void
findFollowerKernel(VehicleData *vehicleData, float3 *seekVectors);

__global__ void
steerForSeekKernel(VehicleData *vehicleData, float3 *seekVectors, float3 *steeringVectors, float weight, kernel_options options);

__global__ void
updateKernel(VehicleData *vehicleData, VehicleConst *vehicleConst, float3 *steeringVectors, float elapsedTime, kernel_options options);

static VehicleData* d_vehicleData = NULL;
static VehicleConst* d_vehicleConst = NULL;
static float3* d_steeringVectors = NULL;
static float3* d_seekVectors = NULL;

void runPolonaiseKernel(VehicleData *h_vehicleData, VehicleConst *h_vehicleConst, int numOfAgents, float elapsedTime) {

    int gpu_count;
    cudaGetDeviceCount(&gpu_count);
    if (gpu_count < 1) {
        return;
    }
    
    cudaSetDevice(0);
    
    dim3 grid(numOfAgents/TPB,1,1);
    dim3 threads(TPB,1,1);
    
    // prepare memory for steeringVectors
    const unsigned int mem_size_steering = sizeof(float3) * numOfAgents;
    if (d_steeringVectors == NULL) {
        cudaMalloc((void **) &d_steeringVectors, mem_size_steering);
    }
    
    cudaMemset(d_steeringVectors, 0, mem_size_steering);

    
    // prepare vehicle data
    const unsigned int mem_size_vehicle = sizeof(VehicleData);
    if (d_vehicleData == NULL) {
        cudaMalloc((void **) &d_vehicleData, mem_size_vehicle);
        cudaMemcpy(d_vehicleData, h_vehicleData, mem_size_vehicle, cudaMemcpyHostToDevice);
    }
    
    const unsigned int mem_size_vehicle_const = sizeof(VehicleConst);
    if (d_vehicleConst == NULL) {
        cudaMalloc((void **)&d_vehicleConst, mem_size_vehicle_const);
        cudaMemcpy(d_vehicleConst, h_vehicleConst, mem_size_vehicle_const, cudaMemcpyHostToDevice);
    }
    
    // prepare steerForSeekKernel
    if (d_seekVectors == NULL) {
        const unsigned int mem_size_seek_vectors = sizeof(float3) * numOfAgents;
        cudaMalloc((void **) &d_seekVectors, mem_size_seek_vectors);
    }
    
    // create and start timer
//    unsigned int timer = 0;
//    CUT_SAFE_CALL(cutCreateTimer(&timer));
//    CUT_SAFE_CALL(cutStartTimer(timer));
    
    // call findFollowerKernel
    findFollowerKernel<<<grid, threads>>>(d_vehicleData, d_seekVectors);
    //CUT_CHECK_ERROR("Kernel execution failed");
    
    // stop and destroy timer
//    CUT_SAFE_CALL(cutStopTimer(timer));
//    printf("Raw processing time (findFollowerKernel): %f (ms) \n", cutGetTimerValue(timer));
//    CUT_SAFE_CALL(cutDeleteTimer(timer));
//    CUT_SAFE_CALL(cutCreateTimer(&timer));
//    CUT_SAFE_CALL(cutStartTimer(timer));

    // call steerForSeekKernel
    steerForSeekKernel<<<grid, threads>>>(d_vehicleData, d_seekVectors, d_steeringVectors, 1.f, NONE);
    //CUT_CHECK_ERROR("Kernel execution failed");
            
    // stop and destroy timer
//    CUT_SAFE_CALL(cutStopTimer(timer));
//    printf("Raw processing time (steerForSeekKernel): %f (ms) \n", cutGetTimerValue(timer));
//    CUT_SAFE_CALL(cutDeleteTimer(timer));
//    CUT_SAFE_CALL(cutCreateTimer(&timer));
//    CUT_SAFE_CALL(cutStartTimer(timer));

    // call updateKernel
    updateKernel<<<grid, threads>>>(d_vehicleData, d_vehicleConst, d_steeringVectors, elapsedTime, NONE);
    //CUT_CHECK_ERROR("Kernel execution failed");
    
    //cudaThreadSynchronize();
    
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
}

void endPolonaise(void)
{
    cudaFree(d_vehicleData);
    cudaFree(d_vehicleConst);
    cudaFree(d_steeringVectors);
    cudaFree(d_seekVectors);
    
    d_vehicleData = NULL;
    d_vehicleConst = NULL;
    d_steeringVectors = NULL;
    d_seekVectors = NULL;
}

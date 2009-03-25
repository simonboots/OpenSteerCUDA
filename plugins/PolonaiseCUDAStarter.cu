#include <cuda_runtime.h>
#include <cutil.h>
#include <stdio.h>
#include "vehicle_t.h"

__global__ void
steerForSeekKernel(vehicle_t *vehicleData, float2 *seekVectors, float2 *steeringVectors);

__global__ void
updateKernel(vehicle_t *vehicleData, float2 *steeringVectors, float elapsedTime);

static vehicle_t* d_vehicleData = NULL;
static float2* d_steeringVectors = NULL;
static int i = 0;

void runPolonaiseKernel(vehicle_t *h_vehicleData, float2 *h_seekVectors, int numOfAgents, float elapsedTime) {

    int gpu_count;
    cudaGetDeviceCount(&gpu_count);
    if (gpu_count < 1) {
        return;
    }
    
    cudaSetDevice(0);
    
    dim3 grid(numOfAgents/TPB,1,1);
    dim3 threads(TPB,1,1);
    
    // prepare memory for steeringVectors
    const unsigned int mem_size_steering = sizeof(float2) * numOfAgents;
    if (d_steeringVectors == NULL) {
        cudaMalloc((void **) &d_steeringVectors, mem_size_steering);
    }
    
    // prepare vehicle data
    const unsigned int mem_size_vehicle = sizeof(vehicle_t);
    
    if (d_vehicleData == NULL) {
        cudaMalloc((void **) &d_vehicleData, mem_size_vehicle);
        cudaMemcpy(d_vehicleData, h_vehicleData, mem_size_vehicle, cudaMemcpyHostToDevice);
    }
    
    // prepare steerForSeekKernel
    float2* d_seekVectors = NULL;
    const unsigned int mem_size_seek_vectors = sizeof(float2) * numOfAgents;
    cudaMalloc((void **) &d_seekVectors, mem_size_seek_vectors);
    cudaMemcpy(d_seekVectors, h_seekVectors, mem_size_seek_vectors, cudaMemcpyHostToDevice);

    // call steerForSeekKernel
    steerForSeekKernel<<<grid, threads>>>(d_vehicleData, d_seekVectors, d_steeringVectors);
    CUT_CHECK_ERROR("Kernel execution failed");
    
    cudaFree(d_seekVectors);

        
    // create and start timer
    unsigned int timer = 0;
    CUT_SAFE_CALL(cutCreateTimer(&timer));
    CUT_SAFE_CALL(cutStartTimer(timer));

    // call updateKernel
    updateKernel<<<grid, threads>>>(d_vehicleData, d_steeringVectors, elapsedTime);
    CUT_CHECK_ERROR("Kernel execution failed");
    
    cudaThreadSynchronize();
    
    // stop and destroy timer
    CUT_SAFE_CALL(cutStopTimer(timer));
    printf("Raw processing time: %f (ms) \n", cutGetTimerValue(timer));
    CUT_SAFE_CALL(cutDeleteTimer(timer));
    CUT_SAFE_CALL(cutCreateTimer(&timer));
    CUT_SAFE_CALL(cutStartTimer(timer));
    
    cudaMemcpy(h_vehicleData, d_vehicleData, mem_size_vehicle, cudaMemcpyDeviceToHost);
    
    // stop and destroy timer
    CUT_SAFE_CALL(cutStopTimer(timer));
    printf("Memcpy time: %f (ms) \n", cutGetTimerValue(timer));
    CUT_SAFE_CALL(cutDeleteTimer(timer));    
    
    
    //cudaFree(vehicleData);
}

void endPolonaise(void)
{
    cudaFree(d_vehicleData);
    cudaFree(d_steeringVectors);
}

#include <cuda_runtime.h>
#include <cutil.h>
#include <stdio.h>
#include "vehicle_t.h"
#include "PolonaiseCUDA.h"

__global__ void
polonaiseCUDAAccelerationKernel(vehicle_t *vehicleData, float elapsedTime);

static vehicle_t* d_data = NULL;
static int i = 0;

void runPolonaiseKernel(vehicle_t *data, int numOfAgents, float elapsedTime) {

    int gpu_count;
    cudaGetDeviceCount(&gpu_count);
    if (gpu_count < 1) {
        return;
    }
    
    cudaSetDevice(0);

    const unsigned int mem_size = sizeof(vehicle_t);
    
    if (d_data == NULL) {
        cudaMalloc((void **) &d_data, mem_size);
        cudaMemcpy(d_data, data, mem_size, cudaMemcpyHostToDevice);
    }
    
    printf("%p\n", (d_data + sizeof(float2) * NUM_OF_AGENTS));
    
    // create and start timer
    unsigned int timer = 0;
    CUT_SAFE_CALL(cutCreateTimer(&timer));
    CUT_SAFE_CALL(cutStartTimer(timer));

    dim3 grid(numOfAgents/TPB,1,1);
    dim3 threads(TPB,1,1);
    
    // call kernel
    polonaiseCUDAAccelerationKernel<<<grid, threads>>>(d_data, elapsedTime);
    CUT_CHECK_ERROR("Kernel execution failed");
    
    cudaThreadSynchronize();
    
    // stop and destroy timer
    CUT_SAFE_CALL(cutStopTimer(timer));
    printf("Raw processing time: %f (ms) \n", cutGetTimerValue(timer));
    CUT_SAFE_CALL(cutDeleteTimer(timer));
    CUT_SAFE_CALL(cutCreateTimer(&timer));
    CUT_SAFE_CALL(cutStartTimer(timer));
    
    cudaMemcpy(data, d_data, mem_size, cudaMemcpyDeviceToHost);
    
    // stop and destroy timer
    CUT_SAFE_CALL(cutStopTimer(timer));
    printf("Memcpy time: %f (ms) \n", cutGetTimerValue(timer));
    CUT_SAFE_CALL(cutDeleteTimer(timer));    
    
    
    //cudaFree(d_data);
}

void endPolonaise(void)
{
    cudaFree(d_data);
}

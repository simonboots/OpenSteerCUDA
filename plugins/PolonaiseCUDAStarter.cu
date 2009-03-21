#include <cuda_runtime.h>
#include <cutil.h>
#include "vehicle_t.h"

#define TPB 128

__global__ void
PolonaiseKernel(vehicle_t *, float);

void runPolonaiseKernel(vehicle_t *data, int numOfAgents, float elapsedTime) {
    int gpu_count;
    cudaGetDeviceCount(&gpu_count);
    if (gpu_count < 1) {
        return;
    }
    
    cudaSetDevice(0);
    
    const unsigned int mem_size = sizeof(vehicle_t) * numOfAgents;
    
    vehicle_t* d_data;
    cudaMalloc((void **) &d_data, mem_size);
    cudaMemcpy(d_data, data, mem_size, cudaMemcpyHostToDevice);
    
    dim3 grid(numOfAgents/TPB,1,1);
    dim3 threads(TPB,1,1);
    
    // call kernel
    PolonaiseKernel<<<grid, threads>>>(d_data, elapsedTime);
    CUT_CHECK_ERROR("Kernel execution failed");
    
    cudaThreadSynchronize();
    
    cudaMemcpy(data, d_data, mem_size, cudaMemcpyDeviceToHost);
    
    cudaFree(d_data);
}

#include <cuda_runtime.h>
#include <cutil.h>

#define TPB 512

__global__ void
PolonaiseKernel(float *);

void runPolonaiseKernel(float *data, int numOfAgents) {
    int gpu_count;
    cudaGetDeviceCount(&gpu_count);
    if (gpu_count < 1) {
        return;
    }
    
    cudaSetDevice(0);
    
    const unsigned int mem_size = sizeof(float) * numOfAgents * 6;
    
    float* d_data;
    cudaMalloc((void **) &d_data, mem_size);
    cudaMemcpy(d_data, data, mem_size, cudaMemcpyHostToDevice);
    
    dim3 grid(numOfAgents/TPB,1,1);
    dim3 threads(TPB,1,1);
    
    // call kernel
    PolonaiseKernel<<<grid, threads>>>(d_data);
    CUT_CHECK_ERROR("Kernel execution failed");
    
    cudaThreadSynchronize();
    
    cudaMemcpy(data, d_data, mem_size, cudaMemcpyDeviceToHost);
    
    cudaFree(d_data);
}

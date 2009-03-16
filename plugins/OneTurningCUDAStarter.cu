#include <cuda_runtime.h>

__global__ void
OneTurningCUDAKernel(int* values);

void runCUDAKernel(int* values, char* device_name)
{
    int gpu_count;
    cudaGetDeviceCount(&gpu_count);
    if (gpu_count < 1) {
        return;
    }
    
    struct cudaDeviceProp device_prop;
    cudaGetDeviceProperties(&device_prop, 0);
    strcpy(device_name, device_prop.name);
    
    cudaSetDevice(0);
    
    const unsigned int mem_size = sizeof(int) * 2;
    
    int* d_data;
    cudaMalloc((void **) &d_data, mem_size);
    cudaMemcpy(d_data, values, mem_size, cudaMemcpyHostToDevice);
    
    dim3 grid(1,1,1);
    dim3 threads(1,1,1);
    
    // call kernel
    OneTurningCUDAKernel<<<grid, threads>>>(d_data);
    
    cudaMemcpy(values, d_data, mem_size, cudaMemcpyDeviceToHost);
    
    cudaFree(d_data);
}
#ifndef _STEER_FOR_SEEK_KERNEL_H_
#define _STEER_FOR_SEEK_KERNEL_H_

#include "VehicleData.h"
#include "PolonaiseCUDADefines.h"

//  (((float*)seekVectors)[id]) 

__global__ void
steerForSeekKernel(VehicleData *vehicleData, float3 *seekVectors, float3 *steeringVectors)
{
    int id = (blockIdx.x * blockDim.x + threadIdx.x);    
    int blockOffset = (blockDim.x * blockIdx.x * 3);
    
    // shmem for forward vector (also stores steering result) (TPB * 12 bytes)
    __shared__ float3 forwardAndSteer[TPB];
    
    // shmem for position (TPB * 12 bytes)
    __shared__ float3 position[TPB];
    
    // shmem for seek vectors (TPB * 12 bytes)
    __shared__ float3 seek[TPB];
    
    // fill forward vector shmem
    ((float*)forwardAndSteer)[threadIdx.x] = ((float*)(*vehicleData).forward)[blockOffset + threadIdx.x];
    ((float*)forwardAndSteer)[threadIdx.x + blockDim.x] = ((float*)(*vehicleData).forward)[blockOffset + threadIdx.x + blockDim.x]; 
    ((float*)forwardAndSteer)[threadIdx.x + 2*blockDim.x] = ((float*)(*vehicleData).forward)[blockOffset + threadIdx.x + 2*blockDim.x];
    
    // fill speed shmem
    float speed = (*vehicleData).speed[id];
    
    // fill position vector shmem
    ((float*)position)[threadIdx.x] = ((float*)(*vehicleData).position)[blockOffset + threadIdx.x];
    ((float*)position)[threadIdx.x + blockDim.x] = ((float*)(*vehicleData).position)[blockOffset + threadIdx.x + blockDim.x];
    ((float*)position)[threadIdx.x + 2*blockDim.x] = ((float*)(*vehicleData).position)[blockOffset + threadIdx.x + 2*blockDim.x];
    
    // fill seek vector shmem
    ((float*)seek)[threadIdx.x] = ((float*)seekVectors)[blockOffset + threadIdx.x];
    ((float*)seek)[threadIdx.x + blockDim.x] = ((float*)seekVectors)[blockOffset + threadIdx.x + blockDim.x];
    ((float*)seek)[threadIdx.x + 2*blockDim.x] = ((float*)seekVectors)[blockOffset + threadIdx.x + 2*blockDim.x];
    
    __syncthreads();
    
    // calculate steering vectors
    forwardAndSteer[threadIdx.x].x = (seek[threadIdx.x].x - position[threadIdx.x].x) - (forwardAndSteer[threadIdx.x].x * speed);
    forwardAndSteer[threadIdx.x].y = (seek[threadIdx.x].y - position[threadIdx.x].y) - (forwardAndSteer[threadIdx.x].y * speed);
    forwardAndSteer[threadIdx.x].z = (seek[threadIdx.x].z - position[threadIdx.x].z) - (forwardAndSteer[threadIdx.x].z * speed);

    __syncthreads();
    
    // copy steering vectors to global memory
    ((float*)steeringVectors)[blockOffset + threadIdx.x] = ((float*)forwardAndSteer)[threadIdx.x];
    ((float*)steeringVectors)[blockOffset + threadIdx.x + blockDim.x] = ((float*)forwardAndSteer)[threadIdx.x + blockDim.x];
    ((float*)steeringVectors)[blockOffset + threadIdx.x + 2*blockDim.x] = ((float*)forwardAndSteer)[threadIdx.x + 2*blockDim.x];
}

#endif // _STEER_FOR_SEEK_KERNEL_H_
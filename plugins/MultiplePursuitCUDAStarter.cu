#include <cuda_runtime.h>
#include <cutil.h>
#include <stdio.h>
#include "VehicleData.h"
#include "RandomizedVector.h"
#include "MultiplePursuitCUDADefines.h"

__global__ __device__ void
steerForPursuitKernel(VehicleData *vehicleData, float3 wandererPosition, float3 wandererVelocity, float3 *steeringVectors, float maxPredictionTime);

__global__ void
updateKernel(VehicleData *vehicleData, float3 *steeringVectors, float elapsedTime);

static float3 *d_steeringVectors = NULL;
static VehicleData *d_vehicleData = NULL;
static float *d_randomVectors = NULL;
//static OpenSteer::RandomizedVector *randomizedVec = new OpenSteer::RandomizedVector(NUM_OF_AGENTS);
//static unsigned int iterations = 0;
static int first_time = 1;

void runMultiplePursuitKernel(VehicleData *h_vehicleData, float3 wandererPosition, float3 wandererVelocity, float elapsedTime, int copy_vehicle_data)
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

    dim3 grid(NUM_OF_AGENTS/TPB,1,1);
    dim3 threads(TPB,1,1);
        
    // prepare memory for steeringVectors
    const unsigned int mem_size_steering = sizeof(float3) * NUM_OF_AGENTS;
    if (d_steeringVectors == NULL) {
        cudaMalloc((void **) &d_steeringVectors, mem_size_steering);
    }
    
    // prepare vehicle data
    const unsigned int mem_size_vehicle = sizeof(VehicleData);
    
    if (d_vehicleData == NULL || copy_vehicle_data == 1) {
        if (d_vehicleData == NULL)
            cudaMalloc((void **) &d_vehicleData, mem_size_vehicle);
        
        cudaMemcpy(d_vehicleData, h_vehicleData, mem_size_vehicle, cudaMemcpyHostToDevice);
    }
    
    // copy random vector
//    const unsigned int mem_size_random = sizeof(float) * randomizedVec->size();
//    if (d_randomVectors == NULL) {
//        cudaMalloc((void **) &d_randomVectors, mem_size_random);
//    }
    
//    if (iterations % 20 == 0) {
//        randomizedVec->renew();
//        cudaMemcpy(d_randomVectors, randomizedVec->getVector(), mem_size_random, cudaMemcpyHostToDevice);
//    }
        
    // create and start timer
//    unsigned int timer = 0;
//    CUT_SAFE_CALL(cutCreateTimer(&timer));
//    CUT_SAFE_CALL(cutStartTimer(timer));
    
    // call steerForSeekKernel
    steerForPursuitKernel<<<grid, threads>>>(d_vehicleData, wandererPosition, wandererVelocity, d_steeringVectors, 20.f);
    //CUT_CHECK_ERROR("Kernel execution failed");
    
    // stop and destroy timer
//    CUT_SAFE_CALL(cutStopTimer(timer));
//    printf("Raw processing time (steerForPursuitKernel): %f (ms) \n", cutGetTimerValue(timer));
//    CUT_SAFE_CALL(cutDeleteTimer(timer));
    
    // create and start timer
//    CUT_SAFE_CALL(cutCreateTimer(&timer));
//    CUT_SAFE_CALL(cutStartTimer(timer));
    
    // call updateKernel
    updateKernel<<<grid, threads>>>(d_vehicleData, d_steeringVectors, elapsedTime);
    //CUT_CHECK_ERROR("Kernel execution failed");
    
    cudaThreadSynchronize();
    
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
    cudaFree(d_steeringVectors);
    
    d_vehicleData = NULL;
    d_steeringVectors = NULL;
}

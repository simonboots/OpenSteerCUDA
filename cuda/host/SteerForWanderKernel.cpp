#include "SteerForWanderKernel.h"
#include <cuda_runtime.h>
#include <iostream>

using namespace OpenSteer;

OpenSteer::SteerForWanderKernel::SteerForWanderKernel()
{
    d_randomNumbers = NULL;
    d_wanderData = NULL;
    randomizedVector = NULL;
    threadsPerBlock = 128;
}

OpenSteer::SteerForWanderKernel::~SteerForWanderKernel() {}

void OpenSteer::SteerForWanderKernel::init()
{
    // random number generator
    randomizedVector = new RandomizedVector(2*getNumberOfAgents());
    
    // device memory for wander data
    mem_size_wander = getNumberOfAgents()*sizeof(float2);
    cudaError_t retval = cudaMalloc((void **)&d_wanderData, mem_size_wander);
    if (retval != cudaSuccess)
        std::cout << "Error while allocating d_wanderData memory: " << cudaGetErrorString(retval) << std::endl;
    
    // device memory for random numbers
    mem_size_random = randomizedVector->size() * sizeof(float);
    retval = cudaMalloc((void **)&d_randomNumbers, mem_size_random);
    if (retval != cudaSuccess)
        std::cout << "Error while allocating d_randomNumbers memory: " << cudaGetErrorString(retval) << std::endl;
        
}

void OpenSteer::SteerForWanderKernel::run()
{
    
}

void OpenSteer::SteerForWanderKernel::close()
{
    if (d_wanderData != NULL) {
        cudaFree(d_wanderData);
        d_wanderData = NULL;        
    }
    
    if (d_randomNumbers != NULL) {
        cudaFree(d_randomNumbers);
        d_randomNumbers = NULL;
    }
    
    if (randomizedVector != NULL) {
        delete randomizedVector;
        randomizedVector = NULL;        
    }
}

void OpenSteer::SteerForWanderKernel::reset()
{
    close();
    init();
}

#include "SteerToAvoidObstacles.h"
#include <cuda_runtime.h>
#include "OpenSteer/VehicleData.h"
#include "OpenSteer/ObstacleData.h"
#include "CUDAKernelOptions.cu"
#include <iostream>

using namespace OpenSteer;
using namespace std;

__global__ void
steerToAvoidObstacles(VehicleData* vehicleData, VehicleConst* vehicleConst, float3 *steeringVectors, float weight, kernel_options options);

OpenSteer::SteerToAvoidObstacles::SteerToAvoidObstacles(float weight, kernel_options options)
{
    threadsPerBlock = 128;
    this->weight = weight;
    this->options = options;
}

OpenSteer::SteerToAvoidObstacles::~SteerToAvoidObstacles() {}

void OpenSteer::SteerToAvoidObstacles::init()
{
    // nothing to do
}

void OpenSteer::SteerToAvoidObstacles::run()
{
    steerToAvoidObstacles<<<gridDim(), blockDim()>>>(getVehicleData(), getVehicleConst(), getSteeringVectors(), weight, options);
}

void OpenSteer::SteerToAvoidObstacles::close()
{
    // nothing to do
}

void OpenSteer::SteerToAvoidObstacles::setObstacles(ObstacleData *obstacleData, int numOfObstacles)
{
    cudaMemcpyToSymbol("d_obstacles", obstacleData, sizeof(ObstacleData) * numOfObstacles, 0, cudaMemcpyHostToDevice);
    cudaMemcpyToSymbol("d_numOfObstacles", &numOfObstacles, sizeof(int), 0, cudaMemcpyHostToDevice);
}


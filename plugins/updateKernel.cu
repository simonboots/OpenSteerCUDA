#ifndef _UPDATE_KERNEL_H_
#define _UPDATE_KERNEL_H_

#include "VehicleData.h"
#include "CUDAVectorUtilities.cu"
#include "MultiplePursuitCUDADefines.h"
#include <stdio.h>
#include <cutil.h>

#define CHECK_BANK_CONFLICTS 1
#if CHECK_BANK_CONFLICTS
#define FV(i) (CUT_BANK_CHECKER(follow_velocity, i))
#define V(i) (CUT_BANK_CHECKER(velocity, i))
#define SA(i) (CUT_BANK_CHECKER(smooth_acceleration, i))
#else
#define FV(i) follow_velocity[i]
#define V(i) velocity[i]
#define SA(i) smooth_acceleration[i]
#endif

__global__ void
updateKernel(VehicleData *vehicleData, float3 *steeringVectors, float elapsedTime)
{
    int id = (blockIdx.x * blockDim.x + threadIdx.x);
    //int numOfAgents = gridDim.x * blockDim.x;
    
    // shared memory for follow_velocity
    __shared__ float3 follow_velocity[TPB];
    
    // shared memory for velocity vector
    __shared__ float3 velocity[TPB];
    
    // shared memory for smooth_acceleration
    __shared__ float3 smooth_acceleration[TPB];
        
    // slow global memory access
    float speed = (*vehicleData).speed[id];
    V(threadIdx.x) = float3Mul((*vehicleData).forward[id], speed);
    FV(threadIdx.x) = steeringVectors[id];
    SA(threadIdx.x) = (*vehicleData).smoothedAcceleration[id];
    
    __syncthreads();
    
    // adjustRawSteeringForce
    float3 adjustedForce;
    
    float maxAdjustedSpeed = 0.2f * (*vehicleData).maxSpeed[id];
    
    if ((speed > maxAdjustedSpeed) || (FV(threadIdx.x).x == 0.f && FV(threadIdx.x).z == 0.f)) {
        adjustedForce = FV(threadIdx.x);
    } else {
        float cosine = interpolate (__powf(speed / maxAdjustedSpeed, 20), 1.0f, -1.0f);
        adjustedForce = limitMaxDeviationAngle(FV(threadIdx.x), cosine, float3Div(V(threadIdx.x), speed));
    }
    
    float3 clippedForce = float3TruncateLength(adjustedForce, (*vehicleData).maxForce[id]);
    float3 new_acceleration = float3Div(clippedForce, (*vehicleData).mass[id]);
    
    if (elapsedTime > 0) {
        float smoothRate = clip(9 * elapsedTime, 0.15f, 0.4f);
        SA(threadIdx.x) = float3BlendIn(smoothRate, new_acceleration, SA(threadIdx.x));
    }
    
    __syncthreads();
    
    V(threadIdx.x).x += SA(threadIdx.x).x * elapsedTime;
    V(threadIdx.x).z += SA(threadIdx.x).z * elapsedTime;
    
    V(threadIdx.x) = float3TruncateLength(V(threadIdx.x), (*vehicleData).maxSpeed[id]);
    
    __syncthreads(); // position is re-written
    
    (*vehicleData).position[id] = make_float3((*vehicleData).position[id].x + (V(threadIdx.x).x * elapsedTime),
                                              (*vehicleData).position[id].y + (V(threadIdx.x).y * elapsedTime),
                                              (*vehicleData).position[id].z + (V(threadIdx.x).z * elapsedTime));
    
    // regenerate local space
    if (speed > 0)
    {
        float3 forwardVector = float3Div(V(threadIdx.x), speed);
        float3 side = float3Cross(forwardVector, make_float3(0.f, 1.f, 0.f));
        (*vehicleData).side[id] = float3Normalize(side);
    }
    
    // copy data back to global memory
    speed = float3Length(V(threadIdx.x));
    (*vehicleData).speed[id] = speed;
    (*vehicleData).forward[id] = float3Div(V(threadIdx.x), speed);
    (*vehicleData).smoothedAcceleration[id] = SA(threadIdx.x);
}

#endif // _UPDATE_KERNEL_H_
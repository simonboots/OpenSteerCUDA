#ifndef _UPDATE_KERNEL_H_
#define _UPDATE_KERNEL_H_

#include "vehicle_t.h"
#include "CUDAVectorUtilities.cu"
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
updateKernel(vehicle_t *vehicleData, float2 *steeringVectors, float elapsedTime)
{
    int id = (blockIdx.x * blockDim.x + threadIdx.x);
    int numOfAgents = gridDim.x * blockDim.x;
    
    // shared memory for follow_velocity
    __shared__ float3 follow_velocity[TPB];
    
    // shared memory for velocity vector
    __shared__ float3 velocity[TPB];
    
    // shared memory for smooth_acceleration
    __shared__ float3 smooth_acceleration[TPB];
        
    // slow global memory access (global memory is float2!)
    V(threadIdx.x) = make_float3((*vehicleData).velocity[id].x, 0.f, (*vehicleData).velocity[id].y);
    FV(threadIdx.x).x = steeringVectors[id].x;
    FV(threadIdx.x).z = steeringVectors[id].y;
    FV(threadIdx.x).y = 0.f;
    SA(threadIdx.x) = make_float3((*vehicleData).smoothedAcceleration[id].x, 0.f, (*vehicleData).smoothedAcceleration[id].y);
    
    __syncthreads();
    
    // adjustRawSteeringForce
    float3 adjustedForce;
    
    float maxAdjustedSpeed = 0.2f * MAX_SPEED;
    float speed = float3Length(V(threadIdx.x));
    
    if ((speed > maxAdjustedSpeed) || (FV(threadIdx.x).x == 0.f && FV(threadIdx.x).z == 0.f)) {
        adjustedForce = make_float3(FV(threadIdx.x).x, 0.f, FV(threadIdx.x).z);
    } else {
        float cosine = interpolate (__powf(speed / maxAdjustedSpeed, 20), 1.0f, -1.0f);
        adjustedForce = limitMaxDeviationAngle(FV(threadIdx.x), cosine, make_float3(V(threadIdx.x).x / speed, 0.f, V(threadIdx.x).y / speed));
    }
    
    float3 clippedForce = float3TruncateLength(adjustedForce, MAX_FORCE);
    float3 new_acceleration = make_float3(clippedForce.x / MASS, 0.f, clippedForce.z / MASS);
    
    if (elapsedTime > 0) {
        float smoothRate = clip(9 * elapsedTime, 0.15f, 0.4f);
        SA(threadIdx.x) = float3BlendIn(smoothRate, new_acceleration, SA(threadIdx.x));
    }
    
    __syncthreads();
    
    V(threadIdx.x).x += SA(threadIdx.x).x * elapsedTime;
    V(threadIdx.x).z += SA(threadIdx.x).z * elapsedTime;
    
    V(threadIdx.x) = float3TruncateLength(V(threadIdx.x), MAX_SPEED);
    
    __syncthreads(); // position is re-written
    
    (*vehicleData).position[id] = make_float2((*vehicleData).position[id].x + (V(threadIdx.x).x * elapsedTime),
                                              (*vehicleData).position[id].y + (V(threadIdx.x).z * elapsedTime));
    
    // regenerate local space
    if (speed > 0)
    {
        float3 forwardVector = make_float3(V(threadIdx.x).x / speed, 0.f, V(threadIdx.x).z / speed);
        float3 side = float3Cross(forwardVector, make_float3(0.f, 1.f, 0.f));
        float3 side_norm = float3Normalize(side);
        (*vehicleData).side[id] = make_float2(side_norm.x, side_norm.z);
    }
    
    // copy data back to global memory
    (*vehicleData).velocity[id] = make_float2(V(threadIdx.x).x, V(threadIdx.x).z);
    (*vehicleData).smoothedAcceleration[id] = make_float2(SA(threadIdx.x).x, SA(threadIdx.x).z);
}

#endif // _UPDATE_KERNEL_H_
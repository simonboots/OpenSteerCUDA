#ifndef _POLONAISE_CUDA_ACCELERATION_KERNEL_H_
#define _POLONAISE_CUDA_ACCELERATION_KERNEL_H_

#include "vehicle_t.h"
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


__device__ float
interpolate(float alpha, float x0, float x1);

__device__ float
float3length(float3 value);

__device__ float
float3lengthSquared(float3 value);

__device__ float3
limitMaxDeviationAngle(float3 source, float cosineOfConeAngle, float3 basis);

__device__ float
float3dot (float3 v1, float3 v2);

__device__ float3
float3ParallelComponent(float3 source, float3 unitBasis);

__device__ float3
float3PerpendicularComponent(float3 source, float3 unitBasis);

__device__ float3
float3Normalize(float3 value);

__device__ float3
float3TruncateLength(float3 vec, float maxLength);

__device__ float
clip(float x, float min, float max);

__device__ float3
float3BlendIn(float smoothRate, float3 newvalue, float3 smoothedAccu);

__device__ float3
float3cross(float3 a, float3 b);

__global__ void
updateKernel(vehicle_t *vehicleData, float2 *steeringVectors, float elapsedTime)
{
    int id = (blockIdx.x * blockDim.x + threadIdx.x);
    int numOfAgents = gridDim.x * blockDim.x;
    
    // find follower
    int follower = ((id + blockDim.x/8) % numOfAgents);
    
    // shared memory for follow_velocity
    __shared__ float3 follow_velocity[TPB];
    
    // shared memory for forward vector
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
    float speed = float3length(V(threadIdx.x));
    
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
        float3 side = float3cross(forwardVector, make_float3(0.f, 1.f, 0.f));
        float3 side_norm = float3Normalize(side);
        (*vehicleData).side[id] = make_float2(side_norm.x, side_norm.z);
    }
    
    // copy data back to global memory
    (*vehicleData).velocity[id] = make_float2(V(threadIdx.x).x, V(threadIdx.x).z);
    (*vehicleData).smoothedAcceleration[id] = make_float2(SA(threadIdx.x).x, SA(threadIdx.x).z);
}

__device__ float
interpolate(float alpha, float x0, float x1)
{
    return x0 + ((x1 - x0) * alpha);
}

__device__ float3
float3Interpolate(float alpha, float3 x0, float3 x1) {
    return make_float3(x0.x + ((x1.x - x0.x) * alpha),
                       x0.y + ((x1.y - x0.y) * alpha),
                       x0.z + ((x1.z - x0.z) * alpha));
}

__device__ float3
limitMaxDeviationAngle(float3 source, float cosineOfConeAngle, float3 basis)
{
    float sourcelength = float3length(source);
    if (sourcelength == 0) return source;
    
    float3 direction = make_float3(source.x / sourcelength, source.y / sourcelength, source.z / sourcelength);
    float cosineOfSourceAngle = float3dot(direction, basis);
    
    if (cosineOfSourceAngle >= cosineOfConeAngle) return source;
    
    float3 perp = float3PerpendicularComponent(source, basis);
    float3 unitPerp = float3Normalize(perp);
    
    float perpDist = sqrtf(1 - (cosineOfConeAngle * cosineOfConeAngle));
    float3 c0 = make_float3(basis.x * cosineOfConeAngle, basis.y * cosineOfConeAngle, basis.z * cosineOfConeAngle);
    float3 c1 = make_float3(unitPerp.x * perpDist, unitPerp.y * perpDist, unitPerp.z * perpDist);
    
    return make_float3((c0.x + c1.x) * sourcelength, (c0.y + c1.y) * sourcelength, (c0.z + c1.z) * sourcelength); 
}

__device__ float
float3length(float3 value)
{
    return sqrtf(float3lengthSquared(value));
}

__device__ float
float3lengthSquared(float3 value)
{
    return float3dot(value, value);
}

__device__ float
float3dot (float3 v1, float3 v2)
{
    return (v1.x * v2.x) + (v1.y * v2.y) + (v1.z * v2.z);
}

__device__ float3
float3cross(float3 a, float3 b)
{
    return make_float3((a.y * b.y) - (a.z * b.y),
                       (a.z * b.x) - (a.x * b.z),
                       (a.x * b.y) - (a.y * b.x));
}

__device__ float3
float3ParallelComponent(float3 source, float3 unitBasis)
{
    float projection = float3dot(source, unitBasis);
    return make_float3(unitBasis.x * projection, unitBasis.y * projection, unitBasis.z * projection);
}

__device__ float3
float3PerpendicularComponent(float3 source, float3 unitBasis)
{
    float3 parallelComponent = float3ParallelComponent(source, unitBasis);
    return make_float3(source.x - parallelComponent.x, source.y - parallelComponent.y, source.z - parallelComponent.z);
}

__device__ float3
float3Normalize(float3 value)
{
    float length = float3length(value);
    return (length>0) ? make_float3(value.x / length, value.y / length, value.z / length) : value;
}

__device__ float3
float3TruncateLength(float3 vec, float maxLength)
{
    float lengthSquared = maxLength * maxLength;
    float vecLengthSquared = float3lengthSquared(vec);
    if (vecLengthSquared <= lengthSquared)
        return vec;
    else {
        float vecLength = sqrtf(vecLengthSquared);
        float ratio = maxLength / vecLength;
        
        return make_float3(vec.x * ratio, vec.y * ratio, vec.z * ratio);
    }
}

__device__ float
clip(float x, float min, float max)
{
    if (x < min) return min;
    if (x > max) return max;
    return x;
}

__device__ float3
float3BlendIn(float smoothRate, float3 newvalue, float3 smoothedAccu)
{
    return float3Interpolate(clip(smoothRate, 0, 1), smoothedAccu, newvalue);
}


#endif // _POLONAISE_CUDA_ACCELERATION_KERNEL_H_
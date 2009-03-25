#ifndef _POLONAISE_CUDA_KERNEL_H_
#define _POLONAISE_CUDA_KERNEL_H_

#include "vehicle_t.h"
#include "PolonaiseCUDA.h"

__device__ void
regenerateLocalSpace(vehicle_t *data, int agentID, float3 newVelocity);

__device__ float3
float3BlendIn(float smoothRate, float3 newvalue, float3 smoothedAccu);

__device__ float
clip(float x, float min, float max);

__device__ float3
float3TruncateLength(float3 vec, float maxLength);

__device__ float3
float3Normalize(float3 value);

__device__ float3
float3PerpendicularComponent(float3 source, float3 unitBasis);

__device__ float3
float3ParallelComponent(float3 source, float3 unitBasis);

__device__ float
float3length(float3 value);

__device__ float
float3lengthSquared(float3 value);

__device__ float
float3dot (float3 v1, float3 v2);

__device__ float3
float3cross(float3 a, float3 b);

__device__ float3
limitMaxDeviationAngle(float3 source, float cosineOfConeAngle, float3 basis);

__device__ float3
float3Interpolate(float alpha, float3 x0, float3 x1);

__device__ float
interpolate(float alpha, float x0, float x1);

__device__ float3
adjustRawSteeringForce(vehicle_t *data, int agentID);

__device__ void
applySteeringForce(vehicle_t *data, int agentID, float elapsedTime);

__global__ void
PolonaiseKernel(vehicle_t *data, float elapsedTime)
{
    int id = (blockIdx.x * blockDim.x + threadIdx.x);
    int numOfAgents = gridDim.x * blockDim.x;
    
    // shared memory
    __shared__ vehicle_t agentData[TPB];
    // copy global memory to shared
    agentData[threadIdx.x].position.x = data[id].position.x;
    agentData[threadIdx.x].position.y = data[id].position.y;
    agentData[threadIdx.x].position.z = data[id].position.z;
    agentData[threadIdx.x].new_position.x = data[id].new_position.x;
    agentData[threadIdx.x].new_position.y = data[id].new_position.y;
    agentData[threadIdx.x].new_position.z = data[id].new_position.z;
    
    agentData[threadIdx.x] = data[id];
    
    // find follower
    int follower = ((id + 1) % numOfAgents);
    
    agentData[threadIdx.x].follow_velocity.x = (data[follower].position.x - agentData[threadIdx.x].position.x) - agentData[threadIdx.x].velocity.x;
    agentData[threadIdx.x].follow_velocity.y = (data[follower].position.y - agentData[threadIdx.x].position.y) - agentData[threadIdx.x].velocity.y;
    agentData[threadIdx.x].follow_velocity.z = (data[follower].position.z - agentData[threadIdx.x].position.z) - agentData[threadIdx.x].velocity.z;
    
    applySteeringForce(agentData, threadIdx.x, elapsedTime);
    
    // copy shared mem to global memory
    data[id] = agentData[threadIdx.x];
}

__device__ void
applySteeringForce(vehicle_t *data, int agentID, float elapsedTime)
{
    float3 adjustedForce = adjustRawSteeringForce(data, agentID);
    float3 clippedForce = float3TruncateLength(adjustedForce, data[agentID].maxForce);
    float mass = data[agentID].mass;
    float3 newAcceleration = make_float3(clippedForce.x / mass, clippedForce.y / mass, clippedForce.z / mass);
    float3 newVelocity = data[agentID].velocity;

    if (elapsedTime > 0) {
        float smoothRate = clip(9 * elapsedTime, 0.15f, 0.4f);
        data[agentID].smoothedAcceleration = float3BlendIn(smoothRate, newAcceleration, data[agentID].smoothedAcceleration);
    }
    
    newVelocity.x += data[agentID].smoothedAcceleration.x * elapsedTime;
    newVelocity.y += data[agentID].smoothedAcceleration.y * elapsedTime;
    newVelocity.z += data[agentID].smoothedAcceleration.z * elapsedTime;
    
    newVelocity = float3TruncateLength(newVelocity, data[agentID].maxSpeed);
    data[agentID].speed = float3length(newVelocity);
    
    data[agentID].new_position = make_float3(data[agentID].position.x + (newVelocity.x * elapsedTime),
                                             data[agentID].position.y + (newVelocity.y * elapsedTime),
                                             data[agentID].position.z + (newVelocity.z * elapsedTime));
    
    data[agentID].velocity = newVelocity;
    
    regenerateLocalSpace(data, agentID, newVelocity);
}

__device__ float3
adjustRawSteeringForce(vehicle_t *data, int agentID)
{
    float maxAdjustedSpeed = 0.2f * data[agentID].maxSpeed;
    if ((data[agentID].speed > maxAdjustedSpeed) || (data[agentID].follow_velocity.x == 0.f && data[agentID].follow_velocity.y == 0.f && data[agentID].follow_velocity.z == 0.f)) {
        return make_float3(data[agentID].follow_velocity.x, data[agentID].follow_velocity.y, data[agentID].follow_velocity.z);
    } else {
        float range = data[agentID].speed / maxAdjustedSpeed;
        float cosine = interpolate (__powf(range, 20), 1.0f, -1.0f);
        return limitMaxDeviationAngle(data[agentID].follow_velocity, cosine, data[agentID].forward);
    }
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

__device__ void
regenerateLocalSpace(vehicle_t *data, int agendID, float3 newVelocity)
{
    float speed = data[agendID].speed;
    if (speed > 0)
    {
        float3 forwardVector = make_float3(newVelocity.x / speed, newVelocity.y / speed, newVelocity.z / speed);
        data[agendID].forward = forwardVector;
        float3 side = float3cross(data[agendID].forward, data[agendID].up);
        data[agendID].side = float3Normalize(side);
    }
}
                


#endif // _POLONAISE_CUDA_KERNEL_H_
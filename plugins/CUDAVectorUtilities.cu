#ifndef _CUDA_VECTOR_UTILITIES_CU_
#define _CUDA_VECTOR_UTILITIES_CU_

#include "CUDAFloatUtilities.cu"

__device__ float3
float3Interpolate(float alpha, float3 x0, float3 x1);

__device__ float
float3Length(float3 value);

__device__ float
float3LengthSquared(float3 value);

__device__ float3
limitMaxDeviationAngle(float3 source, float cosineOfConeAngle, float3 basis);

__device__ float
float3Dot (float3 v1, float3 v2);

__device__ float3
float3ParallelComponent(float3 source, float3 unitBasis);

__device__ float3
float3PerpendicularComponent(float3 source, float3 unitBasis);

__device__ float3
float3Normalize(float3 value);

__device__ float3
float3TruncateLength(float3 vec, float maxLength);

__device__ float3
float3BlendIn(float smoothRate, float3 newvalue, float3 smoothedAccu);

__device__ float3
float3Cross(float3 a, float3 b);

__device__ float3
float3Div(float3 vec, float div);

__device__ float3
float3Sub(float3 a, float3 b);

__device__ float3
float3Add(float3 a, float3 b);

__device__ float3
float3Mul(float3 vec, float mul);

__device__ float3
float3PredictFuturePosition(float3 position, float3 velocity, float predictionTime);

__device__ float
float3Distance(float3 a, float3 b);

__device__ float3
float3Interpolate(float alpha, float3 x0, float3 x1) {
    return make_float3(x0.x + ((x1.x - x0.x) * alpha),
                       x0.y + ((x1.y - x0.y) * alpha),
                       x0.z + ((x1.z - x0.z) * alpha));
}

__device__ float3
limitMaxDeviationAngle(float3 source, float cosineOfConeAngle, float3 basis)
{
    float sourcelength = float3Length(source);
    if (sourcelength == 0) return source;
    
    float3 direction = make_float3(source.x / sourcelength, source.y / sourcelength, source.z / sourcelength);
    float cosineOfSourceAngle = float3Dot(direction, basis);
    
    if (cosineOfSourceAngle >= cosineOfConeAngle) return source;
    
    float3 perp = float3PerpendicularComponent(source, basis);
    float3 unitPerp = float3Normalize(perp);
    
    float perpDist = sqrtf(1 - (cosineOfConeAngle * cosineOfConeAngle));
    float3 c0 = make_float3(basis.x * cosineOfConeAngle, basis.y * cosineOfConeAngle, basis.z * cosineOfConeAngle);
    float3 c1 = make_float3(unitPerp.x * perpDist, unitPerp.y * perpDist, unitPerp.z * perpDist);
    
    return make_float3((c0.x + c1.x) * sourcelength, (c0.y + c1.y) * sourcelength, (c0.z + c1.z) * sourcelength); 
}

__device__ float
float3Length(float3 value)
{
    return sqrtf(float3LengthSquared(value));
}

__device__ float
float3LengthSquared(float3 value)
{
    return float3Dot(value, value);
}

__device__ float
float3Dot (float3 v1, float3 v2)
{
    return (v1.x * v2.x) + (v1.y * v2.y) + (v1.z * v2.z);
}

__device__ float3
float3Cross(float3 a, float3 b)
{
    return make_float3((a.y * b.y) - (a.z * b.y),
                       (a.z * b.x) - (a.x * b.z),
                       (a.x * b.y) - (a.y * b.x));
}

__device__ float3
float3ParallelComponent(float3 source, float3 unitBasis)
{
    float projection = float3Dot(source, unitBasis);
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
    float length = float3Length(value);
    return (length>0) ? make_float3(value.x / length, value.y / length, value.z / length) : value;
}

__device__ float3
float3TruncateLength(float3 vec, float maxLength)
{
    float lengthSquared = maxLength * maxLength;
    float vecLengthSquared = float3LengthSquared(vec);
    if (vecLengthSquared <= lengthSquared)
        return vec;
    else {
        float vecLength = sqrtf(vecLengthSquared);
        float ratio = maxLength / vecLength;
        
        return make_float3(vec.x * ratio, vec.y * ratio, vec.z * ratio);
    }
}

__device__ float3
float3BlendIn(float smoothRate, float3 newvalue, float3 smoothedAccu)
{
    return float3Interpolate(clip(smoothRate, 0, 1), smoothedAccu, newvalue);
}

__device__ float3
float3Div(float3 vec, float div)
{
    vec.x /= div;
    vec.y /= div;
    vec.z /= div;
    
    return vec;
}

__device__ float3
float3Sub(float3 a, float3 b)
{
    a.x -= b.x;
    a.y -= b.y;
    a.z -= b.z;
    
    return a;
}

__device__ float3
float3Add(float3 a, float3 b)
{
    a.x += b.x;
    a.y += b.y;
    a.z += b.z;
    
    return a;
}

__device__ float3
float3Mul(float3 vec, float mul)
{
    vec.x *= mul;
    vec.y *= mul;
    vec.z *= mul;
    
    return vec;
}

__device__ float3
float3PredictFuturePosition(float3 position, float3 velocity, float predictionTime)
{
    return float3Add(position, float3Mul(velocity, predictionTime));
}

__device__ float
float3Distance(float3 a, float3 b)
{
    a = float3Sub(a, b);
    return float3Length(a);
}

#endif // _CUDA_VECTOR_UTILITIES_CU_
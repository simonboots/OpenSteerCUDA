#ifndef _VEHICE_T_H_
#define _VEHICE_T_H_

#include <cuda_runtime.h>

#if defined(NVCC)
typedef struct __align__(16) vehicle {
#else
typedef struct vehicle {
#endif
    
    float3 position;
    float3 new_position;
    float3 velocity;
    float3 follow_velocity;
    
    float3 smoothedAcceleration;
    float3 forward;
    float3 side;
    float3 up;
    
    float speed;
    float maxSpeed;
    float maxForce;
    float mass;
    
} vehicle_t;

#endif // _VEHICE_T_H_
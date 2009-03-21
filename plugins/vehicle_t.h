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
    
    float speed;
    
} vehicle_t;

#endif // _VEHICE_T_H_
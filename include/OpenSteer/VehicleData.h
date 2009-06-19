#ifndef _VEHICLE_DATA_H_
#define _VEHICLE_DATA_H_

#include <cuda_runtime.h>

#define MAX_VEHICLE 8192

typedef struct vehicle_data {
    // LocalSpace
    float3  side[MAX_VEHICLE];
    float3  up[MAX_VEHICLE];
    float3  forward[MAX_VEHICLE];
    float3  position[MAX_VEHICLE];
    
    // SimpleVehicle
    float   speed[MAX_VEHICLE];
    float3  smoothedAcceleration[MAX_VEHICLE];
} VehicleData;

typedef struct vehicle_const {
    float   maxForce[MAX_VEHICLE];
    float   maxSpeed[MAX_VEHICLE];
    float   mass[MAX_VEHICLE];
    float   radius[MAX_VEHICLE];
} VehicleConst;

#endif // _VEHICLE_DATA_H_
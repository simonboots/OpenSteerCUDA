#ifndef _VEHICLE_DATA_H_
#define _VEHICLE_DATA_H_

#include <cuda_runtime.h>

#define MAX_VEHICLE 512


typedef struct vehicle_data {
    // LocalSpace
    float3  side[MAX_VEHICLE];
    float3  up[MAX_VEHICLE];
    float3  forward[MAX_VEHICLE];
    float3  position[MAX_VEHICLE];
    
    // SimpleVehicle
    float   mass[MAX_VEHICLE];
    float   radius[MAX_VEHICLE];
    float   speed[MAX_VEHICLE];
    float   maxForce[MAX_VEHICLE];
    float   maxSpeed[MAX_VEHICLE];
    float   curvature[MAX_VEHICLE];
    float3  lastForward[MAX_VEHICLE];
    float3  lastPosition[MAX_VEHICLE];
    float   smoothedCurvature[MAX_VEHICLE];
    float3  smoothedAcceleration[MAX_VEHICLE];
    float3  smoothedPosition[MAX_VEHICLE];
} VehicleData;

#endif // _VEHICLE_DATA_H_
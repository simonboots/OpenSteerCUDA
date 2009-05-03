#include "AbstractCUDAKernel.h"
#include "OpenSteer/VehicleData.h"
#include "CUDAPlugIn.h"

int OpenSteer::AbstractCUDAKernel::getNumberOfAgents(void) {
    if (cudaplugin != NULL) {
        return cudaplugin->getNumberOfAgents();
    }
    return 0;
}

VehicleData* OpenSteer::AbstractCUDAKernel::getVehicleData(void) {
    if (cudaplugin != NULL) {
        return cudaplugin->getVehicleData();
    }
    return NULL;
}

VehicleConst* OpenSteer::AbstractCUDAKernel::getVehicleConst(void) {
    if (cudaplugin != NULL) {
        return cudaplugin->getVehicleConst();
    }
    return NULL;
}

float3* OpenSteer::AbstractCUDAKernel::getSteeringVectors(void) {
    if (cudaplugin != NULL) {
        return cudaplugin->getSteeringVectors();
    }
    return NULL;
}

float OpenSteer::AbstractCUDAKernel::getElapsedTime(void) {
    if (cudaplugin != NULL) {
        return cudaplugin->getElapsedTime();
    }
    return 0.f;
}
#include "OpenSteer/MemoryBackend.h"
#include <iostream>
#include <exception>

OpenSteer::MemoryBackend* OpenSteer::MemoryBackend::_instance = 0;
int OpenSteer::MemoryBackend::_idCounter = 0;

OpenSteer::MemoryBackend::MemoryBackend() {
    _data = new OpenSteer::VehicleData;
    std::cout << "MemoryBackend initialized" << std::endl;
}

OpenSteer::MemoryBackend::~MemoryBackend() {
    delete _data;
    _idCounter = 0;
}

int OpenSteer::MemoryBackend::getNextID(void) {
    if ((_idCounter + 1) >= MAX_VEHICLE) {
        throw std::exception();
    }
    return _idCounter++;
}


OpenSteer::Vec3 OpenSteer::MemoryBackend::side(int i) const {
    float3 v = _data->side[i];
    return Vec3(v.x, v.y, v.z);
}

void OpenSteer::MemoryBackend::setSide(int i, Vec3& v3) {
    float3 v = make_float3(v3.x, v3.y, v3.z);
    _data->side[i] = v;
}

OpenSteer::Vec3 OpenSteer::MemoryBackend::up(int i) const {
    float3 v = _data->up[i];
    return Vec3(v.x, v.y, v.z);
}

void OpenSteer::MemoryBackend::setUp(int i, Vec3& v3) {
    float3 v = make_float3(v3.x, v3.y, v3.z);
    _data->up[i] = v;
}

OpenSteer::Vec3 OpenSteer::MemoryBackend::forward(int i) const {
    float3 v = _data->forward[i];
    return Vec3(v.x, v.y, v.z);    
}

void OpenSteer::MemoryBackend::setForward(int i, Vec3& v3) {
    float3 v = make_float3(v3.x, v3.y, v3.z);
    _data->forward[i] = v;    
}

OpenSteer::Vec3 OpenSteer::MemoryBackend::position(int i) const {
    float3 v = _data->position[i];
    return Vec3(v.x, v.y, v.z);    
}

void OpenSteer::MemoryBackend::setPosition(int i, Vec3& v3) {
    float3 v = make_float3(v3.x, v3.y, v3.z);
    _data->position[i] = v;        
}

float OpenSteer::MemoryBackend::mass(int i) const {
    return _data->mass[i];
}

void OpenSteer::MemoryBackend::setMass(int i, float v) {
    _data->mass[i] = v;
}

float OpenSteer::MemoryBackend::radius(int i) const {
    return _data->radius[i];
}

void OpenSteer::MemoryBackend::setRadius(int i, float v) {
    _data->radius[i] = v;
}

float OpenSteer::MemoryBackend::speed(int i) const {
    return _data->speed[i];
}

void OpenSteer::MemoryBackend::setSpeed(int i, float v) {
    _data->speed[i] = v;
}

float OpenSteer::MemoryBackend::maxForce(int i) const {
    return _data->maxForce[i];
}

void OpenSteer::MemoryBackend::setMaxForce(int i, float v) {
    _data->maxForce[i] = v;
}

float OpenSteer::MemoryBackend::maxSpeed(int i) const {
    return _data->maxSpeed[i];
}

void OpenSteer::MemoryBackend::setMaxSpeed(int i, float v) {
    _data->maxSpeed[i] = v;
}

float OpenSteer::MemoryBackend::curvature(int i) const {
    return _data->curvature[i];
}

void OpenSteer::MemoryBackend::setCurvature(int i, float v) {
    _data->curvature[i] = v;
}

OpenSteer::Vec3 OpenSteer::MemoryBackend::lastForward(int i) const {
    float3 v = _data->lastForward[i];
    return Vec3(v.x, v.y, v.z);    
}

void OpenSteer::MemoryBackend::setLastForward(int i, Vec3& v3) {
    float3 v = make_float3(v3.x, v3.y, v3.z);
    _data->lastForward[i] = v;
}

OpenSteer::Vec3 OpenSteer::MemoryBackend::lastPosition(int i) const {
    float3 v = _data->lastPosition[i];
    return Vec3(v.x, v.y, v.z);
}

void OpenSteer::MemoryBackend::setLastPosition(int i, Vec3& v3) {
    float3 v = make_float3(v3.x, v3.y, v3.z);
    _data->lastPosition[i] = v;    
}

float OpenSteer::MemoryBackend::smoothedCurvature(int i) const {
    return _data->smoothedCurvature[i];
}

void OpenSteer::MemoryBackend::setSmoothedCurvature(int i, float v) {
    _data->smoothedCurvature[i] = v;
}

OpenSteer::Vec3 OpenSteer::MemoryBackend::smoothedAcceleration(int i) const {
    float3 v = _data->smoothedAcceleration[i];
    return Vec3(v.x, v.y, v.z);
}

void OpenSteer::MemoryBackend::setSmoothedAcceleration(int i, Vec3& v3) {
    float3 v = make_float3(v3.x, v3.y, v3.z);
    _data->smoothedAcceleration[i] = v;
}

OpenSteer::Vec3 OpenSteer::MemoryBackend::smoothedPosition(int i) const {
    float3 v = _data->smoothedPosition[i];
    return Vec3(v.x, v.y, v.z);
}

void OpenSteer::MemoryBackend::setSmoothedPosition(int i, Vec3& v3) {
    float3 v = make_float3(v3.x, v3.y, v3.z);
    _data->smoothedPosition[i] = v;
}
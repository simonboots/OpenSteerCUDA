#include "OpenSteer/MemoryBackend.h"
#include <iostream>
#include <exception>

OpenSteer::MemoryBackend* OpenSteer::MemoryBackend::_instance = 0;
int OpenSteer::MemoryBackend::_idCounter = 0;

OpenSteer::MemoryBackend::MemoryBackend() {
    _data = new VehicleData;
    std::cout << "MemoryBackend initialized" << std::endl;
}

OpenSteer::MemoryBackend::~MemoryBackend() {
    delete _data;
    _idCounter = 0;
}

int OpenSteer::MemoryBackend::getNextID(void) {
    if ((_idCounter + 1) > MAX_VEHICLE) {
        throw std::exception();
    }
    std::cout << "new ID: " << _idCounter << std::endl;
    return _idCounter++;
}

VehicleData* OpenSteer::MemoryBackend::getVehicleData(void) {
    return _data;
}

void OpenSteer::MemoryBackend::setVehicleData(VehicleData* vd) {
    _data = vd;
}


OpenSteer::Vec3 OpenSteer::MemoryBackend::side(int i) const {
    float3 v = _data->side[i];
    return Vec3(v.x, v.y, v.z);
}

OpenSteer::Vec3 OpenSteer::MemoryBackend::setSide(int i, const Vec3& v3) {
    float3 v = make_float3(v3.x, v3.y, v3.z);
    _data->side[i] = v;
    return v3;
}

OpenSteer::Vec3 OpenSteer::MemoryBackend::setSide(int i, float x, float y, float z) {
    float3 v = make_float3(x, y, z);
    _data->side[i] = v;
    return Vec3(x, y, z);
}

OpenSteer::Vec3 OpenSteer::MemoryBackend::up(int i) const {
    float3 v = _data->up[i];
    return Vec3(v.x, v.y, v.z);
}

OpenSteer::Vec3 OpenSteer::MemoryBackend::setUp(int i, const Vec3& v3) {
    float3 v = make_float3(v3.x, v3.y, v3.z);
    _data->up[i] = v;
    return v3;
}

OpenSteer::Vec3 OpenSteer::MemoryBackend::setUp(int i, float x, float y, float z) {
    float3 v = make_float3(x, y, z);
    _data->up[i] = v;
    return Vec3(x, y, z);
}

OpenSteer::Vec3 OpenSteer::MemoryBackend::forward(int i) const {
    float3 v = _data->forward[i];
    return Vec3(v.x, v.y, v.z);    
}

OpenSteer::Vec3 OpenSteer::MemoryBackend::setForward(int i, const Vec3& v3) {
    float3 v = make_float3(v3.x, v3.y, v3.z);
    _data->forward[i] = v;    
    return v3;
}

OpenSteer::Vec3 OpenSteer::MemoryBackend::setForward(int i, float x, float y, float z) {
    float3 v = make_float3(x, y, z);
    _data->forward[i] = v;    
    return Vec3(x, y, z);
}

OpenSteer::Vec3 OpenSteer::MemoryBackend::position(int i) const {
    float3 v = _data->position[i];
    return Vec3(v.x, v.y, v.z);    
}

OpenSteer::Vec3 OpenSteer::MemoryBackend::setPosition(int i, const Vec3& v3) {
    float3 v = make_float3(v3.x, v3.y, v3.z);
    _data->position[i] = v;        
    return v3;
}

OpenSteer::Vec3 OpenSteer::MemoryBackend::setPosition(int i, float x, float y, float z) {
    float3 v = make_float3(x, y, z);
    _data->position[i] = v;        
    return Vec3(x, y, z);
}

float OpenSteer::MemoryBackend::mass(int i) const {
    return _data->mass[i];
}

float OpenSteer::MemoryBackend::setMass(int i, float v) {
    _data->mass[i] = v;
    return v;
}

float OpenSteer::MemoryBackend::radius(int i) const {
    return _data->radius[i];
}

float OpenSteer::MemoryBackend::setRadius(int i, float v) {
    _data->radius[i] = v;
    return v;
}

float OpenSteer::MemoryBackend::speed(int i) const {
    return _data->speed[i];
}

float OpenSteer::MemoryBackend::setSpeed(int i, float v) {
    _data->speed[i] = v;
    return v;
}

float OpenSteer::MemoryBackend::maxForce(int i) const {
    return _data->maxForce[i];
}

float OpenSteer::MemoryBackend::setMaxForce(int i, float v) {
    _data->maxForce[i] = v;
    return v;
}

float OpenSteer::MemoryBackend::maxSpeed(int i) const {
    return _data->maxSpeed[i];
}

float OpenSteer::MemoryBackend::setMaxSpeed(int i, float v) {
    _data->maxSpeed[i] = v;
    return v;
}

float OpenSteer::MemoryBackend::curvature(int i) const {
    return _data->curvature[i];
}

float OpenSteer::MemoryBackend::setCurvature(int i, float v) {
    _data->curvature[i] = v;
    return v;
}

OpenSteer::Vec3 OpenSteer::MemoryBackend::lastForward(int i) const {
    float3 v = _data->lastForward[i];
    return Vec3(v.x, v.y, v.z);    
}

OpenSteer::Vec3 OpenSteer::MemoryBackend::setLastForward(int i, const Vec3& v3) {
    float3 v = make_float3(v3.x, v3.y, v3.z);
    _data->lastForward[i] = v;
    return v3;
}

OpenSteer::Vec3 OpenSteer::MemoryBackend::setLastForward(int i, float x, float y, float z) {
    float3 v = make_float3(x, y, z);
    _data->lastForward[i] = v;
    return Vec3(x, y, z);
}

OpenSteer::Vec3 OpenSteer::MemoryBackend::lastPosition(int i) const {
    float3 v = _data->lastPosition[i];
    return Vec3(v.x, v.y, v.z);
}

OpenSteer::Vec3 OpenSteer::MemoryBackend::setLastPosition(int i, const Vec3& v3) {
    float3 v = make_float3(v3.x, v3.y, v3.z);
    _data->lastPosition[i] = v;    
    return v3;
}

OpenSteer::Vec3 OpenSteer::MemoryBackend::setLastPosition(int i, float x, float y, float z) {
    float3 v = make_float3(x, y, z);
    _data->lastPosition[i] = v;    
    return Vec3(x, y, z);
}

float OpenSteer::MemoryBackend::smoothedCurvature(int i) const {
    return _data->smoothedCurvature[i];
}

float OpenSteer::MemoryBackend::setSmoothedCurvature(int i, float v) {
    _data->smoothedCurvature[i] = v;
    return v;
}

OpenSteer::Vec3 OpenSteer::MemoryBackend::smoothedAcceleration(int i) const {
    float3 v = _data->smoothedAcceleration[i];
    return Vec3(v.x, v.y, v.z);
}

OpenSteer::Vec3 OpenSteer::MemoryBackend::setSmoothedAcceleration(int i, const Vec3& v3) {
    float3 v = make_float3(v3.x, v3.y, v3.z);
    _data->smoothedAcceleration[i] = v;
    return v3;
}

OpenSteer::Vec3 OpenSteer::MemoryBackend::setSmoothedAcceleration(int i, float x, float y, float z) {
    float3 v = make_float3(x, y, z);
    _data->smoothedAcceleration[i] = v;
    return Vec3(x, y, z);
}

OpenSteer::Vec3 OpenSteer::MemoryBackend::smoothedPosition(int i) const {
    float3 v = _data->smoothedPosition[i];
    return Vec3(v.x, v.y, v.z);
}

OpenSteer::Vec3 OpenSteer::MemoryBackend::setSmoothedPosition(int i, const Vec3& v3) {
    float3 v = make_float3(v3.x, v3.y, v3.z);
    _data->smoothedPosition[i] = v;
    return v3;
}

OpenSteer::Vec3 OpenSteer::MemoryBackend::setSmoothedPosition(int i, float x, float y, float z) {
    float3 v = make_float3(x, y, z);
    _data->smoothedPosition[i] = v;
    return Vec3(x, y, z);
}
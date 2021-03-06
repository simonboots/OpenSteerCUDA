#ifndef _MEMORY_BACKEND_H_
#define _MEMORY_BACKEND_H_

#include "VehicleData.h"
#include "OpenSteer/Vec3.h"

namespace OpenSteer {
    
    class MemoryBackend {
    public:
        // Singleton
        static MemoryBackend* instance() {
            if (_instance == 0) {
                _instance = new OpenSteer::MemoryBackend();
            }
            
            return _instance;
        }
        
        static void reset() {
            delete _instance;
            _instance = 0;
            _idCounter = 0;
        }
        
        Vec3 side(int) const;
        Vec3 setSide(int, const Vec3&);
        Vec3 setSide(int, float, float, float);
        Vec3 up(int) const;
        Vec3 setUp(int, const Vec3&);
        Vec3 setUp(int, float, float, float);
        Vec3 forward(int) const;
        Vec3 setForward(int, const Vec3&);
        Vec3 setForward(int, float, float, float);
        Vec3 position(int) const;
        Vec3 setPosition(int, const Vec3&);
        Vec3 setPosition(int, float, float, float);
        
        float mass(int) const;
        float setMass(int, float);
        float radius(int) const;
        float setRadius(int, float);
        float speed(int) const;
        float setSpeed(int, float);
        float maxForce(int) const;
        float setMaxForce(int, float);
        float maxSpeed(int) const;
        float setMaxSpeed(int, float);
        Vec3 smoothedAcceleration(int) const;
        Vec3 setSmoothedAcceleration(int, const Vec3&);
        Vec3 setSmoothedAcceleration(int, float, float, float);
        
        int getNextID(void);
        VehicleData* getVehicleData(void);
        void setVehicleData(VehicleData*);
        VehicleConst* getVehicleConst(void);
        void setVehicleConst(VehicleConst*);
        
        ~MemoryBackend();
        
    protected:
        MemoryBackend();
    private:
        static MemoryBackend* _instance;
        VehicleData *_data;
        VehicleConst *_const;
        static int _idCounter;
    };
    
    
} // namespace OpenSteer

#endif // _MEMORY_BACKEND_H_
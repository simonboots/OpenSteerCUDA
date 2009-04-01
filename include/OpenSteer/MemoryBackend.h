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
        float curvature(int) const;
        float setCurvature(int, float);
        Vec3 lastForward(int) const;
        Vec3 setLastForward(int, const Vec3&);
        Vec3 setLastForward(int, float, float, float);
        Vec3 lastPosition(int) const;
        Vec3 setLastPosition(int, const Vec3&);
        Vec3 setLastPosition(int, float, float, float);
        float smoothedCurvature(int) const;
        float setSmoothedCurvature(int, float);
        Vec3 smoothedAcceleration(int) const;
        Vec3 setSmoothedAcceleration(int, const Vec3&);
        Vec3 setSmoothedAcceleration(int, float, float, float);
        Vec3 smoothedPosition(int) const;
        Vec3 setSmoothedPosition(int, const Vec3&);
        Vec3 setSmoothedPosition(int, float, float, float);
        
        int getNextID(void);
        
        ~MemoryBackend();
        
    protected:
        MemoryBackend();
    private:
        static MemoryBackend* _instance;
        VehicleData *_data;
        static int _idCounter;
    };
    
    
} // namespace OpenSteer

#endif // _MEMORY_BACKEND_H_
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
        void setSide(int, Vec3&);
        Vec3 up(int) const;
        void setUp(int, Vec3&);
        Vec3 forward(int) const;
        void setForward(int, Vec3&);
        Vec3 position(int) const;
        void setPosition(int, Vec3&);
        
        float mass(int) const;
        void setMass(int, float);
        float radius(int) const;
        void setRadius(int, float);
        float speed(int) const;
        void setSpeed(int, float);
        float maxForce(int) const;
        void setMaxForce(int, float);
        float maxSpeed(int) const;
        void setMaxSpeed(int, float);
        float curvature(int) const;
        void setCurvature(int, float);
        Vec3 lastForward(int) const;
        void setLastForward(int, Vec3&);
        Vec3 lastPosition(int) const;
        void setLastPosition(int, Vec3&);
        float smoothedCurvature(int) const;
        void setSmoothedCurvature(int, float);
        Vec3 smoothedAcceleration(int) const;
        void setSmoothedAcceleration(int, Vec3&);
        Vec3 smoothedPosition(int) const;
        void setSmoothedPosition(int, Vec3&);
        
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
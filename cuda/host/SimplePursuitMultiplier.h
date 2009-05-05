#ifndef _SIMPLE_PURSUIT_MULTIPLIER_H_
#define _SIMPLE_PURSUIT_MULTIPLIER_H_

#include "OpenSteer/AbstractCUDAKernel.h"
#include "OpenSteer/PursuitDataProvider.h"

namespace OpenSteer {
    
    class SimplePursuitMultiplier : public AbstractCUDAKernel, public PursuitDataProvider
    {
    public:
        SimplePursuitMultiplier();
        ~SimplePursuitMultiplier(void);
        
        void init(void);
        void run(void);
        void close(void);
        
        void setNewPosition(float3);
        void setNewVelocity(float3);
        
    private:
        float3 newPosition;
        float3 newVelocity;
        unsigned int mem_size_velocity;
        unsigned int mem_size_position;
        bool change;
    };
    
} // namespace

#endif // _SIMPLE_PURSUIT_MULTIPLIER_H_
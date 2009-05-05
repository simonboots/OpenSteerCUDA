#ifndef _PURSUIT_DATA_PROVIDER_H_
#define _PURSUIT_DATA_PROVIDER_H_

namespace OpenSteer {
    class PursuitDataProvider {
    public:
        virtual float3 *getPursuitVelocity(void) {
            return d_pursuitVelocity;
        }
        
        virtual float3 *getPursuitPosition(void) {
            return d_pursuitPosition;
        }
        
    protected:
        float3* d_pursuitVelocity;
        float3* d_pursuitPosition;
    };
} // namespace

#endif // _PURSUIT_DATA_PROVIDER_H_
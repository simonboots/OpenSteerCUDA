#ifndef _TARGET_SPEED_PROVIDER_H_
#define _TARGET_SPEED_PROVIDER_H_

namespace OpenSteer {
    class TargetSpeedProvider {
    public:
        virtual float *getTargetSpeeds(void) {
            return d_targetSpeeds;
        }
    protected:
        float* d_targetSpeeds;
    };
} // namespace

#endif // _TARGET_SPEED_PROVIDER_H_
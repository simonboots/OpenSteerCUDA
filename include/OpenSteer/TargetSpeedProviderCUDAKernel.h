#ifndef _TARGET_SPEED_PROVIDER_CUDA_KERNEL_H_
#define _TARGET_SPEED_PROVIDER_CUDA_KERNEL_H_

namespace OpenSteer {
    class TargetSpeedProviderCUDAKernel {
    public:
        virtual float *getTargetSpeeds(void) {
            return d_targetSpeeds;
        }
    protected:
        float* d_targetSpeeds;
    };
} // namespace


#endif // _TARGET_SPEED_PROVIDER_CUDA_KERNEL_H_
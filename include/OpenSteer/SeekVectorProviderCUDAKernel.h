#ifndef _SEEK_VECTOR_PROVIDER_CUDA_KERNEL_H_
#define _SEEK_VECTOR_PROVIDER_CUDA_KERNEL_H_

namespace OpenSteer {
    class SeekVectorProviderCUDAKernel {
        virtual float3 *getSeekVectors(void) = 0;
    };
} // namespace


#endif // _SEEK_VECTOR_PROVIDER_CUDA_KERNEL_H_
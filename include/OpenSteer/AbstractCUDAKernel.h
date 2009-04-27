#ifndef _ABSTRACT_CUDA_KERNEL_H_
#define _ABSTRACT_CUDA_KERNEL_H_

namespace OpenSteer
{
    class AbstractCUDAKernel
        {
        public:
            AbstractCUDAKernel();
            ~AbstractCUDAKernel();
            virtual void init() = 0;
            virtual void run() = 0;
            virtual void reset() = 0;
        };

} // namespace
    
#endif // _ABSTRACT_CUDA_KERNEL_H_

#ifndef _ABSTRACT_CUDA_KERNEL_H_
#define _ABSTRACT_CUDA_KERNEL_H_

#include <cuda_runtime.h>
#include "OpenSteer/VehicleData.h"

namespace OpenSteer
{
    class CUDAPlugIn;

    class AbstractCUDAKernel
        {
        public:
            virtual void init(void) = 0;
            virtual void run(void) = 0;
            virtual void close(void) = 0;
            
            virtual void reset(void) {
                close();
                init();
            }

            
            virtual dim3 gridDim(void) {
                return dim3(getNumberOfAgents()/threadsPerBlock);
                        
            }
            
            virtual dim3 blockDim(void) {
                return dim3(threadsPerBlock);
            }
            
            int getNumberOfAgents(void);
            VehicleData* getVehicleData(void);
            VehicleConst* getVehicleConst(void);
            float3* getSteeringVectors(void);
            
            void setPlugIn(CUDAPlugIn* plugin) {
                this->cudaplugin = plugin;
            }
            
        protected:
            CUDAPlugIn *cudaplugin;
            int threadsPerBlock;
        };

} // namespace
    
#endif // _ABSTRACT_CUDA_KERNEL_H_

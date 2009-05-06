#ifndef _CUDA_PLUGIN_H_
#define _CUDA_PLUGIN_H_

#include "OpenSteer/AbstractCUDAKernel.h"
#include "OpenSteer/MemoryBackend.h"
#include "OpenSteer/PlugIn.h"
#include "VehicleData.h"
#include <vector>

namespace OpenSteer
{
    class CUDAPlugIn : public PlugIn
        {
        public:
            CUDAPlugIn(void);
            ~CUDAPlugIn(void);
            void close(void);
            bool addKernel(AbstractCUDAKernel *);
            bool removeKernel(AbstractCUDAKernel *);
            void initKernels(void);
            void closeKernels(void);
            virtual void update(const float currentTime, const float elapsedTime);
            void setNumberOfAgents(int);
            int getNumberOfAgents(void);
            float3* getSteeringVectors(void);
            VehicleData* getVehicleData(void);
            VehicleConst* getVehicleConst(void);
            float getElapsedTime(void);
            MemoryBackend* getMemoryBackend(void);
            void recopyVehicleData(void);
            
        protected:
            void copyVehicleData(void);
            std::vector<AbstractCUDAKernel *> kernels;
            int numOfAgents;
            MemoryBackend *memoryBackend;
            float elapsedTime;
            //float3* h_steeringVectors;
            
            // device memory pointers
            float3 *d_steeringVectors;
            unsigned int mem_size_steering;
            VehicleData *d_vehicleData;
            unsigned int mem_size_vehicle_data;
            VehicleConst *d_vehicleConst;
            unsigned int mem_size_vehicle_const;
        };
    
} // namespace


#endif // _CUDA_PLUGIN_H_
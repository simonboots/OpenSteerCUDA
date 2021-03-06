#ifndef _MODIFY_KERNEL_CU_
#define _MODIFY_KERNEL_CU_

#include <cutil.h>
#include "OpenSteer/VehicleData.h"
#include "CUDAVectorUtilities.cu"
#include "CUDANeighborUtilities.cu"
#include "CUDAKernelOptions.cu"

#define CHECK_BANK_CONFLICTS 1
#if CHECK_BANK_CONFLICTS
#define FV_F(i) (CUT_BANK_CHECKER(((float*)follow_velocity), i))
#define V_F(i) (CUT_BANK_CHECKER(((float*)velocity), i))
#define SA_F(i) (CUT_BANK_CHECKER(((float*)smooth_acceleration), i))
#define FV(i) (CUT_BANK_CHECKER(follow_velocity, i))
#define V(i) (CUT_BANK_CHECKER(velocity, i))
#define SA(i) (CUT_BANK_CHECKER(smooth_acceleration, i))
#else
#define FV_F(i) ((float*)follow_velocity)[i]
#define V_F(i) ((float*)velocity)[i]
#define SA_F(i) ((float*)smooth_acceleration)[i]
#define FV(i) follow_velocity[i]
#define V(i) velocity[i]
#define SA(i) smooth_acceleration[i]
#endif

__global__ void
modifyKernel(VehicleData *vehicleData, VehicleConst *vehicleConst, float3 *steeringVectors, float elapsedTime, kernel_options options)
{
    int id = (blockIdx.x * blockDim.x + threadIdx.x);
    //int numOfAgents = gridDim.x * blockDim.x;
    int blockOffset = (blockDim.x * blockIdx.x * 3);
    
    // shared memory for follow_velocity
    __shared__ float3 follow_velocity[TPB];
    
    // shared memory for velocity vector
    __shared__ float3 velocity[TPB];
    
    // shared memory for smooth_acceleration
    __shared__ float3 smooth_acceleration[TPB];
    
    // coalesced global memory access
    float speed = (*vehicleData).speed[id];
    
    // velocity
    V_F(threadIdx.x) = ((float*)(*vehicleData).forward)[blockOffset + threadIdx.x];
    V_F(threadIdx.x + blockDim.x) = ((float*)(*vehicleData).forward)[blockOffset + threadIdx.x + blockDim.x];
    V_F(threadIdx.x + 2*blockDim.x) = ((float*)(*vehicleData).forward)[blockOffset + threadIdx.x + 2*blockDim.x];
    
    // follow_velocity
    FV_F(threadIdx.x) = ((float*)steeringVectors)[blockOffset + threadIdx.x];
    FV_F(threadIdx.x + blockDim.x) = ((float*)steeringVectors)[blockOffset + threadIdx.x + blockDim.x];
    FV_F(threadIdx.x + 2*blockDim.x) = ((float*)steeringVectors)[blockOffset + threadIdx.x + 2*blockDim.x];
    
    // smoothed_acceleration
    SA_F(threadIdx.x) = ((float*)(*vehicleData).smoothedAcceleration)[blockOffset + threadIdx.x];
    SA_F(threadIdx.x + blockDim.x) = ((float*)(*vehicleData).smoothedAcceleration)[blockOffset + threadIdx.x + blockDim.x];
    SA_F(threadIdx.x + 2*blockDim.x) = ((float*)(*vehicleData).smoothedAcceleration)[blockOffset + threadIdx.x + 2*blockDim.x];

    __syncthreads();
    
    // multiply forward vector with speed [3 FLOPS]
    V(threadIdx.x) = float3Mul(V(threadIdx.x), speed);
    
    __syncthreads();
    
    // adjustRawSteeringForce
    float3 v; // v = adjustedForce
    
    // [1 FLOPS]
    float maxAdjustedSpeed = 0.2f * (*vehicleConst).maxSpeed[id];
    
    // [3 FLOPS]
    if ((speed > maxAdjustedSpeed) || (FV(threadIdx.x).x == 0.f && FV(threadIdx.x).z == 0.f)) {
        v = FV(threadIdx.x);
    } else {
        // [3 FLOPS] / [2 FLOPS]
        float cosine = interpolate (__powf(speed / maxAdjustedSpeed, 20), 1.0f, -1.0f);
        // [52 FLOPS] / [3 FLOPS]
        v = limitMaxDeviationAngle(FV(threadIdx.x), cosine, float3Div(V(threadIdx.x), speed));
    }
    
    // [14 FLOPS]
    v = float3TruncateLength(v, (*vehicleConst).maxForce[id]); // v = clippedForce
    // [3 FLOPS]
    v = float3Div(v, (*vehicleConst).mass[id]); // v = new_acceleration
    
    // [1 FLOPS]
    if (elapsedTime > 0) {
        // [2 FLOPS] / [1 FLOPS]
        float smoothRate = clip(9 * elapsedTime, 0.15f, 0.4f);
        
        // [9 FLOPS]
        SA(threadIdx.x) = float3BlendIn(smoothRate, v, SA(threadIdx.x));
    }
    
    __syncthreads();
    
    // writing smoothed_acceleration back to global memory (coalesced)
    ((float*)(*vehicleData).smoothedAcceleration)[blockOffset + threadIdx.x] = SA_F(threadIdx.x);
    ((float*)(*vehicleData).smoothedAcceleration)[blockOffset + threadIdx.x + blockDim.x] = SA_F(threadIdx.x + blockDim.x);
    ((float*)(*vehicleData).smoothedAcceleration)[blockOffset + threadIdx.x + 2*blockDim.x] = SA_F(threadIdx.x + 2*blockDim.x);
    
    // [4 FLOPS]
    V(threadIdx.x).x += SA(threadIdx.x).x * elapsedTime;
    V(threadIdx.x).z += SA(threadIdx.x).z * elapsedTime;
    
    __syncthreads();
    
    //[14 FLOPS]
    V(threadIdx.x) = float3TruncateLength(V(threadIdx.x), (*vehicleConst).maxSpeed[id]);
    
    __syncthreads(); // position is re-written
    
    // ***********************************************
    // using follow_velocity to store position vector!
    // ***********************************************
    
    // loading position data from global memory (coalesced)
    FV_F(threadIdx.x) = ((float*)(*vehicleData).position)[blockOffset + threadIdx.x];
    FV_F(threadIdx.x + blockDim.x) = ((float*)(*vehicleData).position)[blockOffset + threadIdx.x + blockDim.x];
    FV_F(threadIdx.x + 2*blockDim.x) = ((float*)(*vehicleData).position)[blockOffset + threadIdx.x + 2*blockDim.x];
    
    __syncthreads();
    
    FV(threadIdx.x) = make_float3(FV(threadIdx.x).x + (V(threadIdx.x).x * elapsedTime),
                                  FV(threadIdx.x).y + (V(threadIdx.x).y * elapsedTime),
                                  FV(threadIdx.x).z + (V(threadIdx.x).z * elapsedTime));
    
    __syncthreads();
    
    // handle spherical wrap around
    if ((options & SPHERICAL_WRAP_AROUND) != 0) {
        // [20 FLOPS]
        FV(threadIdx.x) = sphericalWrapAround(FV(threadIdx.x), make_float3(0.f, 0.f, 0.f));
        __syncthreads();
    }

    
    // writing position data back to global memory (coalesced)
    ((float*)(*vehicleData).position)[blockOffset + threadIdx.x] = FV_F(threadIdx.x);
    ((float*)(*vehicleData).position)[blockOffset + threadIdx.x + blockDim.x] = FV_F(threadIdx.x + blockDim.x);
    ((float*)(*vehicleData).position)[blockOffset + threadIdx.x + 2*blockDim.x] = FV_F(threadIdx.x + 2*blockDim.x);
    
    __syncthreads();
        
    // copy speed back to global memory
    // [6 FLOPS]
    speed = float3Length(V(threadIdx.x));
    (*vehicleData).speed[id] = speed;
    
    // **********************************************
    // using velocity vector to store forward vector!
    // **********************************************
    // [6 FLOPS]
    V(threadIdx.x) = float3Div(V(threadIdx.x), speed);
    __syncthreads();
    
    // regenerate local space for banking
    if ((options & LOCAL_SPACE_BANKING) != 0) {
        // the length of this global-upward-pointing vector controls the vehicle's
        // tendency to right itself as it is rolled over from turning acceleration
        float3 globalUp = make_float3(0, 0.2f, 0);
        
        // acceleration points toward the center of local path curvature, the
        // length determines how much the vehicle will roll while turning
        float3 accelUp = float3Mul(SA(threadIdx.x), 0.05f);
        
        // combined banking, sum of UP due to turning and global UP
        float3 bankUp = float3Add(accelUp, globalUp);
        
        // ******************************************************
        // using smoothed_acceleration vector to store up vector!
        // ******************************************************
        
        // loading position data from global memory (coalesced)
        SA_F(threadIdx.x) = ((float*)(*vehicleData).up)[blockOffset + threadIdx.x];
        SA_F(threadIdx.x + blockDim.x) = ((float*)(*vehicleData).up)[blockOffset + threadIdx.x + blockDim.x];
        SA_F(threadIdx.x + 2*blockDim.x) = ((float*)(*vehicleData).up)[blockOffset + threadIdx.x + 2*blockDim.x];
        __syncthreads();
        
        // blend bankUp into vehicle's UP basis vector
        float smoothRate = elapsedTime * 3;
        SA(threadIdx.x) = float3Normalize(float3BlendIn(smoothRate, bankUp, SA(threadIdx.x)));
        __syncthreads();
        
    } else {
        
        // ******************************************************
        // using smoothed_acceleration vector to store up vector!
        // ******************************************************
        
        // loading position data from global memory (coalesced)
        SA_F(threadIdx.x) = ((float*)(*vehicleData).up)[blockOffset + threadIdx.x];
        SA_F(threadIdx.x + blockDim.x) = ((float*)(*vehicleData).up)[blockOffset + threadIdx.x + blockDim.x];
        SA_F(threadIdx.x + 2*blockDim.x) = ((float*)(*vehicleData).up)[blockOffset + threadIdx.x + 2*blockDim.x];
        __syncthreads();
    }
    
    // regenerate local space
    // ******************************************
    // using follow_velocity to store side vector
    // ******************************************
    
    // setUnitSideFromForwardAndUp()
    // [10 9 FLOPS]
    FV(threadIdx.x) = float3Normalize(float3Cross(V(threadIdx.x), SA(threadIdx.x)));
    __syncthreads();
    
    // new up vector
    // [9 FLOPS]
    SA(threadIdx.x) = float3Cross(FV(threadIdx.x), V(threadIdx.x));
    __syncthreads();

    // writing up vector back to global memory (coalesced)
    ((float*)(*vehicleData).up)[blockOffset + threadIdx.x] = SA_F(threadIdx.x);
    ((float*)(*vehicleData).up)[blockOffset + threadIdx.x + blockDim.x] = SA_F(threadIdx.x + blockDim.x);
    ((float*)(*vehicleData).up)[blockOffset + threadIdx.x + 2*blockDim.x] = SA_F(threadIdx.x + 2*blockDim.x);    
    
    // writing side vector back to global memory (coalesced)
    ((float*)(*vehicleData).side)[blockOffset + threadIdx.x] = FV_F(threadIdx.x);
    ((float*)(*vehicleData).side)[blockOffset + threadIdx.x + blockDim.x] = FV_F(threadIdx.x + blockDim.x);
    ((float*)(*vehicleData).side)[blockOffset + threadIdx.x + 2*blockDim.x] = FV_F(threadIdx.x + 2*blockDim.x);
    
    // writing forward vector back to global memory (coalesced)
    ((float*)(*vehicleData).forward)[blockOffset + threadIdx.x] = V_F(threadIdx.x);
    ((float*)(*vehicleData).forward)[blockOffset + threadIdx.x + blockDim.x] = V_F(threadIdx.x + blockDim.x);
    ((float*)(*vehicleData).forward)[blockOffset + threadIdx.x + 2*blockDim.x] = V_F(threadIdx.x + 2*blockDim.x);
    
}

#endif // _MODIFY_KERNEL_CU_
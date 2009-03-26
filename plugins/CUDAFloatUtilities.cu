#ifndef _CUDA_FLOAT_UTILITIES_H_
#define _CUDA_FLOAT_UTILITIES_H_

__device__ float
interpolate(float alpha, float x0, float x1);

__device__ float
clip(float x, float min, float max);

//__device__ float
//frandom01();

__device__ float
interpolate(float alpha, float x0, float x1)
{
    return x0 + ((x1 - x0) * alpha);
}

__device__ float
clip(float x, float min, float max)
{
    if (x < min) return min;
    if (x > max) return max;
    return x;
}

//__device__ float
//frandom01()
//{
//    return (((float) rand()) / ((float) RAND_MAX));
//}

#endif // _CUDA_FLOAT_UTILITIES_H_
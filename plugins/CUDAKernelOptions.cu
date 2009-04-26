#ifndef _CUDA_KERNEL_OPTIONS_CU_
#define _CUDA_KERNEL_OPTIONS_CU_

#define TPB 64
#define MAX_OBSTACLES 128

typedef enum kernel_opt
{
    NONE = 0,
    IGNORE_UNLESS_ZERO = 1 << 0,
    LOCAL_SPACE_BANKING = 1 << 1,
    SPHERICAL_WRAP_AROUND = 1 << 2
} kernel_options;

#endif // _CUDA_KERNEL_OPTIONS_CU_
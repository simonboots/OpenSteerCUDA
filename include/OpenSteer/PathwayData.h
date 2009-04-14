#ifndef _PATHWAY_DATA_H_
#define _PATHWAY_DATA_H_
#include <cuda_runtime.h>

#define MAX_PATHWAY_POINTS 20

typedef struct pathway_data {
    float3 points[MAX_PATHWAY_POINTS];
    int numElements;
    int isCyclic;
    float radius;
} PathwayData;

#endif // _PATHWAY_DATA_H_
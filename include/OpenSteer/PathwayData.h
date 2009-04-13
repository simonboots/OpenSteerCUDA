#ifndef _PATHWAY_DATA_H_
#define _PATHWAY_DATA_H_

#define MAX_PATHWAY_POINTS 20

typedef struct pathway_data {
    int numElements;
    int isCyclic;
    float radius;
    float3 points[MAX_PATHWAY_POINTS];
} PathwayData;

#endif // _PATHWAY_DATA_H_
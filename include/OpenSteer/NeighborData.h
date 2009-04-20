#ifndef _NEIGHBOR_DATA_H_
#define _NEIGHBOR_DATA_H_

#define MAX_NEIGHBORS 7

typedef struct neighbor_data {
    int numOfNeighbors;
    int idsOfNeighbors[MAX_NEIGHBORS];
} NeighborData;

#endif // _NEIGHBOR_DATA_H_
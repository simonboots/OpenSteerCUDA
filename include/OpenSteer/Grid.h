#ifndef _GRID_H_
#define _GRID_H_

#include <vector>
#include "OpenSteer/Vec3.h"

typedef std::vector<int> agents;
typedef agents::const_iterator agentsIterator;
typedef std::vector<agents*> cells;
typedef cells::const_iterator cellsIterator;

namespace OpenSteer {
    
class Grid {
public:
    static const unsigned int worldSize = 100;
    static const unsigned int cellSize = 5; // 100 / 5 = 20^3 cells
    
    Grid();
    ~Grid();
    static int index(Vec3 position);
    static int index(float x, float y, float z);
    void clear(void);
    int numOfAgents(void);
    int numOfCells(void);
    void save(Vec3 position, int id);
    void save(float x, float y, float z, int id);
    int *getIndices(void);
    int *getAgents(void);
    
private:
    cells allCells;
    int _numOfAgents;
    int _numOfCells;
    int lastNumOfIndices;
    int lastNumOfAgents;
    int* pAgents;
    int* pIndices;
};
    
} // namespace

#endif // _GRID_H_
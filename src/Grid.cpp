#include "OpenSteer/Grid.h"

OpenSteer::Grid::Grid()
{
    _numOfCells = (2 * Grid::worldSize / Grid::cellSize) * (2 * Grid::worldSize / Grid::cellSize) * (2 * Grid::worldSize / Grid::cellSize);
    
    for (int i = 0; i < _numOfCells; i++) {
        allCells.push_back(new agents);
    }
    _numOfAgents = 0;
    pAgents = NULL;
    pIndices = NULL;
}

OpenSteer::Grid::~Grid()
{
    allCells.clear();
}

int OpenSteer::Grid::index(Vec3 position)
{
    return index(position.x, position.y, position.z);
}

int OpenSteer::Grid::index(float _x, float _y, float _z)
{
    int cellsPerDimension = 2 * Grid::worldSize / Grid::cellSize;
    int x = (_x + Grid::worldSize) / Grid::cellSize;
    int y = (_y + Grid::worldSize) / Grid::cellSize;
    int z = (_z + Grid::worldSize) / Grid::cellSize;
    
    return x +
           y * cellsPerDimension +
           z * cellsPerDimension * cellsPerDimension;
}

void OpenSteer::Grid::clear(void)
{
    for (cellsIterator i = allCells.begin(); i != allCells.end(); i++) {
        (*i)->clear();
    }
    
    _numOfAgents = 0;
}

int OpenSteer::Grid::numOfAgents(void)
{
    return _numOfAgents;
}

int OpenSteer::Grid::numOfCells(void)
{
    return _numOfCells;
}

void OpenSteer::Grid::save(Vec3 position, int id)
{
    save(position.x, position.y, position.z, id);
}

void OpenSteer::Grid::save(float x, float y, float z, int id)
{
    int index = Grid::index(x, y, z);
    (allCells.at(index))->push_back(id);
    _numOfAgents++;
}

int* OpenSteer::Grid::getIndices(void)
{
    static int lastNumOfIndices = 0;
    
    if (lastNumOfIndices != _numOfCells) {
        if (pIndices != NULL)
            delete pIndices;
        
        lastNumOfIndices = _numOfCells;
        pIndices = new int[_numOfCells];
    }
    
    int sum = 0;
    int n = 0;
    for (cellsIterator i = allCells.begin(); i != allCells.end(); i++) {
        pIndices[n++] = sum;
        sum += (*i)->size();
    }
    
    return pIndices;
}

int* OpenSteer::Grid::getAgents(void)
{
    static int lastNumOfAgents = 0;
    
    if (lastNumOfAgents != _numOfAgents) {
        if (pAgents != NULL)
            delete pAgents;
        
        lastNumOfAgents = _numOfAgents;
        pAgents = new int[_numOfAgents];
    } 
    
    int n = 0;
    
    for (cellsIterator ci = allCells.begin(); ci != allCells.end(); ci++) {
        for (agentsIterator ai = (*ci)->begin(); ai != (*ci)->end(); ai++) {
            pAgents[n++] = *ai;
        }
    }
    
    return pAgents;
}
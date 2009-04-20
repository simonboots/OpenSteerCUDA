#include "OpenSteer/Grid.h"

OpenSteer::Grid::Grid()
{
    int numOfCells = (Grid::worldSize / Grid::cellSize) * (Grid::worldSize / Grid::cellSize) * (Grid::worldSize / Grid::cellSize);
    
    for (int i = 0; i < numOfCells; i++) {
        allCells.push_back(new agents);
    }
    numOfAgents = 0;
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
    int cellsPerDimension = Grid::worldSize / Grid::cellSize;
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
    
    numOfAgents = 0;
}

int OpenSteer::Grid::size(void)
{
    return numOfAgents;
}

void OpenSteer::Grid::save(Vec3 position, int id)
{
    save(position.x, position.y, position.z, id);
}

void OpenSteer::Grid::save(float x, float y, float z, int id)
{
    int index = Grid::index(x, y, z);
    (allCells.at(index))->push_back(id);
    numOfAgents++;
}

int* OpenSteer::Grid::getIndices(void)
{
    int numOfCells = (Grid::worldSize / Grid::cellSize) * (Grid::worldSize / Grid::cellSize) * (Grid::worldSize / Grid::cellSize);
    int *indices = new int[numOfCells];
    
    int sum = 0;
    int n = 0;
    for (cellsIterator i = allCells.begin(); i != allCells.end(); i++) {
        indices[n++] = sum;
        sum += (*i)->size();
    }
    
    return indices;
}

int* OpenSteer::Grid::getAgents(void)
{
    int *agents = new int[numOfAgents];
    
    int n = 0;
    
    for (cellsIterator ci = allCells.begin(); ci != allCells.end(); ci++) {
        for (agentsIterator ai = (*ci)->begin(); ai != (*ci)->end(); ai++) {
            agents[n++] = *ai;
        }
    }
}
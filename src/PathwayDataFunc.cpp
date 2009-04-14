#include "PathwayDataFunc.h"
#include "PathwayData.h"
#include "Pathway.h"
#include <iostream>

PathwayData* transformPathway(OpenSteer::PolylinePathway &polyline)
{
    PathwayData *pw = new PathwayData;
    
    pw->numElements = polyline.pointCount;
    pw->isCyclic = polyline.cyclic ? 1 : 0;
    pw->radius = (float)polyline.radius;
    
    if (polyline.pointCount > MAX_PATHWAY_POINTS) 
        std::cout << "WARNING: Polyline point count greater than MAX_PATHWAYPOINTS, truncating..." << std::endl;
    
    for (int i = 0; i < polyline.pointCount && i < MAX_PATHWAY_POINTS; i++) {
        (pw->points[i]).x = polyline.points[i].x;
        (pw->points[i]).y = polyline.points[i].y;
        (pw->points[i]).z = polyline.points[i].z;
    }
    
    return pw;
}

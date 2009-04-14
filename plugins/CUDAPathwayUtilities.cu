#ifndef _CUDA_PATHWAY_UTILITIES_H_
#define _CUDA_PATHWAY_UTILITIES_H_

#include "CUDAFloatUtilities.cu"
#include "CUDAVectorUtilities.cu"
#include <stdio.h>

// Given an arbitrary point ("A"), returns the nearest point ("P") on
// this path.  Also returns, via output arguments, the path tangent at
// P and a measure of how far A is outside the Pathway's "tube".  Note
// that a negative distance indicates A is inside the Pathway.
__device__ float3
mapPointToPath (float3 *points, int numOfPoints, float3 point, float3* tangent, float* outside);

// given a distance along the path, convert it to a point on the path
__device__ float3
mapPathDistanceToPoint(float3 *points, int numOfPoints, int isCyclic, float distance);

// Given an arbitrary point, convert it to a distance along the path.
__device__ float
mapPointToPathDistance(float3 *points, int numOfPoints, float3 point);

// is the given point inside the path tube?
__device__ int
isInsidePath(float3 *points, int numOfPoints, float pathRadius, float3 point);

// how far outside path tube is the given point?  (negative is inside)
__device__ float
howFarOutsidePath(float3 *points, int numOfPoints, float pathRadius, float3 point);

// compute minimum distance from a point to a line segment
__device__ float
pointToSegmentDistance(float3 point, float3 ep0, float3 ep1, float3 *nearestPointOnSegment, float *segmentProjection);

// assessor for total path length;
__device__ float
getTotalPathLength(float3 *points, int numOfPoints);

// get segment normal
__device__ float3
getSegmentNormal(float3 p0, float3 p1);

// ==============
// Implementation
// ==============

__device__ float3
mapPointToPath (float3 *points, int numOfPoints, float pathRadius, float3 point, float3* tangent, float* outside)
{
    float minDistance = MAXFLOAT;
    float d;
    int i;
    float3 nearestPoint;

    for (i = 1; i < numOfPoints; i++) {
        float3 nearestPointOnSegment;
        float segmentProjection;

        d = pointToSegmentDistance(point, points[i-1], points[i], &nearestPointOnSegment, &segmentProjection);
        if (d < minDistance) {
            minDistance = d;
            nearestPoint = nearestPointOnSegment;
        }
    }
    
    *outside = float3Distance(nearestPoint, point) - pathRadius;
    
    return nearestPoint;
}

__device__ float3
mapPathDistanceToPoint(float3 *points, int numOfPoints, int isCyclic, float distance)
{
    float remaining = distance;
    float totalPathLength = getTotalPathLength(points, numOfPoints);
    
    if (isCyclic == 1) {
        remaining = fmod(distance, totalPathLength);
    } else {
        if (distance < 0) return points[0];
        if (distance >= totalPathLength) return points[numOfPoints - 1];
    }
    
    // step through segments, subtracting off segment lengths until
    // locating the segment that contains the original pathDistance.
    // Interpolate along that segment to find 3d point value to return.
    
    float3 result;
    int i;
    
    for (i = 1; i < numOfPoints; i++) {
        float segmentLength = float3Length(float3Sub(points[i-1], points[i]));
        
        if (segmentLength < remaining) {
            remaining -= segmentLength;
        } else {
            float ratio = remaining / segmentLength;
            result = float3Interpolate(ratio, points[i-1], points[i]);
            break;
        }
    }
    return result;
}

__device__ float
mapPointToPathDistance(float3 *points, int numOfPoints, float3 point)
{
    float d;
    float minDistance = MAXFLOAT;
    float segmentLengthTotal = 0;
    float pathDistance = 0;
    int i;
    
    for (i = 1; i < numOfPoints; i++) {
        float segmentProjection;
        float3 nearestPointOnSegment;
        d = pointToSegmentDistance(point, points[i-1], points[i], &nearestPointOnSegment, &segmentProjection);
        
        if (d < minDistance) {
            minDistance = d;
            pathDistance = segmentLengthTotal + segmentProjection;
        }
        segmentLengthTotal += float3Length(float3Sub(points[i-1], points[i]));
    }
    
    return pathDistance;
}

__device__ int
isInsidePath(float3 *points, int numOfPoints, float pathRadius, float3 point)
{
    float outside;
    float3 tangent;
    mapPointToPath(points, numOfPoints, pathRadius, point, &tangent, &outside);
    if (outside < 0) return 1;
    return 0;
}

__device__ float
howFarOutsidePath(float3 *points, int numOfPoints, float pathRadius, float3 point)
{
    float outside;
    float3 tangent;
    mapPointToPath(points, numOfPoints, pathRadius, point, &tangent, &outside);
    return outside;
}

__device__ float
pointToSegmentDistance(float3 point, float3 ep0, float3 ep1, float3 *nearestPointOnSegment, float *segmentProjection)
{
    // convert the test point to be "local" to ep0
    float3 local = float3Sub(point, ep0);
    float segmentLength = float3Length(float3Sub(ep0, ep1));
    
    // find the projection of "local" onto "segmentNormal"
    float3 segmentNormal = getSegmentNormal(ep0, ep1);

    *segmentProjection = float3Dot(segmentNormal, local);
    
    // handle boundary cases: when projection is not on segment, the
    // nearest point is one of the endpoints of the segment
    if (*segmentProjection < 0) {
        *nearestPointOnSegment = ep0;
        *segmentProjection = 0;
        return float3Distance(point, ep0);
    }
    
    if (*segmentProjection > segmentLength) {
        *nearestPointOnSegment = ep1;
        *segmentProjection = segmentLength;
        return float3Distance(point, ep1);
    }
    
    // otherwise nearest point is projection point on segment
    *nearestPointOnSegment = float3Mul(segmentNormal, *segmentProjection);
    *nearestPointOnSegment = float3Add(*nearestPointOnSegment, ep0);
    
    return float3Distance(point, *nearestPointOnSegment);
}

__device__ float
getTotalPathLength(float3 *points, int numOfPoints)
{
    float totalLength = 0;
    int i;
    
    for (i = 1; i < numOfPoints; i++) {
        totalLength += float3Length(float3Sub(points[i-1], points[i]));
    }
    
    return totalLength;
}

__device__ float3
getSegmentNormal(float3 p0, float3 p1)
{
    float3 normal = float3Sub(p1, p0);
    return float3Div(normal, float3Length(normal));
}

#endif // _CUDA_PATHWAY_UTILITIES_H_
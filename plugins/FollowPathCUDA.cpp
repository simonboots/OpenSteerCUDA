// ----------------------------------------------------------------------------
//
//
// OpenSteer -- Steering Behaviors for Autonomous Characters
//
// Copyright (c) 2002-2003, Sony Computer Entertainment America
// Original author: Craig Reynolds <craig_reynolds@playstation.sony.com>
//
// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.
//
//
// ----------------------------------------------------------------------------
//
//
// FollowPathCUDA: a FollowPathCUDA OpenSteerDemo PlugIn
//
// 04-14-09 sst: created 
//
//
// ----------------------------------------------------------------------------


#include <iomanip>
#include <sstream>
#include <iostream>
#include "OpenSteer/SimpleVehicleMB.h"
#include "OpenSteer/Obstacle.h"
#include "OpenSteer/OpenSteerDemo.h"
#include "OpenSteer/Pathway.h"
#include "OpenSteer/PathwayData.h"
#include "OpenSteer/PathwayDataFunc.h"
#include "OpenSteer/VehicleData.h"
#include "OpenSteer/ObstacleData.h"
#include "OpenSteer/CUDAPlugIn.h"
#include "kernelclasses.h"

using namespace OpenSteer;

typedef std::vector<SphericalObstacle*> SOG;  // spherical obstacle group
typedef SOG::const_iterator SOI;              // spherical obstacle iterator

// ----------------------------------------------------------------------------

PolylinePathway* getTestPathForFollowPathCUDA (void);
PolylinePathway* gTestPathForFollowPathCUDA = NULL;
Vec3 gEndpoint0ForFollowPathCUDA;
Vec3 gEndpoint1ForFollowPathCUDA;

// ----------------------------------------------------------------------------

class FollowPathCUDA : public SimpleVehicleMB
    {
    public:
        
        // constructor
        FollowPathCUDA () {
            reset ();
        }
        
        // reset state
        void reset (void)
        {
            SimpleVehicleMB::reset (); // reset the vehicle 
            setSpeed (1.5f);         // speed along Forward direction.
            setMaxForce (10.f);      // steering force is clipped to this magnitude
            setMaxSpeed (5);         // velocity is clipped to this magnitude
            setPosition ( RandomUnitVectorOnXZPlane() * 5);        // randomize initial position
            
            randomizeHeadingOnXZPlane();
            
            clearTrailHistory ();    // prevent long streaks due to teleportation 
            pathDirection = (frandom01() > 0.5) ? -1 : +1;
            path = getTestPathForFollowPathCUDA();
        }
        
        // per frame simulation update
        void update (const float currentTime, const float elapsedTime)
        {
            //applySteeringForce(steerToFollowPath(pathDirection, 3.f, *path), elapsedTime);
            
            //if (Vec3::distance(position(), gEndpoint0ForFollowPathCUDA) < path->radius) pathDirection = 1;
            //if (Vec3::distance(position(), gEndpoint1ForFollowPathCUDA) < path->radius) pathDirection = -1;
            //applySteeringForce(steerForWander(elapsedTime).setYtoZero(), elapsedTime);
            annotationVelocityAcceleration ();
            recordTrailVertex (currentTime, position());
        }
        
        // draw this character/vehicle into the scene
        void draw (void)
        {
            drawBasic2dCircularVehicle (*this, gGray50);
            drawTrail ();
        }
        
        PolylinePathway* path;
        int pathDirection;
    };


// ----------------------------------------------------------------------------
// PlugIn for OpenSteerDemo


class FollowPathCUDAPlugIn : public CUDAPlugIn
    {
    public:
        
        const char* name (void) {return "FollowPath CUDA";}
        
        float selectionOrderSortKey (void) {return 2.f;}
        
        const static int numOfObstacles = 2;
        
        int *directions;
        
        // be more "nice" to avoid a compiler warning
        virtual ~FollowPathCUDAPlugIn() {}
        
        void open (void)
        {
            setNumberOfAgents(2048);
            directions = new int[getNumberOfAgents()];
            
            for (int i = 0; i<numOfAgents; i++) {
                theVehicles.push_back(new FollowPathCUDA());
                directions[i] = (theVehicles.back())->pathDirection;
            }
            gFollowPathCUDA = theVehicles.front();
            OpenSteerDemo::selectedVehicle = gFollowPathCUDA;
            
            // initialize camera
            OpenSteerDemo::init2dCamera (*gFollowPathCUDA);
            OpenSteerDemo::camera.setPosition (10,
                                               OpenSteerDemo::camera2dElevation,
                                               10);
            OpenSteerDemo::camera.fixedPosition.set (40, 40, 40);
            
            allObstacles.push_back(new SphericalObstacle(3.f, Vec3(0.5, 0, 30.5)));
            allObstacles.push_back(new SphericalObstacle(8.f, Vec3(51.5, 0, 15.5)));
            
            SteerToAvoidObstacles *stao = new SteerToAvoidObstacles(1.f, NONE);
            addKernel(stao);
            
            SteerToFollowPath *stfp = new SteerToFollowPath(3.f, 1.f, IGNORE_UNLESS_ZERO);
            addKernel(stfp);
            
            addKernel(new Modify(NONE));
            
            initKernels();
            
            stao->setObstacles(&allObstacles);
            stfp->setPathwayData(*(getTestPathForFollowPathCUDA()));
            stfp->setDirections(directions);
        }
        
        void update (const float currentTime, const float elapsedTime)
        {
            CUDAPlugIn::update(currentTime, elapsedTime);
            
            for (iterator iter = theVehicles.begin(); iter != theVehicles.end(); iter++) {
                (*iter)->update(currentTime, elapsedTime);
            }
        }
        
        void redraw (const float currentTime, const float elapsedTime)
        {
            for (iterator iter = theVehicles.begin(); iter != theVehicles.end(); iter++) {
                (*iter)->draw();
            }
            
            // textual annotation (following the test vehicle's screen position)
            //std::ostringstream annote;
            //annote << std::setprecision (2) << std::setiosflags (std::ios::fixed);
            //annote << "      speed: " << gPolonaise->speed() << std::ends;
            //draw2dTextAt3dLocation (annote, gPolonaise->position(), gRed);
            draw2dTextAt3dLocation (*"start", Vec3::zero, gGreen);
            
            drawPath();
            // update camera, tracking test vehicle
            OpenSteerDemo::updateCamera (currentTime, elapsedTime, *gFollowPathCUDA);
            
            // draw "ground plane"
            OpenSteerDemo::gridUtility (gFollowPathCUDA->position());
            
            drawObstacles();
        }
        
        void drawObstacles (void)
        {
            const Vec3 color (0.8f, 0.6f, 0.4f);
            const SOG& allSO = allObstacles;
            for (SOI so = allSO.begin(); so != allSO.end(); so++)
            {
                drawXZCircle ((**so).radius, (**so).center, color, 40);
            }
        }
        
        void drawPath (void)
        {
            // draw a line along each segment of path
            const PolylinePathway& path = *getTestPathForFollowPathCUDA ();
            for (int i = 0; i < path.pointCount; i++)
                if (i > 0) drawLine (path.points[i], path.points[i-1], gRed);
        }
        
        void close (void)
        {
            closeKernels();

            theVehicles.clear ();
            delete (gFollowPathCUDA);
            gFollowPathCUDA = NULL;
            
            CUDAPlugIn::close();
        }
        
        void reset (void)
        {
            for (iterator iter = theVehicles.begin(); iter != theVehicles.end(); iter++) {
                (*iter)->reset();
            }
        }
        
        const AVGroup& allVehicles (void) {return (const AVGroup&) theVehicles;}
        
        FollowPathCUDA* gFollowPathCUDA;
        std::vector<FollowPathCUDA*> theVehicles; // for allVehicles
        SOG allObstacles;
        typedef std::vector<FollowPathCUDA*>::const_iterator iterator;
    };


FollowPathCUDAPlugIn gFollowPathCUDAPlugIn;


// ----------------------------------------------------------------------------
// create path for PlugIn 
//
//
//        | gap |
//
//        f      b
//        |\    /\        -
//        | \  /  \       ^
//        |  \/    \      |
//        |  /\     \     |
//        | /  \     c   top
//        |/    \g  /     |
//        /        /      |
//       /|       /       V      z     y=0
//      / |______/        -      ^
//     /  e      d               |
//   a/                          |
//    |<---out-->|               o----> x
//


PolylinePathway* getTestPathForFollowPathCUDA (void)
{
    if (gTestPathForFollowPathCUDA == NULL)
    {
        const float pathRadius = 2.;
        
        const int pathPointCount = 7;
        const float size = 30;
        const float top = 2 * size;
        const float gap = 1.2f * size;
        const float out = 2 * size;
        const float h = 0.5;
        const Vec3 pathPoints[pathPointCount] =
        {Vec3 (h+gap-out,     0,  h+top+out),  // 0 a
            Vec3 (h+gap,         0,  h+top),      // 1 b
            Vec3 (h+gap+(top/2), 0,  h+top/2),    // 2 c
            Vec3 (h+gap,         0,  h),          // 3 d
            Vec3 (h,             0,  h),          // 4 e
            Vec3 (h,             0,  h+top),      // 5 f
        Vec3 (h+gap,         0,  h+top/2)};   // 6 g
        
        //        gObstacle1.center = interpolate (0.2f, pathPoints[0], pathPoints[1]);
        //        gObstacle2.center = interpolate (0.5f, pathPoints[2], pathPoints[3]);
        //        gObstacle1.radius = 3;
        //        gObstacle2.radius = 5;
        //        gObstacles.push_back (&gObstacle1);
        //        gObstacles.push_back (&gObstacle2);
        //        
        gEndpoint0ForFollowPathCUDA = pathPoints[0];
        gEndpoint1ForFollowPathCUDA = pathPoints[pathPointCount-1];
        
        gTestPathForFollowPathCUDA = new PolylinePathway (pathPointCount,
                                                      pathPoints,
                                                      pathRadius,
                                                      false);
    }
    return gTestPathForFollowPathCUDA;
}


// ----------------------------------------------------------------------------

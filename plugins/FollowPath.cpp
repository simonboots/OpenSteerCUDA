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
// FollowPath: a FollowPath OpenSteerDemo PlugIn
//
// 04-13-09 sst: created 
//
//
// ----------------------------------------------------------------------------


#include <iomanip>
#include <sstream>
#include "OpenSteer/SimpleVehicleMB.h"
#include "OpenSteer/OpenSteerDemo.h"
#include "OpenSteer/Pathway.h"


using namespace OpenSteer;


// ----------------------------------------------------------------------------

PolylinePathway* getTestPathForFollowPath (void);
PolylinePathway* gTestPathForFollowPath = NULL;
Vec3 gEndpoint0ForFollowPath;
Vec3 gEndpoint1ForFollowPath;

// ----------------------------------------------------------------------------

class FollowPath : public SimpleVehicleMB
    {
    public:
        
        // constructor
        FollowPath () {
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
            path = getTestPathForFollowPath();
        }
        
        // per frame simulation update
        void update (const float currentTime, const float elapsedTime)
        {
            applySteeringForce(steerToFollowPath(pathDirection, 3.f, *path), elapsedTime);
            
            if (Vec3::distance(position(), gEndpoint0ForFollowPath) < path->radius) pathDirection = 1;
            if (Vec3::distance(position(), gEndpoint1ForFollowPath) < path->radius) pathDirection = -1;
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


class FollowPathPlugIn : public PlugIn
    {
    public:
        
        const char* name (void) {return "FollowPath";}
        
        float selectionOrderSortKey (void) {return 1.f;}
        
        const static int numOfAgents = 2048;
        
        // be more "nice" to avoid a compiler warning
        virtual ~FollowPathPlugIn() {}
        
        void open (void)
        {
            for (int i = 0; i<numOfAgents; i++) {
                theVehicles.push_back(new FollowPath());
            }
            gFollowPath = theVehicles.front();
            OpenSteerDemo::selectedVehicle = gFollowPath;
            
            // initialize camera
            OpenSteerDemo::init2dCamera (*gFollowPath);
            OpenSteerDemo::camera.setPosition (10,
                                               OpenSteerDemo::camera2dElevation,
                                               10);
            OpenSteerDemo::camera.fixedPosition.set (40, 40, 40);
        }
        
        void update (const float currentTime, const float elapsedTime)
        {
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
            OpenSteerDemo::updateCamera (currentTime, elapsedTime, *gFollowPath);
            
            // draw "ground plane"
            OpenSteerDemo::gridUtility (gFollowPath->position());
        }
        
        void drawPath (void)
        {
            // draw a line along each segment of path
            const PolylinePathway& path = *getTestPathForFollowPath ();
            for (int i = 0; i < path.pointCount; i++)
                if (i > 0) drawLine (path.points[i], path.points[i-1], gRed);
        }
        
        void close (void)
        {
            theVehicles.clear ();
            delete (gFollowPath);
            gFollowPath = NULL;
            
            // reset MemoryBackend of SimpleVehicleMB
            SimpleVehicleMB::resetBackend();
        }
        
        void reset (void)
        {
            for (iterator iter = theVehicles.begin(); iter != theVehicles.end(); iter++) {
                (*iter)->reset();
            }
        }
        
        const AVGroup& allVehicles (void) {return (const AVGroup&) theVehicles;}
        
        FollowPath* gFollowPath;
        std::vector<FollowPath*> theVehicles; // for allVehicles
        typedef std::vector<FollowPath*>::const_iterator iterator;
    };


FollowPathPlugIn gFollowPathPlugIn;


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


PolylinePathway* getTestPathForFollowPath (void)
{
    if (gTestPathForFollowPath == NULL)
    {
        const float pathRadius = 2;
        
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
        gEndpoint0ForFollowPath = pathPoints[0];
        gEndpoint1ForFollowPath = pathPoints[pathPointCount-1];
        
        gTestPathForFollowPath = new PolylinePathway (pathPointCount,
                                         pathPoints,
                                         pathRadius,
                                         false);
    }
    return gTestPathForFollowPath;
}


// ----------------------------------------------------------------------------

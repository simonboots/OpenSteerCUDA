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
// WanderAroundCUDA: a WanderAroundCUDA OpenSteerDemo PlugIn
//
// 04-09-09 sst: created 
//
//
// ----------------------------------------------------------------------------


#include <iomanip>
#include <sstream>
#include "OpenSteer/SimpleVehicleMB.h"
#include "OpenSteer/ObstacleData.h"
#include "OpenSteer/OpenSteerDemo.h"
#include "OpenSteer/CUDAPlugIn.h"
#include "CUDAKernelOptions.cu"
#include "kernelclasses.h"

#define testOneObstacleOverlap(radius, center)               \
{                                                            \
    float d = Vec3::distance (c, center);                    \
    float clearance = d - (r + (radius));                    \
    if (minClearance > clearance) minClearance = clearance;  \
}

using namespace OpenSteer;

typedef std::vector<SphericalObstacle*> SOG;  // spherical obstacle group
typedef SOG::const_iterator SOI;              // spherical obstacle iterator

const float gMaxStartRadius = 40;


// ----------------------------------------------------------------------------


class WanderAroundCUDA : public SimpleVehicleMB
    {
    public:
        
        // constructor
            WanderAroundCUDA () {
            reset ();
        }
        
        // reset state
        void reset (void)
        {
            SimpleVehicleMB::reset (); // reset the vehicle 
            setSpeed (1.5f);         // speed along Forward direction.
            setMaxForce (10.f);      // steering force is clipped to this magnitude
            setMaxSpeed (1.5);         // velocity is clipped to this magnitude
            setPosition ( RandomUnitVectorOnXZPlane() * 5);        // randomize initial position
            randomizeHeadingOnXZPlane();
            clearTrailHistory ();    // prevent long streaks due to teleportation 
        }
        
        // per frame simulation update
        void update (const float currentTime, const float elapsedTime)
        {
            //annotationVelocityAcceleration ();
            //recordTrailVertex (currentTime, position());
        }
        
        // draw this character/vehicle into the scene
        void draw (void)
        {
            drawBasic2dCircularVehicle (*this, gGray50);
            //drawTrail ();
        }
    };


// ----------------------------------------------------------------------------
// PlugIn for OpenSteerDemo


class WanderAroundCUDAPlugIn : public CUDAPlugIn
    {
    public:
        
        const char* name (void) {return "WanderAround CUDA";}
        
        float selectionOrderSortKey (void) {return 5.f;}
        
        const static int numOfObstacles = 30;
        unsigned int obstacleCount;
        
        // be more "nice" to avoid a compiler warning
        virtual ~WanderAroundCUDAPlugIn() {}
        
        void open (void)
        {
            setNumberOfAgents(4096);
            
            for (int i = 0; i<getNumberOfAgents(); i++) {
                theVehicles.push_back(new WanderAroundCUDA());
            }
            
            gWanderAround = theVehicles.front();
            OpenSteerDemo::selectedVehicle = gWanderAround;
            
            // initialize camera
            OpenSteerDemo::init2dCamera (*gWanderAround);
            OpenSteerDemo::camera.setPosition (10,
                                               OpenSteerDemo::camera2dElevation,
                                               10);
            OpenSteerDemo::camera.fixedPosition.set (40, 40, 40);
            
            obstacleCount = 0;
            
            for (int i = 0; i < numOfObstacles; i++) addOneObstacle();
            
            FindNeighbors *fn = new FindNeighbors(4.24f);
            addKernel(fn);
            SteerToAvoidObstacles *stao = new SteerToAvoidObstacles(1.f, NONE);
            addKernel(stao);
            addKernel(new SteerToAvoidCloseNeighbors(fn, 0.f, 8.f, IGNORE_UNLESS_ZERO));
            addKernel(new SteerToAvoidNeighbors(fn, 3.f, 8.f, IGNORE_UNLESS_ZERO));
            addKernel(new SteerForWander(1.f, NONE));
            addKernel(new Modify(SPHERICAL_WRAP_AROUND));
            
            initKernels();
            
            stao->setObstacles(&allObstacles);
            
        }
        
        void update (const float currentTime, const float elapsedTime)
        {
            CUDAPlugIn::update(currentTime, elapsedTime);

            //for (iterator iter = theVehicles.begin(); iter != theVehicles.end(); iter++) {
//                (*iter)->update(currentTime, elapsedTime);
//            }
        }
        
        void redraw (const float currentTime, const float elapsedTime)
        {
            for (iterator iter = theVehicles.begin(); iter != theVehicles.end(); iter++) {
                (*iter)->draw();
            }

            // update camera, tracking test vehicle
            OpenSteerDemo::updateCamera (currentTime, elapsedTime, *gWanderAround);
            
            // draw "ground plane"
            OpenSteerDemo::gridUtility (gWanderAround->position());

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
        
        void addOneObstacle (void)
        {
            
            // pick a random center and radius,
            // loop until no overlap with other obstacles and the home base
            float r;
            Vec3 c;
            float minClearance;
            const float requiredClearance = gWanderAround->radius() * 4; // 2 x diameter
            do
            {
                r = frandom2 (1.5, 4);
                c = randomVectorOnUnitRadiusXZDisk () * gMaxStartRadius * 1.1f;
                minClearance = FLT_MAX;
                
                for (SOI so = allObstacles.begin(); so != allObstacles.end(); so++)
                {
                    testOneObstacleOverlap ((**so).radius, (**so).center);
                }
            }
            while (minClearance < requiredClearance);
            
            // add new non-overlapping obstacle to registry
            allObstacles.push_back (new SphericalObstacle (r, c));
            obstacleCount++;
        }
        
        
        void close (void)
        {
            closeKernels();
            theVehicles.clear ();
            delete (gWanderAround);
            gWanderAround = NULL;
            allObstacles.clear();
            //obstacleCount = 0;

            CUDAPlugIn::close();
        }
        
        void reset (void)
        {
            for (iterator iter = theVehicles.begin(); iter != theVehicles.end(); iter++) {
                (*iter)->reset();
            }
        }
        
        const AVGroup& allVehicles (void) {return (const AVGroup&) theVehicles;}
        
        WanderAroundCUDA* gWanderAround;
        std::vector<WanderAroundCUDA*> theVehicles; // for allVehicles
        SOG allObstacles;
        typedef std::vector<WanderAroundCUDA*>::const_iterator iterator;
    };


WanderAroundCUDAPlugIn gWanderAroundCUDAPlugIn;


// ----------------------------------------------------------------------------

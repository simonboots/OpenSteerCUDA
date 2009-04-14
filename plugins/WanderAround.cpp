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
// WanderAround: a WanderAround OpenSteerDemo PlugIn
//
// 04-08-09 sst: created 
//
//
// ----------------------------------------------------------------------------


#include <iomanip>
#include <sstream>
#include "OpenSteer/SimpleVehicleMB.h"
#include "OpenSteer/OpenSteerDemo.h"


using namespace OpenSteer;


// ----------------------------------------------------------------------------


class WanderAround : public SimpleVehicleMB
    {
    public:
        
        // constructor
        WanderAround () {
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
        }
        
        // per frame simulation update
        void update (const float currentTime, const float elapsedTime)
        {
            applySteeringForce(steerForWander(elapsedTime).setYtoZero(), elapsedTime);
            annotationVelocityAcceleration ();
            recordTrailVertex (currentTime, position());
        }
        
        // draw this character/vehicle into the scene
        void draw (void)
        {
            drawBasic2dCircularVehicle (*this, gGray50);
            drawTrail ();
        }
    };


// ----------------------------------------------------------------------------
// PlugIn for OpenSteerDemo


class WanderAroundPlugIn : public PlugIn
    {
    public:
        
        const char* name (void) {return "WanderAround";}
        
        float selectionOrderSortKey (void) {return 4.f;}
        
        const static int numOfAgents = 2048;
        
        // be more "nice" to avoid a compiler warning
        virtual ~WanderAroundPlugIn() {}
        
        void open (void)
        {
            for (int i = 0; i<numOfAgents; i++) {
                theVehicles.push_back(new WanderAround());
            }
            gWanderAround = theVehicles.front();
            OpenSteerDemo::selectedVehicle = gWanderAround;
            
            // initialize camera
            OpenSteerDemo::init2dCamera (*gWanderAround);
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
            
            // update camera, tracking test vehicle
            OpenSteerDemo::updateCamera (currentTime, elapsedTime, *gWanderAround);
            
            // draw "ground plane"
            OpenSteerDemo::gridUtility (gWanderAround->position());
        }
        
        void close (void)
        {
            theVehicles.clear ();
            delete (gWanderAround);
            gWanderAround = NULL;
            
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
        
        WanderAround* gWanderAround;
        std::vector<WanderAround*> theVehicles; // for allVehicles
        typedef std::vector<WanderAround*>::const_iterator iterator;
    };


WanderAroundPlugIn gWanderAroundPlugIn;




// ----------------------------------------------------------------------------

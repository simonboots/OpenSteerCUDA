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
// JustWanderCUDA: a JustWanderCUDA OpenSteerDemo PlugIn
//
// 04-09-09 sst: created 
//
//
// ----------------------------------------------------------------------------

#include "OpenSteer/OpenSteerDemo.h"
#include "OpenSteer/SimpleVehicleMB.h"
#include "OpenSteer/CUDAPlugIn.h"
#include "SteerForWander.h"
#include "Update.h"

using namespace OpenSteer;

class JustWanderCUDA : public SimpleVehicleMB
    {
    public:
        
        // constructor
        JustWanderCUDA () {
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
    };


// ----------------------------------------------------------------------------
// PlugIn for OpenSteerDemo


class JustWanderCUDAPlugIn : public CUDAPlugIn
    {
    public:
        JustWanderCUDAPlugIn() : CUDAPlugIn() {}
        
        const char* name (void) {return "JustWander CUDA";}
        
        float selectionOrderSortKey (void) {return 0.f;}
                
        // be more "nice" to avoid a compiler warning
        virtual ~JustWanderCUDAPlugIn() {}
        
        void open (void)
        {
            setNumberOfAgents(1024);

            for (int i = 0; i<numOfAgents; i++) {
                theVehicles.push_back(new JustWanderCUDA());
            }
            
            gJustWander = theVehicles.front();
            OpenSteerDemo::selectedVehicle = gJustWander;
            
            // initialize camera
            OpenSteerDemo::init2dCamera (*gJustWander);
            OpenSteerDemo::camera.setPosition (10,
                                               OpenSteerDemo::camera2dElevation,
                                               10);
            OpenSteerDemo::camera.fixedPosition.set (40, 40, 40);
            addKernel(new SteerForWander(1.f, NONE));
            addKernel(new Update(NONE));
            
            initKernels();
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
            
            // update camera, tracking test vehicle
            OpenSteerDemo::updateCamera (currentTime, elapsedTime, *gJustWander);
            
            // draw "ground plane"
            OpenSteerDemo::gridUtility (gJustWander->position());            
        }
        
        void close (void)
        {
            closeKernels();
            theVehicles.clear ();
            delete (gJustWander);
            gJustWander = NULL;            
            
            CUDAPlugIn::close();
        }
        
        void reset (void)
        {
            for (iterator iter = theVehicles.begin(); iter != theVehicles.end(); iter++) {
                (*iter)->reset();
            }
        }
        
        const AVGroup& allVehicles (void) {return (const AVGroup&) theVehicles;}
        
        JustWanderCUDA* gJustWander;
        std::vector<JustWanderCUDA*> theVehicles; // for allVehicles
        typedef std::vector<JustWanderCUDA*>::const_iterator iterator;
    };


JustWanderCUDAPlugIn gJustWanderCUDAPlugIn;

// ----------------------------------------------------------------------------

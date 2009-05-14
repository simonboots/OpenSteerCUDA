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
// Polonaise: a Polonaise OpenSteerDemo PlugIn
//
// 03-18-09 sst: created 
//
//
// ----------------------------------------------------------------------------


#include <iomanip>
#include <sstream>
#include "OpenSteer/SimpleVehicleMB.h"
#include "OpenSteer/OpenSteerDemo.h"
#include "OpenSteer/CUDAPlugIn.h"
#include "kernelclasses.h"

using namespace OpenSteer;


// ----------------------------------------------------------------------------


class PolonaiseCUDA : public SimpleVehicleMB
{
public:

    // constructor
    PolonaiseCUDA () {
        reset ();
    }

    // reset state
    void reset (void)
    {
        SimpleVehicleMB::reset (); // reset the vehicle 
        setSpeed (1.5f);         // speed along Forward direction.
        setMaxForce (10.f);      // steering force is clipped to this magnitude
        setMaxSpeed (5);         // velocity is clipped to this magnitude
        setPosition ( RandomUnitVectorOnXZPlane() * 10);        // randomize initial position
        randomizeHeadingOnXZPlane();
        clearTrailHistory ();    // prevent long streaks due to teleportation 
    }

    // per frame simulation update
    void update (const float currentTime, const float elapsedTime)
    {
        annotationVelocityAcceleration ();
        recordTrailVertex (currentTime, position());
    }

    // draw this character/vehicle into the scene
    void draw (void)
    {
        drawBasic2dCircularVehicle (*this, gGray50);
        drawTrail ();
    }
private:
    std::vector<PolonaiseCUDA*> *allVehicles;
    typedef std::vector<PolonaiseCUDA*>::const_iterator iterator;
    
};


// ----------------------------------------------------------------------------
// PlugIn for OpenSteerDemo


class PolonaiseCUDAPlugIn : public CUDAPlugIn
{
public:
    
    const char* name (void) {return "Polonaise CUDA";}

    float selectionOrderSortKey (void) {return 4.f;}

    // be more "nice" to avoid a compiler warning
    virtual ~PolonaiseCUDAPlugIn() {}

    void open (void)
    {
        setNumberOfAgents(2048);
        for (int i = 0; i<getNumberOfAgents(); i++) {
            theVehicle.push_back(new PolonaiseCUDA());
        }
        gPolonaise = theVehicle.front();
        OpenSteerDemo::selectedVehicle = gPolonaise;

        // initialize camera
        OpenSteerDemo::init2dCamera (*gPolonaise);
        OpenSteerDemo::camera.setPosition (10,
                                           OpenSteerDemo::camera2dElevation,
                                           10);
        OpenSteerDemo::camera.fixedPosition.set (40, 40, 40);
        
        // initialize kernels
        FindFollower *ff = new FindFollower();
        addKernel(ff);
        addKernel(new SteerForSeek(ff, 1.f, NONE));
        addKernel(new Modify(NONE));
        
        initKernels();
    }

    void update (const float currentTime, const float elapsedTime)
    {        
        CUDAPlugIn::update(currentTime, elapsedTime);
        
        for (iterator iter = theVehicle.begin(); iter != theVehicle.end(); iter++) {
            (*iter)->update(currentTime, elapsedTime);
        }
    }

    void redraw (const float currentTime, const float elapsedTime)
    {
        for (iterator iter = theVehicle.begin(); iter != theVehicle.end(); iter++) {
            (*iter)->draw();
        }
        // textual annotation (following the test vehicle's screen position)
        std::ostringstream annote;
        annote << std::setprecision (2) << std::setiosflags (std::ios::fixed);
        annote << "      speed: " << gPolonaise->speed() << std::ends;
        draw2dTextAt3dLocation (annote, gPolonaise->position(), gRed);
        draw2dTextAt3dLocation (*"start", Vec3::zero, gGreen);

        // update camera, tracking test vehicle
        OpenSteerDemo::updateCamera (currentTime, elapsedTime, *gPolonaise);

        // draw "ground plane"
        OpenSteerDemo::gridUtility (gPolonaise->position());
    }

    void close (void)
    {
        closeKernels();
        theVehicle.clear ();
        delete (gPolonaise);
        gPolonaise = NULL;        
        
        CUDAPlugIn::close();
    }

    void reset (void)
    {
        for (iterator iter = theVehicle.begin(); iter != theVehicle.end(); iter++) {
            (*iter)->reset();
        }
    }

    const AVGroup& allVehicles (void) {return (const AVGroup&) theVehicle;}

    PolonaiseCUDA* gPolonaise;
    std::vector<PolonaiseCUDA*> theVehicle; // for allVehicles
    typedef std::vector<PolonaiseCUDA*>::const_iterator iterator;
};


PolonaiseCUDAPlugIn gPolonaiseCUDAPlugIn;

// ----------------------------------------------------------------------------

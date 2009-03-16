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
// One vehicle turning way: a (near) minimal OpenSteerDemo PlugIn
//
// 06-24-02 cwr: created 
//
//
// ----------------------------------------------------------------------------


#include <iomanip>
#include <sstream>
#include "OpenSteer/SimpleVehicle.h"
#include "OpenSteer/OpenSteerDemo.h"

void runCUDAKernel(int* values, char* device_name);

using namespace OpenSteer;


// ----------------------------------------------------------------------------


class OneTurningCUDA : public SimpleVehicle
{
public:

    // constructor
    OneTurningCUDA () {reset ();}

    // reset state
    void reset (void)
    {
        SimpleVehicle::reset (); // reset the vehicle 
        setSpeed (1.5f);         // speed along Forward direction.
        setMaxForce (0.3f);      // steering force is clipped to this magnitude
        setMaxSpeed (5);         // velocity is clipped to this magnitude
        clearTrailHistory ();    // prevent long streaks due to teleportation 
    }

    // per frame simulation update
    void update (const float currentTime, const float elapsedTime)
    {
        applySteeringForce (Vec3 (-2, 0, -3), elapsedTime);
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


class OneTurningCUDAPlugIn : public PlugIn
{
public:
    
    const char* name (void) {return "One Turning Away (CUDA)";}

    float selectionOrderSortKey (void) {return 0.06f;}

    // be more "nice" to avoid a compiler warning
    virtual ~OneTurningCUDAPlugIn() {}

    void open (void)
    {
        gOneTurning = new OneTurningCUDA;
        OpenSteerDemo::selectedVehicle = gOneTurning;
        theVehicle.push_back (gOneTurning);

        // initialize camera
        OpenSteerDemo::init2dCamera (*gOneTurning);
        OpenSteerDemo::camera.setPosition (10,
                                           OpenSteerDemo::camera2dElevation,
                                           10);
        OpenSteerDemo::camera.fixedPosition.set (40, 40, 40);
    }

    void update (const float currentTime, const float elapsedTime)
    {
        // update simulation of test vehicle
        gOneTurning->update (currentTime, elapsedTime);
    }

    void redraw (const float currentTime, const float elapsedTime)
    {
        // draw test vehicle
        gOneTurning->draw ();

        // textual annotation (following the test vehicle's screen position)
        int values[2] = {3, 9};
        char device_name[256];
        runCUDAKernel(values, device_name);
        std::ostringstream annote;
        annote << std::setprecision (2) << std::setiosflags (std::ios::fixed);
        annote << "      values[0]: " << values[0];
        annote << " values[1]: " << values[1];
        annote << " dev: " << device_name;
        annote << " speed: " << gOneTurning->speed() << std::ends;
        draw2dTextAt3dLocation (annote, gOneTurning->position(), gRed);
        draw2dTextAt3dLocation (*"start", Vec3::zero, gGreen);

        // update camera, tracking test vehicle
        OpenSteerDemo::updateCamera (currentTime, elapsedTime, *gOneTurning);

        // draw "ground plane"
        OpenSteerDemo::gridUtility (gOneTurning->position());
    }

    void close (void)
    {
        theVehicle.clear ();
        delete (gOneTurning);
        gOneTurning = NULL;
    }

    void reset (void)
    {
        // reset vehicle
        gOneTurning->reset ();
    }

    const AVGroup& allVehicles (void) {return (const AVGroup&) theVehicle;}

    OneTurningCUDA* gOneTurning;
    std::vector<OneTurningCUDA*> theVehicle; // for allVehicles
};


OneTurningCUDAPlugIn gOneTurningCUDAPlugIn;




// ----------------------------------------------------------------------------

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
#include "OpenSteer/SimpleVehicle.h"
#include "OpenSteer/OpenSteerDemo.h"


void runPolonaiseKernel(float *data, int numOfAgents);

using namespace OpenSteer;


// ----------------------------------------------------------------------------


class PolonaiseCUDA : public SimpleVehicle
{
public:

    // constructor
    PolonaiseCUDA (std::vector<PolonaiseCUDA*> *vehicles) {
        allVehicles = vehicles;
        reset ();
    }

    // reset state
    void reset (void)
    {
        SimpleVehicle::reset (); // reset the vehicle 
        setSpeed (1.5f);         // speed along Forward direction.
        setMaxForce (10.f);      // steering force is clipped to this magnitude
        setMaxSpeed (5);         // velocity is clipped to this magnitude
        setPosition ( RandomUnitVectorOnXZPlane() * 5);        // randomize initial position
        randomizeHeadingOnXZPlane();
        clearTrailHistory ();    // prevent long streaks due to teleportation 
    }

    // per frame simulation update
    void update (const float currentTime, const float elapsedTime, Vec3& desiredVelocity)
    {        
        applySteeringForce (desiredVelocity, elapsedTime);
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


class PolonaiseCUDAPlugIn : public PlugIn
{
public:
    
    const char* name (void) {return "Polonaise CUDA";}

    float selectionOrderSortKey (void) {return 0.002f;}
    
    const static int numOfAgents = 1024;

    // be more "nice" to avoid a compiler warning
    virtual ~PolonaiseCUDAPlugIn() {}

    void open (void)
    {
        for (int i = 0; i<numOfAgents; i++) {
            theVehicle.push_back(new PolonaiseCUDA(&theVehicle));
        }
        gPolonaise = theVehicle.front();
        OpenSteerDemo::selectedVehicle = gPolonaise;

        // initialize camera
        OpenSteerDemo::init2dCamera (*gPolonaise);
        OpenSteerDemo::camera.setPosition (10,
                                           OpenSteerDemo::camera2dElevation,
                                           10);
        OpenSteerDemo::camera.fixedPosition.set (40, 40, 40);
    }

    void update (const float currentTime, const float elapsedTime)
    {
        /* For steerForSeek we need 3 Vec3s:
           1. position
           2. velocity
           3. target
         
           target can be omitted as target is position of forerunner
         
           For each agent we need 6 float values
           desiredVelocity is stored in Velocity data
         */
        
        float *rawValues = new float[numOfAgents * 6];

        // copy all data to rawValues array
        int i = 0;
        for (iterator iter = theVehicle.begin(); iter != theVehicle.end(); iter++) {
            rawValues[i] = (*iter)->position().x;
            rawValues[i+1] = (*iter)->position().y;
            rawValues[i+2] = (*iter)->position().z;
            rawValues[i+3] = (*iter)->velocity().x;
            rawValues[i+4] = (*iter)->velocity().y;
            rawValues[i+5] = (*iter)->velocity().z;
            i += 6;
        }
        
        runPolonaiseKernel(rawValues, numOfAgents);
        
        
        // use new data for desiredVelocity
        i = 0;
        for (iterator iter = theVehicle.begin(); iter != theVehicle.end(); iter++) {
            Vec3 desiredVelocity(rawValues[i+3], rawValues[i+4], rawValues[i+5]);
            (*iter)->update(currentTime, elapsedTime, desiredVelocity);
            i += 6;
        }
        
        
        delete[] rawValues;
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
        theVehicle.clear ();
        delete (gPolonaise);
        gPolonaise = NULL;
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

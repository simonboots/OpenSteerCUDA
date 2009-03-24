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
#include "vehicle_t.h"


void runPolonaiseKernel(vehicle_t *data, int numOfAgents, float elapsedTime);
void endPolonaise(void);

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
        setPosition ( RandomUnitVectorOnXZPlane() * 10);        // randomize initial position
        randomizeHeadingOnXZPlane();
        clearTrailHistory ();    // prevent long streaks due to teleportation 
    }

    // per frame simulation update
    void update (const float currentTime, const float elapsedTime)
    {
        measurePathCurvature (elapsedTime);
        
        // running average of recent positions
        blendIntoAccumulator (elapsedTime * 0.06f, // QQQ
                              position (),
                              _smoothedPosition);
        
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

    float selectionOrderSortKey (void) {return 0.00002f;}
    
    const static int numOfAgents = NUM_OF_AGENTS;
    vehicle_t vehicleData;

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
        static int counter = 0;

        // copy all data to vehicleData array
        int i = 0;
        if (counter == 0) {
            
            for (iterator iter = theVehicle.begin(); iter != theVehicle.end(); iter++) {
                vehicleData.position[i] = make_float2((*iter)->position().x, (*iter)->position().z);
                vehicleData.velocity[i] = make_float2((*iter)->velocity().x, (*iter)->velocity().z);
                vehicleData.side[i] = make_float2((*iter)->side().x, (*iter)->side().z);
                vehicleData.smoothedAcceleration[i] = make_float2(0.0f, 0.0f);
                i++;
            }
        }
        
        runPolonaiseKernel(&vehicleData, numOfAgents, elapsedTime);
        
        
        // use new data for desiredVelocity
        i = 0;
        for (iterator iter = theVehicle.begin(); iter != theVehicle.end(); iter++) {
            (*iter)->setSpeed(Vec3(vehicleData.velocity[i].x, 0.f, vehicleData.velocity[i].y).length());
            (*iter)->setPosition(Vec3(vehicleData.position[i].x, 0.f, vehicleData.position[i].y));
            (*iter)->setForward(Vec3(vehicleData.velocity[i].x, 0.f, vehicleData.velocity[i].y) / (*iter)->speed());
            (*iter)->setSide(Vec3(vehicleData.side[i].x, 0.f, vehicleData.side[i].y));
            (*iter)->resetSmoothedAcceleration(Vec3(vehicleData.smoothedAcceleration[i].x, 0.f, vehicleData.smoothedAcceleration[i].y));

            (*iter)->update(currentTime, elapsedTime);
            i++;
        }
        
        
        counter = 1;
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
        endPolonaise();
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

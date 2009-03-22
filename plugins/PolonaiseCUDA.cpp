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
    void update (const float currentTime, const float elapsedTime, Vec3 desiredVelocity)
    {        
        applySteeringForce (desiredVelocity, elapsedTime);
        annotationVelocityAcceleration ();
        recordTrailVertex (currentTime, position());
    }

    // draw this character/vehicle into the scene
    void draw (void)
    {
        drawBasic2dCircularVehicle (*this, gGray50);
        //drawTrail ();
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
    
    const static int numOfAgents = 4096;
    vehicle_t *vehicleData;

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
        
        vehicleData = new vehicle_t[numOfAgents];
    }

    void update (const float currentTime, const float elapsedTime)
    {
        static int counter = 0;
        /* For steerForSeek we need 3 Vec3s:
           1. position
           2. velocity
           3. target
         
           target can be omitted as target is position of forerunner
         
           For each agent we need 6 float values
           desiredVelocity is stored in Velocity data
         */

        // copy all data to vehicleData array
        int i = 0;
        if (counter == 0) {
            
            for (iterator iter = theVehicle.begin(); iter != theVehicle.end(); iter++) {
                vehicleData[i].position = make_float3((*iter)->position().x, (*iter)->position().y, (*iter)->position().z);
                vehicleData[i].velocity = make_float3((*iter)->velocity().x, (*iter)->velocity().y, (*iter)->velocity().z);
                vehicleData[i].follow_velocity = make_float3(0.f, 0.f, 0.f);
                vehicleData[i].forward = make_float3((*iter)->forward().x, (*iter)->forward().y, (*iter)->forward().z);
                vehicleData[i].side = make_float3((*iter)->side().x, (*iter)->side().y, (*iter)->side().z);
                vehicleData[i].up = make_float3((*iter)->up().x, (*iter)->up().y, (*iter)->up().z);
                vehicleData[i].new_position = make_float3(0.0f, 0.0f, 0.0f);
                vehicleData[i].smoothedAcceleration = make_float3(0.0f, 0.0f, 0.0f);
                vehicleData[i].speed = (*iter)->speed();
                vehicleData[i].maxSpeed = (*iter)->maxSpeed();
                vehicleData[i].maxForce = (*iter)->maxForce();
                vehicleData[i].mass = (*iter)->mass();
                i++;
            }
        }
        
        runPolonaiseKernel(vehicleData, numOfAgents, elapsedTime);
        
        
        // use new data for desiredVelocity
        i = 0;
        for (iterator iter = theVehicle.begin(); iter != theVehicle.end(); iter++) {
            (*iter)->setSpeed(vehicleData[i].speed);
            (*iter)->setPosition(Vec3(vehicleData[i].new_position.x, vehicleData[i].new_position.y, vehicleData[i].new_position.z));
            (*iter)->setForward(Vec3(vehicleData[i].forward.x, vehicleData[i].forward.y, vehicleData[i].forward.z));
            (*iter)->setSide(Vec3(vehicleData[i].side.x, vehicleData[i].side.y, vehicleData[i].side.z));
            (*iter)->resetSmoothedAcceleration(Vec3(vehicleData[i].smoothedAcceleration.x, vehicleData[i].smoothedAcceleration.y, vehicleData[i].smoothedAcceleration.z));
            vehicleData[i].position = vehicleData[i].new_position;

            (*iter)->update(currentTime, elapsedTime, Vec3(vehicleData[i].velocity.x, vehicleData[i].velocity.y, vehicleData[i].velocity.z));
            i++;
        }
        
        
        counter = 1;
    }

    void redraw (const float currentTime, const float elapsedTime)
    {
//        for (iterator iter = theVehicle.begin(); iter != theVehicle.end(); iter++) {
//            (*iter)->draw();
//        }
//        // textual annotation (following the test vehicle's screen position)
//        std::ostringstream annote;
//        annote << std::setprecision (2) << std::setiosflags (std::ios::fixed);
//        annote << "      speed: " << gPolonaise->speed() << std::ends;
//        draw2dTextAt3dLocation (annote, gPolonaise->position(), gRed);
//        draw2dTextAt3dLocation (*"start", Vec3::zero, gGreen);
//
//        // update camera, tracking test vehicle
//        OpenSteerDemo::updateCamera (currentTime, elapsedTime, *gPolonaise);
//
//        // draw "ground plane"
//        OpenSteerDemo::gridUtility (gPolonaise->position());
    }

    void close (void)
    {
        theVehicle.clear ();
        delete (gPolonaise);
        gPolonaise = NULL;
        delete[] vehicleData;
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

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
// Multiple pursuit (for testing pursuit)
//
// 08-22-02 cwr: created 
//
//
// ----------------------------------------------------------------------------


#include "OpenSteer/SimpleVehicle.h"
#include "OpenSteer/SimpleVehicleMB.h"
#include "OpenSteer/OpenSteerDemo.h"
#include "VehicleData.h"

void runMultiplePursuitKernel(VehicleData *h_vehicleData, int numOfVehicles, float3 wandererPosition, float3 wandererVelocity, float elapsedTime, int copy_vehicle_data);
void endMultiplePursuit(void);

using namespace OpenSteer;

// ----------------------------------------------------------------------------
// This PlugIn uses two vehicle types: MpWanderer and MpPursuer.  They have
// a common base class, MpBase, which is a specialization of SimpleVehicle.


class MpBaseCUDA : public SimpleVehicle
    {
    public:
        
        // constructor
        MpBaseCUDA () {reset ();}
        
        // reset state
        void reset (void)
        {
            SimpleVehicle::reset (); // reset the vehicle 
            setSpeed (3.f);            // speed along Forward direction.
            setMaxForce (5.0);       // steering force is clipped to this magnitude
            setMaxSpeed (3.0);       // velocity is clipped to this magnitude
            clearTrailHistory ();    // prevent long streaks due to teleportation 
            gaudyPursuitAnnotation = true; // select use of 9-color annotation
        }
        
        // draw into the scene
        void draw (void)
        {
            drawBasic2dCircularVehicle (*this, bodyColor);
            drawTrail ();
        }
        
        // for draw method
        Vec3 bodyColor;
    };

class MpBaseCUDAMB : public SimpleVehicleMB
    {
    public:
        
        // constructor
        MpBaseCUDAMB () {reset ();}
        
        // reset state
        void reset (void)
        {
            SimpleVehicleMB::reset (); // reset the vehicle 
            setSpeed (3.f);            // speed along Forward direction.
            setMaxForce (5.0);       // steering force is clipped to this magnitude
            setMaxSpeed (3.0);       // velocity is clipped to this magnitude
            clearTrailHistory ();    // prevent long streaks due to teleportation 
            gaudyPursuitAnnotation = true; // select use of 9-color annotation
        }
        
        // draw into the scene
        void draw (void)
        {
            drawBasic2dCircularVehicle (*this, bodyColor);
            drawTrail ();
        }
        
        // for draw method
        Vec3 bodyColor;
    };


class MpWandererCUDA : public MpBaseCUDA
    {
    public:
        
        // constructor
        MpWandererCUDA () {reset ();}
        
        // reset state
        void reset (void)
        {
            MpBaseCUDA::reset ();
            bodyColor.set (0.4f, 0.6f, 0.4f); // greenish
        }
        
        // one simulation step
        void update (const float currentTime, const float elapsedTime)
        {
            const Vec3 wander2d = steerForWander (elapsedTime).setYtoZero ();
            const Vec3 steer = forward() + (wander2d * 3);
            applySteeringForce (steer, elapsedTime);
            
            // for annotation
            recordTrailVertex (currentTime, position());
        }
        
    };


class MpPursuerCUDA : public MpBaseCUDAMB
    {
    public:
        
        // constructor
        MpPursuerCUDA (MpWandererCUDA* w) {wanderer = w; reset ();}
        static bool did_reset;
        
        // reset state
        void reset (void)
        {
            MpPursuerCUDA::did_reset = true;
            MpBaseCUDAMB::reset ();
            setMaxSpeed (3.2);       // velocity is clipped to this magnitude
            bodyColor.set (0.6f, 0.4f, 0.4f); // redish
            randomizeStartingPositionAndHeading ();
        }
        
        // one simulation step
        void update (const float currentTime, const float elapsedTime)
        {
            // when pursuer touches quarry ("wanderer"), reset its position
            const float d = Vec3::distance (position(), wanderer->position());
            const float r = radius() + wanderer->radius();
            if (d < r) {
                did_reset = true;
                std::cout << "Did reset" << std::endl;
                reset ();
            }
            
            const float maxTime = 20; // xxx hard-to-justify value
            //applySteeringForce (steerForPursuit (*wanderer, maxTime), elapsedTime);
            
            // for annotation
            recordTrailVertex (currentTime, position());
        }
        
        // reset position
        void randomizeStartingPositionAndHeading (void)
        {
            // randomize position on a ring between inner and outer radii
            // centered around the home base
            const float inner = 20;
            const float outer = 30;
            const float radius = frandom2 (inner, outer);
            const Vec3 randomOnRing = RandomUnitVectorOnXZPlane () * radius;
            setPosition (wanderer->position() + randomOnRing);
            
            // randomize 2D heading
            randomizeHeadingOnXZPlane ();
        }
        
        MpWandererCUDA* wanderer;
    };

bool MpPursuerCUDA::did_reset = false;


// ----------------------------------------------------------------------------
// PlugIn for OpenSteerDemo


class MpPlugInCUDA : public PlugIn
    {
    public:
        
        const char* name (void) {return "Multiple Pursuit CUDA";}
        
        float selectionOrderSortKey (void) {return 2.5f;}
        
        VehicleData *vData;
        int pursuerCount;
        
        virtual ~MpPlugInCUDA() {} // be more "nice" to avoid a compiler warning
        
        void open (void)
        {
            // create the wanderer, saving a pointer to it
            wanderer = new MpWandererCUDA;
            //allMP.push_back (wanderer);
            
            // create the specified number of pursuers, save pointers to them
            pursuerCount = 2048;
            for (int i = 0; i < pursuerCount; i++)
                allMP.push_back (new MpPursuerCUDA (wanderer));
            pBegin = allMP.begin();  // iterator pointing to first pursuer
            pEnd = allMP.end();          // iterator pointing to last pursuer
            
            // initialize camera
            OpenSteerDemo::selectedVehicle = wanderer;
            OpenSteerDemo::camera.mode = Camera::cmStraightDown;
            OpenSteerDemo::camera.fixedDistDistance = OpenSteerDemo::cameraTargetDistance;
            OpenSteerDemo::camera.fixedDistVOffset = OpenSteerDemo::camera2dElevation;
        }
        
        void update (const float currentTime, const float elapsedTime)
        {
            // update the wanderer
            wanderer->update (currentTime, elapsedTime);
            
            static bool copy_vehicle_data = true;
            
            MemoryBackend *mb = MemoryBackend::instance();
            vData = mb->getVehicleData();
            
            runMultiplePursuitKernel(vData,
                                     pursuerCount,
                                     make_float3(wanderer->position().x, wanderer->position().y, wanderer->position().z),
                                     make_float3(wanderer->velocity().x, wanderer->velocity().y, wanderer->velocity().z),
                                     elapsedTime,
                                     copy_vehicle_data ? 1 : 0);
            
            copy_vehicle_data = false;
            
           
            // copy data back
            for (iterator i = pBegin; i != pEnd; i++)
            {
                  ((MpPursuerCUDA&) (**i)).update (currentTime, elapsedTime);
            }
            
            
            copy_vehicle_data = MpPursuerCUDA::did_reset;
            MpPursuerCUDA::did_reset = false;
        }
        
        void redraw (const float currentTime, const float elapsedTime)
        {
            // selected vehicle (user can mouse click to select another)
            AbstractVehicle& selected = *OpenSteerDemo::selectedVehicle;
            
            // vehicle nearest mouse (to be highlighted)
            AbstractVehicle& nearMouse = *OpenSteerDemo::vehicleNearestToMouse ();
            
            // update camera
            OpenSteerDemo::updateCamera (currentTime, elapsedTime, selected);
            
            // draw "ground plane"
            OpenSteerDemo::gridUtility (selected.position());
            
            // draw each vehicles
            wanderer->draw();
            for (iterator i = allMP.begin(); i != pEnd; i++) (**i).draw ();
            
            // highlight vehicle nearest mouse
            OpenSteerDemo::highlightVehicleUtility (nearMouse);
            OpenSteerDemo::circleHighlightVehicleUtility (selected);
        }
        
        void close (void)
        {
            // delete wanderer, all pursuers, and clear list
            delete (wanderer);
            for (iterator i = pBegin; i != pEnd; i++) delete ((MpPursuerCUDA*)*i);
            allMP.clear();
            endMultiplePursuit();
            
            // reset MemoryBackend of SimpleVehicleMB
            SimpleVehicleMB::resetBackend();
        }
        
        void reset (void)
        {
            // reset wanderer and pursuers
            wanderer->reset ();
            for (iterator i = pBegin; i != pEnd; i++) ((MpPursuerCUDA&)(**i)).reset ();
            
            // immediately jump to default camera position
            OpenSteerDemo::camera.doNotSmoothNextMove ();
            OpenSteerDemo::camera.resetLocalSpace ();
        }
        
        const AVGroup& allVehicles (void) {return (const AVGroup&) allMP;}
        
        // a group (STL vector) of all vehicles
        std::vector<MpBaseCUDAMB*> allMP;
        typedef std::vector<MpBaseCUDAMB*>::const_iterator iterator;
        iterator pBegin, pEnd;
        
        MpWandererCUDA* wanderer;
    };


MpPlugInCUDA gMpPlugInCUDA;



// ----------------------------------------------------------------------------

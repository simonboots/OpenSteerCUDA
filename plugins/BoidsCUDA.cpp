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
// OpenSteer BoidsCUDA
// 
// 04-21-09 sst: created 
//
//
// ----------------------------------------------------------------------------


#include <sstream>
#include "OpenSteer/SimpleVehicleMB.h"
#include "OpenSteer/OpenSteerDemo.h"
#include "OpenSteer/Grid.h"




// Include names declared in the OpenSteer namespace into the namespaces to search to find names.
using namespace OpenSteer;

// ----------------------------------------------------------------------------

void runBoidsKernel(VehicleData *h_vehicleData, VehicleConst *h_vehicleConst, int numOfVehicles, int* h_neighborIndices, int numOfNIndices, int* h_neighborAgents, int numOfNAgents, float elapsedTime);
void endBoids(void);

// ----------------------------------------------------------------------------


class BoidCUDA : public OpenSteer::SimpleVehicleMB
{
public:
    
    // type for a flock: an STL vector of Boid pointers
    typedef std::vector<BoidCUDA*> groupType;
    
    
    // constructor
    BoidCUDA ()
    {
        // reset all boid state
        reset ();
    }
    
    
    // destructor
    ~BoidCUDA () {}
    
    
    // reset state
    void reset (void)
    {
        // reset the vehicle
        SimpleVehicleMB::reset ();
        
        // steering force is clipped to this magnitude
        setMaxForce (27);
        
        // velocity is clipped to this magnitude
        setMaxSpeed (9);
        
        // initial slow speed
        setSpeed (maxSpeed() * 0.3f);
        
        // randomize initial orientation
        regenerateOrthonormalBasisUF (RandomUnitVector ());
        
        // randomize initial position
        setPosition (RandomVectorInUnitRadiusSphere () * 20);
    }
    
    
    // draw this boid into the scene
    void draw (void)
    {
        drawBasic3dSphericalVehicle (*this, gGray70);
        // drawTrail ();
    }
    
    // allocate one and share amoung instances just to save memory usage
    // (change to per-instance allocation to be more MP-safe)
    static AVGroup neighbors;
    
    static float worldRadius;
};


AVGroup BoidCUDA::neighbors;
float BoidCUDA::worldRadius = 50.0f;

// ----------------------------------------------------------------------------
// PlugIn for OpenSteerDemo


class BoidsCUDAPlugIn : public PlugIn
    {
    public:
        
        const char* name (void) {return "Boids CUDA";}
        
        float selectionOrderSortKey (void) {return 1.f;}
        
        static const int numOfAgents = 4096;
        
        virtual ~BoidsCUDAPlugIn() {} // be more "nice" to avoid a compiler warning
        
        Grid *grid;
        
        void open (void)
        {
            // make default-sized flock
            population = 0;
            for (int i = 0; i < numOfAgents; i++) addBoidToFlock ();
            
            // initialize camera
            OpenSteerDemo::init3dCamera (*OpenSteerDemo::selectedVehicle);
            OpenSteerDemo::camera.mode = Camera::cmFixed;
            OpenSteerDemo::camera.fixedDistDistance = OpenSteerDemo::cameraTargetDistance;
            OpenSteerDemo::camera.fixedDistVOffset = 0;
            OpenSteerDemo::camera.lookdownDistance = 20;
            OpenSteerDemo::camera.aimLeadTime = 0.5;
            OpenSteerDemo::camera.povOffset.set (0, 0.5, -2);
            
            grid = new Grid();
        }
        
        void update (const float currentTime, const float elapsedTime)
        {
            MemoryBackend *mb = MemoryBackend::instance();
            VehicleData *vData = mb->getVehicleData();
            VehicleConst *vConst = mb->getVehicleConst();
            
            int n = 0;
            for (iterator i = flock.begin(); i != flock.end(); i++)
            {
                grid->save((**i).position(), n++);
            }
            
            runBoidsKernel(vData, vConst, numOfAgents, grid->getIndices(), grid->numOfCells(), grid->getAgents(), grid->numOfAgents(), elapsedTime);
            
            grid->clear();
        }
        
        void redraw (const float currentTime, const float elapsedTime)
        {
            // selected vehicle (user can mouse click to select another)
            AbstractVehicle& selected = *OpenSteerDemo::selectedVehicle;
            
            // vehicle nearest mouse (to be highlighted)
            AbstractVehicle& nearMouse = *OpenSteerDemo::vehicleNearestToMouse ();
            
            // update camera
            OpenSteerDemo::updateCamera (currentTime, elapsedTime, selected);
            
            // draw each boid in flock
            for (iterator i = flock.begin(); i != flock.end(); i++) (**i).draw ();
            
            // highlight vehicle nearest mouse
            OpenSteerDemo::drawCircleHighlightOnVehicle (nearMouse, 1, gGray70);
            
            // highlight selected vehicle
            OpenSteerDemo::drawCircleHighlightOnVehicle (selected, 1, gGray50);
        }
        
        void close (void)
        {
            // delete each member of the flock
            while (population > 0) removeBoidFromFlock ();
            
            endBoids();
            
            delete grid;
            
            // reset MemoryBackend of SimpleVehicleMB
            SimpleVehicleMB::resetBackend();
        }
        
        void reset (void)
        {
            // reset each boid in flock
            for (iterator i = flock.begin(); i != flock.end(); i++) (**i).reset();
            
            // reset camera position
            OpenSteerDemo::position3dCamera (*OpenSteerDemo::selectedVehicle);
            
            // make camera jump immediately to new position
            OpenSteerDemo::camera.doNotSmoothNextMove ();
        }
        
        void addBoidToFlock (void)
        {
            population++;
            BoidCUDA* boid = new BoidCUDA ();
            flock.push_back (boid);
            if (population == 1) OpenSteerDemo::selectedVehicle = boid;
        }
        
        void removeBoidFromFlock (void)
        {
            if (population > 0)
            {
                // save a pointer to the last boid, then remove it from the flock
                const BoidCUDA* boid = flock.back();
                flock.pop_back();
                population--;
                
                // if it is OpenSteerDemo's selected vehicle, unselect it
                if (boid == OpenSteerDemo::selectedVehicle)
                    OpenSteerDemo::selectedVehicle = NULL;
                
                // delete the Boid
                delete boid;
            }
        }
        
        // return an AVGroup containing each boid of the flock
        const AVGroup& allVehicles (void) {return (const AVGroup&)flock;}
        
        // flock: a group (STL vector) of pointers to all boids
        BoidCUDA::groupType flock;
        typedef BoidCUDA::groupType::const_iterator iterator;
        
        // keep track of current flock size
        int population;
    };


BoidsCUDAPlugIn gBoidsCUDAPlugIn;



// ----------------------------------------------------------------------------

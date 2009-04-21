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
#include "OpenSteer/Proximity.h"




// Include names declared in the OpenSteer namespace into the namespaces to search to find names.
using namespace OpenSteer;

// ----------------------------------------------------------------------------


typedef OpenSteer::AbstractProximityDatabase<AbstractVehicle*> ProximityDatabase;
typedef OpenSteer::AbstractTokenForProximityDatabase<AbstractVehicle*> ProximityToken;


// ----------------------------------------------------------------------------


class BoidCUDA : public OpenSteer::SimpleVehicleMB
{
public:
    
    // type for a flock: an STL vector of Boid pointers
    typedef std::vector<BoidCUDA*> groupType;
    
    
    // constructor
    BoidCUDA (ProximityDatabase& pd)
    {
        // allocate a token for this boid in the proximity database
        proximityToken = NULL;
        newPD (pd);
        
        // reset all boid state
        reset ();
    }
    
    
    // destructor
    ~BoidCUDA ()
    {
        // delete this boid's token in the proximity database
        delete proximityToken;
    }
    
    
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
        
        // notify proximity database that our position has changed
        proximityToken->updateForNewPosition (position());
    }
    
    
    // draw this boid into the scene
    void draw (void)
    {
        drawBasic3dSphericalVehicle (*this, gGray70);
        // drawTrail ();
    }
    
    
    // per frame simulation update
    void update (const float currentTime, const float elapsedTime)
    {
        // steer to flock and perhaps to stay within the spherical boundary
        applySteeringForce (steerToFlock () + handleBoundary(), elapsedTime);
        
        // notify proximity database that our position has changed
        proximityToken->updateForNewPosition (position());
    }
    
    
    // basic flocking
    Vec3 steerToFlock (void)
    {
        const float separationRadius =  5.0f;
        const float separationAngle  = -0.707f;
        const float separationWeight =  12.0f;
        
        const float alignmentRadius = 7.5f;
        const float alignmentAngle  = 0.7f;
        const float alignmentWeight = 8.0f;
        
        const float cohesionRadius = 9.0f;
        const float cohesionAngle  = -0.15f;
        const float cohesionWeight = 8.0f;
        
        const float maxRadius = maxXXX (separationRadius,
                                        maxXXX (alignmentRadius,
                                                cohesionRadius));
        
        // find all flockmates within maxRadius using proximity database
        neighbors.clear();
        proximityToken->findNeighbors (position(), maxRadius, neighbors);
        
        // determine each of the three component behaviors of flocking
        const Vec3 separation = steerForSeparation (separationRadius,
                                                    separationAngle,
                                                    neighbors);
        const Vec3 alignment  = steerForAlignment  (alignmentRadius,
                                                    alignmentAngle,
                                                    neighbors);
        const Vec3 cohesion   = steerForCohesion   (cohesionRadius,
                                                    cohesionAngle,
                                                    neighbors);
        
        // apply weights to components (save in variables for annotation)
        const Vec3 separationW = separation * separationWeight;
        const Vec3 alignmentW = alignment * alignmentWeight;
        const Vec3 cohesionW = cohesion * cohesionWeight;
        
        // annotation
        // const float s = 0.1;
        // annotationLine (position, position + (separationW * s), gRed);
        // annotationLine (position, position + (alignmentW  * s), gOrange);
        // annotationLine (position, position + (cohesionW   * s), gYellow);
        
        return separationW + alignmentW + cohesionW;
    }
    
    
    // Take action to stay within sphereical boundary.  Returns steering
    // value (which is normally zero) and may take other side-effecting
    // actions such as kinematically changing the Boid's position.
    Vec3 handleBoundary (void)
    {
        // while inside the sphere do noting
        if (position().length() < worldRadius) return Vec3::zero;
        
        // once outside, select strategy
        switch (boundaryCondition)
        {
            case 0:
            {
                // steer back when outside
                const Vec3 seek = xxxsteerForSeek (Vec3::zero);
                const Vec3 lateral = seek.perpendicularComponent (forward ());
                return lateral;
            }
            case 1:
            {
                // wrap around (teleport)
                setPosition (position().sphericalWrapAround (Vec3::zero,
                                                             worldRadius));
                return Vec3::zero;
            }
        }
        return Vec3::zero; // should not reach here
    }
    
    
    // make boids "bank" as they fly
    void regenerateLocalSpace (const Vec3& newVelocity,
                               const float elapsedTime)
    {
        regenerateLocalSpaceForBanking (newVelocity, elapsedTime);
    }
    
    // switch to new proximity database -- just for demo purposes
    void newPD (ProximityDatabase& pd)
    {
        // delete this boid's token in the old proximity database
        delete proximityToken;
        
        // allocate a token for this boid in the proximity database
        proximityToken = pd.allocateToken (this);
    }
    
    
    // cycle through various boundary conditions
    static void nextBoundaryCondition (void)
    {
        const int max = 2;
        boundaryCondition = (boundaryCondition + 1) % max;
    }
    static int boundaryCondition;
    
    // a pointer to this boid's interface object for the proximity database
    ProximityToken* proximityToken;
    
    // allocate one and share amoung instances just to save memory usage
    // (change to per-instance allocation to be more MP-safe)
    static AVGroup neighbors;
    
    static float worldRadius;
};


AVGroup BoidCUDA::neighbors;
float BoidCUDA::worldRadius = 50.0f;
int BoidCUDA::boundaryCondition = 0;


// ----------------------------------------------------------------------------
// PlugIn for OpenSteerDemo


class BoidsCUDAPlugIn : public PlugIn
    {
    public:
        
        const char* name (void) {return "Boids CUDA";}
        
        float selectionOrderSortKey (void) {return 0.5f;}
        
        static const int numOfAgents = 2048;
        
        virtual ~BoidsCUDAPlugIn() {} // be more "nice" to avoid a compiler warning
        
        void open (void)
        {
            // make the database used to accelerate proximity queries
            cyclePD = -1;
            getPD ();
            
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
        }
        
        void update (const float currentTime, const float elapsedTime)
        {
            // update flock simulation for each boid
            for (iterator i = flock.begin(); i != flock.end(); i++)
            {
                (**i).update (currentTime, elapsedTime);
            }
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
            
            // delete the proximity database
            delete pd;
            pd = NULL;
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
        
        // for purposes of demonstration, allow cycling through various
        // types of proximity databases.  this routine is called when the
        // OpenSteerDemo user pushes a function key.
        void getPD (void)
        {
            // save pointer to old PD
            ProximityDatabase* oldPD = pd;
            
            const Vec3 center;
            const float div = 10.0f;
            const Vec3 divisions (div, div, div);
            const float diameter = BoidCUDA::worldRadius * 1.1f * 2;
            const Vec3 dimensions (diameter, diameter, diameter);
            typedef LQProximityDatabase<AbstractVehicle*> LQPDAV;
            pd = new LQPDAV (center, dimensions, divisions);

            //pd = new BruteForceProximityDatabase<AbstractVehicle*> ();
            
            delete oldPD;
        }
        
        void addBoidToFlock (void)
        {
            population++;
            BoidCUDA* boid = new BoidCUDA (*pd);
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
        
        // pointer to database used to accelerate proximity queries
        ProximityDatabase* pd;
        
        // keep track of current flock size
        int population;
        
        // which of the various proximity databases is currently in use
        int cyclePD;
    };


BoidsCUDAPlugIn gBoidsCUDAPlugIn;



// ----------------------------------------------------------------------------

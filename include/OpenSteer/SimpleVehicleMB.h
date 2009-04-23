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
// SimpleVehicle
//
// A steerable point mass with a velocity-aligned local coordinate system.
// SimpleVehicle is useful for developing prototype vehicles in OpenSteerDemo,
// it is the base class for vehicles in the PlugIns supplied with OpenSteer.
// Note that SimpleVehicle is intended only as sample code.  Your application
// can use the OpenSteer library without using SimpleVehicle, as long as you
// implement the AbstractVehicle protocol.
//
// SimpleVehicle makes use of the "mixin" concept from OOP.  To quote
// "Mixin-Based Programming in C++" a clear and helpful paper by Yannis
// Smaragdakis and Don Batory (http://citeseer.nj.nec.com/503808.html):
//
//     ...The idea is simple: we would like to specify an extension without
//     predetermining what exactly it can extend. This is equivalent to
//     specifying a subclass while leaving its superclass as a parameter to be
//     determined later. The benefit is that a single class can be used to
//     express an incremental extension, valid for a variety of classes...
// 
// In OpenSteer, vehicles are defined by an interface: an abstract base class
// called AbstractVehicle.  Implementations of that interface, and related
// functionality (like steering behaviors and vehicle physics) are provided as
// template-based mixin classes.  The intent of this design is to allow you to
// reuse OpenSteer code with your application's own base class.
//
// 10-04-04 bk:  put everything into the OpenSteer namespace
// 01-29-03 cwr: created
//
//
//
// ----------------------------------------------------------------------------


#ifndef OPENSTEER_SIMPLEVEHICLE_MB_H
#define OPENSTEER_SIMPLEVEHICLE_MB_H


#include "AbstractVehicle.h"
#include "SteerLibrary.h"
#include "Annotation.h"


namespace OpenSteer {
    
    
    // ----------------------------------------------------------------------------
    
    
    // SimpleVehicle_1 adds concrete LocalSpace methods to AbstractVehicle
    typedef LocalSpaceMixinMB<AbstractVehicle> SimpleVehicleMB_1;
    
    
    // SimpleVehicle_2 adds concrete annotation methods to SimpleVehicle_1
    typedef AnnotationMixin<SimpleVehicleMB_1> SimpleVehicleMB_2;
    
    
    // SimpleVehicle_3 adds concrete steering methods to SimpleVehicle_2
    typedef SteerLibraryMixin<SimpleVehicleMB_2> SimpleVehicleMB_3;
    
    
    // SimpleVehicle adds concrete vehicle methods to SimpleVehicle_3
    class SimpleVehicleMB : public SimpleVehicleMB_3
        {
        public:
            
            // constructor
            SimpleVehicleMB ();
            
            // destructor
            ~SimpleVehicleMB ();
            
            // reset memory backend
            static void resetBackend()
            {
                MemoryBackend::reset();
            }
            
            // reset vehicle state
            void reset (void)
            {
                // reset LocalSpace state
                resetLocalSpace ();
                
                // reset SteerLibraryMixin state
                // (XXX this seems really fragile, needs to be redesigned XXX)
                SimpleVehicleMB_3::reset ();
                
                setMass (1);          // mass (defaults to 1 so acceleration=force)
                setSpeed (0);         // speed along Forward direction.
                
                setRadius (0.5f);     // size of bounding sphere
                
                setMaxForce (0.1f);   // steering force is clipped to this magnitude
                setMaxSpeed (1.0f);   // velocity is clipped to this magnitude
                
                // reset bookkeeping to do running averages of these quanities
                resetSmoothedAcceleration ();
            }
            
            // get/set mass
            float mass (void) const {return mb->mass(mb_id);}
            float setMass (float m) {return mb->setMass(mb_id, m);}
            
            // get velocity of vehicle
            Vec3 velocity (void) const {return forward() * speed();}
            
            // get/set speed of vehicle  (may be faster than taking mag of velocity)
            float speed (void) const {return mb->speed(mb_id);}
            float setSpeed (float s) {return mb->setSpeed(mb_id, s);}
            
            // size of bounding sphere, for obstacle avoidance, etc.
            float radius (void) const {return mb->radius(mb_id);}
            float setRadius (float m) {return mb->setRadius(mb_id, m);}
            
            // get/set maxForce
            float maxForce (void) const {return mb->maxForce(mb_id);}
            float setMaxForce (float mf) {return mb->setMaxForce(mb_id, mf);}
            
            // get/set maxSpeed
            float maxSpeed (void) const {return mb->maxSpeed(mb_id);}
            float setMaxSpeed (float ms) {return mb->setMaxSpeed(mb_id, ms);}
            
            
            // apply a given steering force to our momentum,
            // adjusting our orientation to maintain velocity-alignment.
            void applySteeringForce (const Vec3& force, const float deltaTime);
            
            // the default version: keep FORWARD parallel to velocity, change
            // UP as little as possible.
            virtual void regenerateLocalSpace (const Vec3& newVelocity,
                                               const float elapsedTime);
            
            // alternate version: keep FORWARD parallel to velocity, adjust UP
            // according to a no-basis-in-reality "banking" behavior, something
            // like what birds and airplanes do.  (XXX experimental cwr 6-5-03)
            void regenerateLocalSpaceForBanking (const Vec3& newVelocity,
                                                 const float elapsedTime);
            
            // adjust the steering force passed to applySteeringForce.
            // allows a specific vehicle class to redefine this adjustment.
            // default is to disallow backward-facing steering at low speed.
            // xxx experimental 8-20-02
            virtual Vec3 adjustRawSteeringForce (const Vec3& force,
                                                 const float deltaTime);
            
            // apply a given braking force (for a given dt) to our momentum.
            // xxx experimental 9-6-02
            void applyBrakingForce (const float rate, const float deltaTime);
            
            // predict position of this vehicle at some time in the future
            // (assumes velocity remains constant)
            Vec3 predictFuturePosition (const float predictionTime) const;
            
            Vec3 smoothedAcceleration (void) {return mb->smoothedAcceleration(mb_id);}
            Vec3 resetSmoothedAcceleration (const Vec3& value = Vec3::zero)
            {
                mb->setSmoothedAcceleration(mb_id, value);
                return value;
            }
            
            // give each vehicle a unique number
            int serialNumber;
            static int serialNumberCounter;
            
            // draw lines from vehicle's position showing its velocity and acceleration
            void annotationVelocityAcceleration (float maxLengthA, float maxLengthV);
            void annotationVelocityAcceleration (float maxLength)
            {annotationVelocityAcceleration (maxLength, maxLength);}
            void annotationVelocityAcceleration (void)
            {annotationVelocityAcceleration (3, 3);}
            
            // set a random "2D" heading: set local Up to global Y, then effectively
            // rotate about it by a random angle (pick random forward, derive side).
            void randomizeHeadingOnXZPlane (void)
            {
                setUp (Vec3::up);
                setForward (RandomUnitVectorOnXZPlane ());
                setSide (localRotateForwardToSide (forward()));
            }
        };
    
    
} // namespace OpenSteer


// ----------------------------------------------------------------------------
#endif // OPENSTEER_SIMPLEVEHICLE_MB_H

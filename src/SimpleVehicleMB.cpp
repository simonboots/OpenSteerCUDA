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
// Note that SimpleVehicle is provided only as sample code.  Your application
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
// 03-31-09 sst: modified to use MemoryBackend
//
//
// ----------------------------------------------------------------------------


#include "OpenSteer/SimpleVehicleMB.h"
#include "OpenSteer/OpenSteerDemo.h"
#include <algorithm>


// ----------------------------------------------------------------------------
// serial numbers  (XXX should this be part of a "OpenSteerDemo vehicle mixin"?)


int OpenSteer::SimpleVehicleMB::serialNumberCounter = 0;


// ----------------------------------------------------------------------------
// constructor


OpenSteer::SimpleVehicleMB::SimpleVehicleMB (void)
{
    mb = MemoryBackend::instance();
    mb_id = mb->getNextID();
    
    // set inital state
    reset ();
    
    // maintain unique serial numbers
    serialNumber = serialNumberCounter++;
}


// ----------------------------------------------------------------------------
// destructor


OpenSteer::SimpleVehicleMB::~SimpleVehicleMB (void)
{
}


// ----------------------------------------------------------------------------
// adjust the steering force passed to applySteeringForce.
//
// allows a specific vehicle class to redefine this adjustment.
// default is to disallow backward-facing steering at low speed.
//
// xxx should the default be this ad-hocery, or no adjustment?
// xxx experimental 8-20-02
//
// parameter names commented out to prevent compiler warning from "-W"


OpenSteer::Vec3 
OpenSteer::SimpleVehicleMB::adjustRawSteeringForce (const Vec3& force,
                                                    const float /* deltaTime */)
{
    const float maxAdjustedSpeed = 0.2f * maxSpeed ();
    
    if ((speed () > maxAdjustedSpeed) || (force == Vec3::zero))
    {
        return force;
    }
    else
    {
        const float range = speed() / maxAdjustedSpeed;
        // const float cosine = interpolate (pow (range, 6), 1.0f, -1.0f);
        // const float cosine = interpolate (pow (range, 10), 1.0f, -1.0f);
        // const float cosine = interpolate (pow (range, 20), 1.0f, -1.0f);
        // const float cosine = interpolate (pow (range, 100), 1.0f, -1.0f);
        // const float cosine = interpolate (pow (range, 50), 1.0f, -1.0f);
        const float cosine = interpolate (pow (range, 20), 1.0f, -1.0f);
        return limitMaxDeviationAngle (force, cosine, forward());
    }
}


// ----------------------------------------------------------------------------
// xxx experimental 9-6-02
//
// apply a given braking force (for a given dt) to our momentum.
//
// (this is intended as a companion to applySteeringForce, but I'm not sure how
// well integrated it is.  It was motivated by the fact that "braking" (as in
// "capture the flag" endgame) by using "forward * speed * -rate" as a steering
// force was causing problems in adjustRawSteeringForce.  In fact it made it
// get NAN, but even if it had worked it would have defeated the braking.
//
// maybe the guts of applySteeringForce should be split off into a subroutine
// used by both applySteeringForce and applyBrakingForce?


void 
OpenSteer::SimpleVehicleMB::applyBrakingForce (const float rate, const float deltaTime)
{
    const float rawBraking = speed () * rate;
    const float clipBraking = ((rawBraking < maxForce ()) ?
                               rawBraking :
                               maxForce ());
    
    setSpeed (speed () - (clipBraking * deltaTime));
}


// ----------------------------------------------------------------------------
// apply a given steering force to our momentum,
// adjusting our orientation to maintain velocity-alignment.


void 
OpenSteer::SimpleVehicleMB::applySteeringForce (const Vec3& force,
                                                const float elapsedTime)
{
    
    const Vec3 adjustedForce = adjustRawSteeringForce (force, elapsedTime);
    
    // enforce limit on magnitude of steering force
    const Vec3 clippedForce = adjustedForce.truncateLength (maxForce ());
    
    // compute acceleration and velocity
    Vec3 newAcceleration = (clippedForce / mass());
    Vec3 newVelocity = velocity();
    
    // damp out abrupt changes and oscillations in steering acceleration
    // (rate is proportional to time step, then clipped into useful range)
    if (elapsedTime > 0)
    {
        const float smoothRate = clip (9 * elapsedTime, 0.15f, 0.4f);
        Vec3 tsmoothedAcceleration = smoothedAcceleration();
        blendIntoAccumulator (smoothRate,
                              newAcceleration,
                              tsmoothedAcceleration);
        resetSmoothedAcceleration(tsmoothedAcceleration);
    }
    
    // Euler integrate (per frame) acceleration into velocity
    newVelocity += smoothedAcceleration() * elapsedTime;
    
    // enforce speed limit
    newVelocity = newVelocity.truncateLength (maxSpeed ());
    
    // update Speed
    setSpeed (newVelocity.length());
    
    // Euler integrate (per frame) velocity into position
    setPosition (position() + (newVelocity * elapsedTime));
    
    // regenerate local space (by default: align vehicle's forward axis with
    // new velocity, but this behavior may be overridden by derived classes.)
    regenerateLocalSpace (newVelocity, elapsedTime);
}


// ----------------------------------------------------------------------------
// the default version: keep FORWARD parallel to velocity, change UP as
// little as possible.
//
// parameter names commented out to prevent compiler warning from "-W"


void 
OpenSteer::SimpleVehicleMB::regenerateLocalSpace (const Vec3& newVelocity,
                                                const float /* elapsedTime */)
{
    // adjust orthonormal basis vectors to be aligned with new velocity
    if (speed() > 0) regenerateOrthonormalBasisUF (newVelocity / speed());
}


// ----------------------------------------------------------------------------
// alternate version: keep FORWARD parallel to velocity, adjust UP according
// to a no-basis-in-reality "banking" behavior, something like what birds and
// airplanes do

// XXX experimental cwr 6-5-03


void 
OpenSteer::SimpleVehicleMB::regenerateLocalSpaceForBanking (const Vec3& newVelocity,
                                                          const float elapsedTime)
{
    // the length of this global-upward-pointing vector controls the vehicle's
    // tendency to right itself as it is rolled over from turning acceleration
    const Vec3 globalUp (0, 0.2f, 0);
    
    // acceleration points toward the center of local path curvature, the
    // length determines how much the vehicle will roll while turning
    const Vec3 accelUp = smoothedAcceleration() * 0.05f;
    
    // combined banking, sum of UP due to turning and global UP
    const Vec3 bankUp = accelUp + globalUp;
    
    // blend bankUp into vehicle's UP basis vector
    const float smoothRate = elapsedTime * 3;
    Vec3 tempUp = up();
    blendIntoAccumulator (smoothRate, bankUp, tempUp);
    setUp (tempUp.normalize());
    
    //  annotationLine (position(), position() + (globalUp * 4), gWhite);  // XXX
    //  annotationLine (position(), position() + (bankUp   * 4), gOrange); // XXX
    //  annotationLine (position(), position() + (accelUp  * 4), gRed);    // XXX
    //  annotationLine (position(), position() + (up ()    * 1), gYellow); // XXX
    
    // adjust orthonormal basis vectors to be aligned with new velocity
    if (speed() > 0) regenerateOrthonormalBasisUF (newVelocity / speed());
}

// ----------------------------------------------------------------------------
// draw lines from vehicle's position showing its velocity and acceleration


void 
OpenSteer::SimpleVehicleMB::annotationVelocityAcceleration (float maxLengthA, 
                                                          float maxLengthV)
{
    const float desat = 0.4f;
    const float aScale = maxLengthA / maxForce ();
    const float vScale = maxLengthV / maxSpeed ();
    const Vec3& p = position();
    const Vec3 aColor (desat, desat, 1); // bluish
    const Vec3 vColor (    1, desat, 1); // pinkish
    
    annotationLine (p, p + (velocity ()           * vScale), vColor);
    annotationLine (p, p + (smoothedAcceleration() * aScale), aColor);
}


// ----------------------------------------------------------------------------
// predict position of this vehicle at some time in the future
// (assumes velocity remains constant, hence path is a straight line)
//
// XXX Want to encapsulate this since eventually I want to investigate
// XXX non-linear predictors.  Maybe predictFutureLocalSpace ?
//
// XXX move to a vehicle utility mixin?


OpenSteer::Vec3 
OpenSteer::SimpleVehicleMB::predictFuturePosition (const float predictionTime) const
{
    return position() + (velocity() * predictionTime);
}


// ----------------------------------------------------------------------------

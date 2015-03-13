///////////////////////////////////////////////////////////////////////////////////////
//
//	Resolution.cpp
//	
//	Authors: Joshua Davis
//	Copyright 2010, DigiOen Institute of Technology
//
///////////////////////////////////////////////////////////////////////////////////////
#include "Precompiled.h"
#include "Resolution.h"
#include "Body.h"
#include "Physics.h"
#include <algorithm>
#include "DebugDraw.h"
#include "Manifold.h"

namespace Framework
{

  ContactSet::ContactSet()
  {
    IterationCount = 3;
  }

	BodyManifold * ContactSet::GetNextContact()
	{
		ErrorIf(NumberOfContacts==MaxContacts,"Maximum number of contacts reached. There is too many colliding objects in the simulation.");
		return &contactArray[NumberOfContacts++];
	}

	void ContactSet::Reset()
	{
		NumberOfContacts = 0;
	}

  void ResolveContactVelocityFull(BodyManifold& m, float dt)
  {
    /*The full impulse equation:
                           -(1 + e)*Dot(vRel,n)
      j = ---------------------------------------------------------
          M1^-1 + M2^-1 + I1^-1*cross(r1,n)^2 + I2^-1*cross(r2,n)^2

      where e is the restitution, vRel is the relative velocity of the points,
      M is the mass, I is the inertia and r1/r2 are the vector from the center
      of mass to the point of contact in the world space.
    */

    //Find the velocity of the two object along the contact normal
    float separatingVelocity = m.CalculateSeparatingVelocity();
    //if these objects are already moving apart, ignore this contact so
    //that we don't push it faster depending on how much they are penetrating
    if(separatingVelocity > 0.0f)
    {
      m.ContactImpulse = 0;
      return;
    }

    //get the mass of the contact along the normal
    float totalInvMass = m.GetContactMass(m.Normal);
    //calculate the numerator to the impulse equation
    float numerator = -(1.0f + m.Restitution) * separatingVelocity;    
    //calculate j (the impulse) in the direction of the normal
    float jNormal = numerator / totalInvMass;
    m.ContactImpulse = jNormal;
    //Apply the normal impulse to object 1 and 2
    Vec2 normalImpulse = jNormal * m.Normal;
    m.ApplyImpulse(0,-normalImpulse);
    m.ApplyImpulse(1,normalImpulse);


    //Note: the friction calculation is almost the exact same as the normal.
    //The only differences are that we use the tangent instead of the normal
    //direction, we have no restitution, and we have to take care of the
    //friction coefficients.

    
    //we don't actually have dynamic and static friction on our bodies,
    //but here's what we would do if we had separate values.
    //Note: dynamicFriction should be less than or equal to static friction.
    float staticFriction, dynamicFriction;
    staticFriction = dynamicFriction = m.FrictionCof;
    //get the tangent and the separating velocity in that direction.
    Vec2 tangent;
    float tangentVelocity = -m.CalculateTangentVelocity(tangent);
    //get the mass of the contact along the tangent
    totalInvMass = m.GetContactMass(tangent);
    //If the object falls perfectly down, it will have no tangent velocity.
    //Therefore, there is nothing to do and we should exit out.
    if(abs(tangentVelocity) < .001f)
      return;
    //calculate j (the impulse) in the direction of the tangent
    float jTangent = tangentVelocity / totalInvMass;

    Vec2 tangentImpulse;
    //We have just calculated the amount to stop the tangential velocity.
    //We need to make sure that kinetic friction (the normal impulse times the
    //friction coefficient) would not cause our object to go backwards. If it
    //would, we can just use the tangential impulse.
    //Otherwise, apply dynamic friction as normal.
    if(jTangent < jNormal * staticFriction)
      tangentImpulse = jTangent * tangent;
    else
      tangentImpulse = dynamicFriction * jNormal * tangent;
    m.ApplyImpulse(0,-tangentImpulse);
    m.ApplyImpulse(1,tangentImpulse);
  }

  void ResolvePenetrationFull(BodyManifold& m, float dt)
  {
    // The movement of each object is based on their inverse mass, so
    // total that.
    float totalInverseMass = m.Bodies[0]->InvMass + m.Bodies[1]->InvMass;
    // Add a slop factor to reduce jittering
    // (aka only resolve penetration above some threshold).
    float penetration = Max(m.Depth - 2.0f,0.0f) / totalInverseMass;
    Vec2 movePerIMass = m.Normal * penetration;

    // If stack stability can be increased by not resolving all the penetrations
    // in one step
    movePerIMass *= PHYSICS->PenetrationResolvePercentage;

    // Calculate the the movement amounts
    Vec2 movement0 = movePerIMass * -m.Bodies[0]->InvMass;
    Vec2 movement1 = movePerIMass * m.Bodies[1]->InvMass;

    // Apply the penetration resolution
    m.Bodies[0]->Position = m.Bodies[0]->Position + movement0;
    m.Bodies[1]->Position = m.Bodies[1]->Position + movement1;
  }

	//Resolve Positions
	void ContactSet::ResolvePositions(float dt)
	{
    for(unsigned int index = 0; index < NumberOfContacts; ++index)
      ResolvePenetrationFull(contactArray[index],dt);
	}

	//Resolve Velocities of all contacts
	void ContactSet::ResolveVelocities(float dt)
	{
    //This is an iterative solver. That means we do several passes over
    //all of the data so that we can approach the correct answer. Also,
    //each iteration propagates energy. This means we can get a line of
    //billiards to propagate energy to the end in one frame with enough
    //iterations.
    for(unsigned int i = 0; i < IterationCount; ++i)
      for(unsigned int index = 0; index < NumberOfContacts; ++index)
        ResolveContactVelocityFull(contactArray[index],dt);
	}

	void ContactSet::ResolveContacts(float dt)
	{
		this->ResolveVelocities(dt);
    this->ResolvePositions(dt);
	}

}
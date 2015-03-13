///////////////////////////////////////////////////////////////////////////////////////
//
//	ContactConstraint.cpp
//  A non-penetration contact constraint.
//	
//	Authors: Joshua Davis
//	Copyright 2011, DigiPen Institute of Technology
//
///////////////////////////////////////////////////////////////////////////////////////
#include "Precompiled.h"

#include "ContactConstraint.h"
#include "Body.h"
#include "DebugDraw.h"

namespace Framework
{

  /*A contact constraint is known as a non-penetration constraint. 
    The position constraint is of the form C: penetration = 0.
    We can derive the velocity constraint as follows:
    C: dot(p2 - p1,n) = 0
    C: dot(c2 + r2 - c1 - r1,n) = 0
    Cdot: dot(v2 + cross(w2,r2) - v1 - cross(w1,r1),n) = 0
    Cdot: dot(v2,n) + dot(cross(w2,r2),n) - dot(v1,n) - dot(cross(w1,r1),n)
    Cdot: dot(v2,n) + dot(cross(r2,n),w2) - dot(v1,n) - dot(cross(r1,n),w1)
    J: [-n, -cross(r1,n), n, cross(r2,n)]
    Identity used:    a x b * c =     a * b x c     = c x a * b
              dot(cross(a,b),c) = dot(a,cross(b,c)) = dot(cross(c,a),b)
  */
  ContactConstraint::ContactConstraint()
  {
    
  }

  ContactConstraint::~ContactConstraint()
  {

  }

  void ContactConstraint::Set(BodyManifold* contact)
  {
    ContactPoint = *contact;
    Constraint::SetBodies(contact->Bodies[0],contact->Bodies[1]);
    ContactPoint.ContactImpulse = 0;
    ContactPoint.TangentImpulse = 0;
  }

  void ContactConstraint::Update(float dt)
  {
    Vec2& normal = ContactPoint.Normal;
    //the jacobian for the normal is ( -n, -r1 x n, n, r2 x n)
    NormalJacobian.Set(-normal,-Cross2D(ContactPoint.WorldRs[0],normal),
                        normal, Cross2D(ContactPoint.WorldRs[1],normal));

    //the jacobian for the tangent is ( -t, -r1 x t, t, r2 x t)
    Vec2 tangent = -TangentVector(normal);
    TangentJacobian.Set(-tangent,-Cross2D(ContactPoint.WorldRs[0],tangent),
                         tangent, Cross2D(ContactPoint.WorldRs[1],tangent));

    //compute the effective mass for the normal (J * M^-1 * J^T)
    NormalMass = CalculateEffectiveMass(NormalJacobian);
    //compute the effective mass for the tangent (J * M^-1 * J^T)
    TangentMass = CalculateEffectiveMass(TangentJacobian);
    Valid = false;

    //we need to add energy to the system to correct penetration.
    NormalBias = -ContactPoint.Depth;
    //we can also add restitution by adding energy based upon the separating velocity
    ConstraintVelocity velocity;
    velocity.Set(Bodies[0],Bodies[1]);
    float relativeVel = CalculateJV(NormalJacobian,velocity);
    if(relativeVel < -20.0f)
      NormalBias += ContactPoint.Restitution * relativeVel;
  }

  void ContactConstraint::SolveIteration(float dt)
  {
    ConstraintVelocity velocities;
    //get the current velocities
    velocities.Set(ContactPoint.Bodies[0],ContactPoint.Bodies[1]);

    //calculate -(jv + b) / (effectiveMass)
    float jv = CalculateJV(NormalJacobian,velocities);
    float lambda = -(jv + NormalBias) / NormalMass;

    //our clamp bounds is [0,+infinity]
    float oldImpulse = ContactPoint.ContactImpulse;
    float newImpulse = Max(oldImpulse + lambda, 0);
    lambda = newImpulse - oldImpulse;
    ContactPoint.ContactImpulse = newImpulse;
    //apply the clamped impulse
    ApplyConstraintImpulse(NormalJacobian,lambda);


    //get the newly changed velocities
    velocities.Set(ContactPoint.Bodies[0],ContactPoint.Bodies[1]);
    //calculate -(jv + b) / (effectiveMass)
    jv = CalculateJV(TangentJacobian,velocities);
    lambda = -(jv) / TangentMass;
    float maxFriction = ContactPoint.FrictionCof * ContactPoint.ContactImpulse;

    //We are setting static friction and dynamic friction to be equal.
    //According to physics, the max force that friction can apply is
    //bound by the normal force.
    //Therefore, we can set our bounds to be [-mu * jNormal,mu * jNormal]
    oldImpulse = ContactPoint.TangentImpulse;
    newImpulse = Clamp(oldImpulse + lambda, -maxFriction,maxFriction);
    lambda = newImpulse - oldImpulse;
    ContactPoint.TangentImpulse = newImpulse;
    //apply the clamped impulse
    ApplyConstraintImpulse(TangentJacobian,lambda);
  }

}

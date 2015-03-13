///////////////////////////////////////////////////////////////////////////////////////
//
//	Manifold.cpp
//  A manifold is used to represent the collision data of two objects.
//	
//	Authors: Joshua Davis
//	Copyright 2011, DigiPen Institute of Technology
//
///////////////////////////////////////////////////////////////////////////////////////
#include "Precompiled.h"
#include "Manifold.h"
#include "Body.h"
#include "Physics.h"

namespace Framework
{

  //Functions that determine how restitution and friction 
  //are determined between body pairs.

  //There really is no physically accurate way to mathematically combine 
  //these factors. A material does not have a single friction or restitution.
  //The physically accurate way is to have a large database that
  //define the relationship between all physical materials or simulate 
  //molecular physics.

  float DetermineRestitution(Body * a,Body * b)
  {
    return 	std::min(a->Restitution,b->Restitution);
  }

  float DetermineFriction(Body * a,Body * b)
  {
    return sqrt(a->Friction*b->Friction);
  }

  BodyManifold::BodyManifold()
  {
    ContactImpulse = 0.0f;
    TangentImpulse = 0.0f;
  }

  void BodyManifold::Set(Manifold* manifold, Body* body0, Body* body1)
  {
    //while the correct solution would be to use multiple points of contact
    //for solving, we're just going to use one by averaging the points of contact.
    
    Bodies[0] = body0;
    Bodies[1] = body1;
    ContactImpulse = 0.0f;
    TangentImpulse = 0.0f;
    Normal = manifold->Normal;
    Restitution = DetermineRestitution(body0,body1);
    FrictionCof = DetermineFriction(body0,body1);

    Vec2 worldPoint(0.0f,0.0f);
    Depth = 0.0f;
    for(uint i = 0; i < manifold->PointCount; ++i)
    {
      Manifold::ContactPoint& point = manifold->PointAt(i);
      worldPoint += (point.Points[0] + point.Points[1]) * .5f;
      Depth += point.Depth;
    }
    float pointCount = (float)manifold->PointCount;
    worldPoint /= pointCount;
    Depth /= pointCount;

    // get the vector from the center to the point of contact
    WorldRs[0] = worldPoint - body0->Position;
    WorldRs[1] = worldPoint - body1->Position;
    
  }

  float BodyManifold::CalculateSeparatingVelocity()
  {
    // the separating velocity of two points is the difference of each
    // point's velocity along the direction of the normal.
    Vec2 point1Vel = Bodies[0]->GetPointVelocity(WorldRs[0]);
    Vec2 point2Vel = Bodies[1]->GetPointVelocity(WorldRs[1]);

    return Dot(Normal,point2Vel - point1Vel);
  }

  float BodyManifold::CalculateTangentVelocity(Vec2& tangent)
  {
    // This is the same as the normal separating velocity, however we
    // have to compute a tangent direction. We can make a tangent vector
    // by removing the component in the direction of the normal from the
    // separating velocity.
    Vec2 point1Vel = Bodies[0]->GetPointVelocity(WorldRs[0]);
    Vec2 point2Vel = Bodies[1]->GetPointVelocity(WorldRs[1]);
    Vec2 relativeVel = point2Vel - point1Vel;

    tangent = relativeVel - Dot(relativeVel,Normal) * Normal;
    float length = Normalize(tangent);
    return length;
  }

  float BodyManifold::GetMassTerm(uint bodyIndex, Vec2Param axis)
  {
    // performing  I^-1*cross(r,n)^2

    //The mass term is a "weighted average" of the two objects.
    //This is not the most intuitive value due to inertia.
    float invMass = Bodies[bodyIndex]->InvMass;
    float invInertia = Bodies[bodyIndex]->InvInertia;
    //The inertia measures how easy it is to rotate a point on the object.
    float inertia = Cross2D(WorldRs[bodyIndex],axis);
    inertia *= inertia; 
    inertia *= invInertia;
    return inertia + invMass;
  }

  float BodyManifold::GetContactMass(Vec2Param axis)
  {
    float obj1Mass = GetMassTerm(0,axis);
    float obj2Mass = GetMassTerm(1,axis);
    return obj1Mass + obj2Mass;
  }

  void BodyManifold::ApplyImpulse(int bodyIndex, Vec2Param impulse)
  {
    //Apply the impulse taking into account the inverse
    //mass so that larger objects will move less.
    Body& body = *(Bodies[bodyIndex]);
    body.Velocity += impulse * body.InvMass;
    float torque = Cross2D(WorldRs[bodyIndex],impulse);
    body.AngularVelocity += torque * body.InvInertia;
  }

}

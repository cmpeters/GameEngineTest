///////////////////////////////////////////////////////////////////////////////////////
///
///	\file Manifold.h
///	A manifold is used to represent the collision data of two objects.
///	
///	Authors:  Joshua Davis
///	Copyright 2011, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////////////
#pragma once //Makes sure this header is only included once

#include "VMath.h"
#include "Engine.h"
#include "Intersection.h"

namespace Framework
{
  class Body;

  // A manifold stores the collision data between two objects.
  struct BodyManifold
  {
    BodyManifold();
    void Set(Manifold* manifold, Body* body0, Body* body1);

    // The normal of the collision.
    Vec2 Normal;
    // The amount of overlap between the two objects.
    float Depth;
    // The vectors from the center of each object to the point of contact.
    Vec2 WorldRs[2];
    
    Body* Bodies[2];
    float ContactImpulse;
    float TangentImpulse;

    float Restitution;
    float FrictionCof;
    // Gets the separating velocity in the direction of the normal
    float CalculateSeparatingVelocity();
    // Get the separating velocity in the direction of the tangent
    float CalculateTangentVelocity(Vec2& tangent);
    // Get the contact mass of the body at the given index
    // in the direction of the axis passed in.
    float GetMassTerm(uint bodyIndex, Vec2Param axis);
    // Get the contact mass of both objects with the given axis.
    float GetContactMass(Vec2Param axis);
    // Apply the given impulse on the body at the given index.
    void ApplyImpulse(int bodyIndex, Vec2Param impulse);
  };

}

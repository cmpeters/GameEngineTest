///////////////////////////////////////////////////////////////////////////////////////
//
//	Constraint.h
//  Base Constraint object.
//	
//	Authors: Joshua Davis
//	Copyright 2011, DigiPen Institute of Technology
//
///////////////////////////////////////////////////////////////////////////////////////
#pragma once

#include "VMath.h"
#include "Resolution.h"

namespace Framework
{

  ///The Jacobian is a structure that tells in what direction
  ///the constraint needs to be solved. It is used in the calculation
  ///to get the magnitude of the impulse to apply.
  struct Jacobian
  {
    ///Used for a standard jacobian between two bodies.
    void Set(Vec2Param linear1, float angular1, Vec2Param linear2, float angular2);
    ///Used for a specialized jacobian between one body and the world.
    void Set(Vec2Param linear1, float angular1);

    Vec2 Linear1,Linear2;
    float Angular1,Angular2;
  };

  ///Just a simple structure to wrap two object's velocities.
  struct ConstraintVelocity
  {
    ///Used for a standard constraint between two bodies.
    void Set(Body* body1, Body* body2);
    ///Used for a constraint between one body and the world.
    void Set(Body* body1);

    Vec2 V1,V2;
    float W1,W2;
  };

  // Base constraint class. Also contains functions that
  // are helpful for all constraint types.
  class Constraint
  {
  public:
    Constraint();
    virtual ~Constraint();

    virtual void Update(float dt) = 0;
    virtual void SolveIteration(float dt) = 0;

    void SetBodies(Body* body1, Body* body2);
    void ApplyConstraintImpulse(Jacobian& jacobian, float impulseMagnitude);
    float CalculateJV(Jacobian& Jacobian, ConstraintVelocity& velocities);
    float CalculateEffectiveMass(Jacobian& jacobian);

    //Linked list Nodes
    Constraint * Next;
    Constraint * Prev;

  protected:
    friend class ConstraintSolver;
    bool Valid;
    float MaxForce;
    Body* Bodies[2];
  };
  
}

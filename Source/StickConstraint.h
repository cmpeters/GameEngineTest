///////////////////////////////////////////////////////////////////////////////////////
//
//	StickConstraint.h
//  A distance constraint. Attempts to keep two points
//  on two objects a fixed distance apart.
//	
//	Authors: Joshua Davis
//	Copyright 2011, DigiPen Institute of Technology
//
///////////////////////////////////////////////////////////////////////////////////////
#pragma once

#include "Constraint.h"

namespace Framework
{

  ///A constraint that enforces a fixed distance
  ///between two points on two objects.
  class StickConstraint : public Constraint
  {
  public:
    StickConstraint();
    virtual ~StickConstraint();

    virtual void Update(float dt);
    virtual void SolveIteration(float dt);

    void SetBodyPoints(Vec2Param body1Point, Vec2Param body2Point);
    void SetDistance(float distance);

  private:
    // We can cache the mass that the constraint feels since it
    // doesn't change during each iteration of one frame.
    float EffectiveMass;
    // Used to make sure we clamp our total impulse correctly.
    float AccumulatedImpulse;
    // The local points that the stick is attached to on each object.
    Vec2 BodyRs[2];
    Jacobian StickJacobian;
    // The desired distance of the stick.
    float Distance;
    float Bias;
  };

}

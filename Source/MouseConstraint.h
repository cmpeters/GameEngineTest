///////////////////////////////////////////////////////////////////////////////////////
//
//	MouseConstraint.h
//  A distance constraint with the world. Attempts to make a point on an object
//  move to a world point.
//	
//	Authors: Joshua Davis
//	Copyright 2011, DigiPen Institute of Technology
//
///////////////////////////////////////////////////////////////////////////////////////
#pragma once

#include "Constraint.h"

namespace Framework
{

  ///A constraint that enforces a fixed distance a point on
  ///an object and a point in the world. Just a specialized stick
  ///constraint.
  class MouseConstraint : public Constraint
  {
  public:
    MouseConstraint();
    virtual ~MouseConstraint();

    virtual void Update(float dt);
    virtual void SolveIteration(float dt);

    void SetBody(Body* body);
    void SetBodyPoint(Vec2Param bodyPoint);
    void SetWorldPoint(Vec2Param worldPoint);
    void SetTarget(Vec2Param target);
    void SetDistance(float distance);

  private:
    float EffectiveMass;
    float AccumulatedImpulse;
    Vec2 BodyR,Target;
    Jacobian StickJacobian;
    float Distance;
    float Bias;
  };

}

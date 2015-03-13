///////////////////////////////////////////////////////////////////////////////////////
//
//	MouseConstraint.cpp
//  A distance constraint with the world. Attempts to make a point on an object
//  move to a world point.
//	
//	Authors: Joshua Davis
//	Copyright 2011, DigiPen Institute of Technology
//
///////////////////////////////////////////////////////////////////////////////////////
#include "Precompiled.h"

#include "MouseConstraint.h"
#include "Body.h"
#include "DebugDraw.h"

namespace Framework
{

  MouseConstraint::MouseConstraint()
  {
    EffectiveMass = 0;
    AccumulatedImpulse = 0;
    Distance = 2;
    Bias = 0;
  }

  MouseConstraint::~MouseConstraint()
  {

  }

  void MouseConstraint::Update(float dt)
  {
    //Bring the vector from the objects center to the connection point
    //from body space (where it doesn't change) to world space.
    Vec2 worldR1 = Bodies[0]->GetWorldOffsetFromBodyPoint(BodyR);
    Vec2 worldPoint1 = worldR1 + Bodies[0]->Position;

    Vec2 p2p1 = Target - worldPoint1;
    float distance = Normalize(p2p1);
    Bias = .5f * distance * distance - Distance * Distance;
    //the jacobian for is ( -d, -r1 x d, 0, 0)
    StickJacobian.Set(-p2p1,-Cross2D(worldR1,p2p1));

    float linearMass1 = Dot(StickJacobian.Linear1, StickJacobian.Linear1) * Bodies[0]->InvMass;
    float angularMass1 = StickJacobian.Angular1 * StickJacobian.Angular1 * Bodies[0]->InvInertia;
    EffectiveMass = linearMass1 + angularMass1;
    ErrorIf(EffectiveMass == 0.0f,"Constraint is connected to an object of infinite mass. Cannot grab an infinite mass object with a mouse constraint.");
    EffectiveMass = EffectiveMass;


    Drawer::Instance.DrawSegment( worldPoint1 , Target );
  }

  void MouseConstraint::SolveIteration(float dt)
  {
    ConstraintVelocity velocities;
    //get the current velocities
    velocities.Set(Bodies[0]);

    //calculate -(jv + b) / (effectiveMass)
    float jv = CalculateJV(StickJacobian,velocities);
    float lambda = -(jv + Bias) / EffectiveMass;

    //Clamp our total impulse between our max allowable impulse amount.
    float oldImpulse = AccumulatedImpulse;
    AccumulatedImpulse = Clamp(oldImpulse + lambda, -MaxForce, MaxForce);
    lambda = AccumulatedImpulse - oldImpulse;
    //apply the clamped impulse
    Bodies[0]->Velocity += StickJacobian.Linear1 * lambda * Bodies[0]->InvMass;
    Bodies[0]->AngularVelocity += StickJacobian.Angular1 * lambda * Bodies[0]->InvInertia;
  }

  void MouseConstraint::SetBody(Body* body)
  {
    Bodies[0] = body;
  }

  void MouseConstraint::SetBodyPoint(Vec2Param bodyPoint)
  {
    BodyR = bodyPoint;
  }

  void MouseConstraint::SetWorldPoint(Vec2Param worldPoint)
  {
    BodyR = Bodies[0]->GetBodyPointFromWorldPoint(worldPoint);
  }

  void MouseConstraint::SetTarget(Vec2Param target)
  {
    Target = target;
  }

  void MouseConstraint::SetDistance(float distance)
  {
    Distance = distance;
  }
}

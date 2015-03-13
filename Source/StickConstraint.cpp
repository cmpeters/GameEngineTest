///////////////////////////////////////////////////////////////////////////////////////
//
//	StickConstraint.cpp
//  A distance constraint. Attempts to keep two points
//  on two objects a fixed distance apart.
//	
//	Authors: Joshua Davis
//	Copyright 2011, DigiPen Institute of Technology
//
///////////////////////////////////////////////////////////////////////////////////////
#include "Precompiled.h"

#include "StickConstraint.h"
#include "Body.h"
#include "DebugDraw.h"

namespace Framework
{

  /*
    C   : .5(p2 - p1)^2 - L^2 = 0
    cDot: (p2 - p1) * (v2 + cross(w2,r2) - v1 - cross(w1,r1) = 0
    Let : d = p2 - p1
    cDot: dot(d,v2) + dot(d,cross(w2,r2)) - dot(d,v1) - dot(d,cross(w1,r1)) = 0
    cDot: dot(d,v2) + dot(w2,cross(r2,d)) - dot(d,v1) - dot(w1,cross(r1,d)) = 0
    J   : [-d,-cross(r1,d),d,cross(r2,d)]
    Identity used:    a x b * c =     a * b x c     = c x a * b
              dot(cross(a,b),c) = dot(a,cross(b,c)) = dot(cross(c,a),b)
*/
  StickConstraint::StickConstraint() 
  {
    EffectiveMass = 0;
    AccumulatedImpulse = 0;
    Distance = 2;
    Bias = 0;
  }

  StickConstraint::~StickConstraint()
  {

  }

  void StickConstraint::Update(float dt)
  {
    //Bring the vector from the objects center to the connection point
    //from body space (where it doesn't change) to world space.
    Vec2 worldR1 = Bodies[0]->GetWorldOffsetFromBodyPoint(BodyRs[0]);
    Vec2 worldR2 = Bodies[1]->GetWorldOffsetFromBodyPoint(BodyRs[1]);
    Vec2 worldPoint1 = worldR1 + Bodies[0]->Position;
    Vec2 worldPoint2 = worldR2 + Bodies[1]->Position;

    Vec2 p2p1 = worldPoint2 - worldPoint1;
    float distance = Normalize(p2p1);
    Bias = .5f * distance * distance - Distance * Distance;
    //the jacobian is ( -d, -r1 x d, d, r2 x d)
    StickJacobian.Set(-p2p1,-Cross2D(worldR1,p2p1),
                       p2p1, Cross2D(worldR2,p2p1));
    EffectiveMass = CalculateEffectiveMass(StickJacobian);


    Drawer::Instance.DrawSegment( worldPoint1 , worldPoint2 );
  }

  void StickConstraint::SolveIteration(float dt)
  {
    ConstraintVelocity velocities;
    //get the current velocities
    velocities.Set(Bodies[0],Bodies[1]);

    //calculate -(jv + b) / (effectiveMass)
    float jv = CalculateJV(StickJacobian,velocities);
    float lambda = -(jv + Bias) / EffectiveMass;

    //Clamp our total impulse between our max allowable impulse amount.
    float oldImpulse = AccumulatedImpulse;
    AccumulatedImpulse = Clamp(oldImpulse + lambda, -MaxForce, MaxForce);
    lambda = AccumulatedImpulse - oldImpulse;
    //apply the clamped impulse
    ApplyConstraintImpulse(StickJacobian,lambda);
  }

  void StickConstraint::SetBodyPoints(Vec2Param body1Point, Vec2Param body2Point)
  {
    BodyRs[0] = body1Point;
    BodyRs[1] = body2Point;
  }

  void StickConstraint::SetDistance(float distance)
  {
    Distance = distance;
  }
}

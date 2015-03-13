///////////////////////////////////////////////////////////////////////////////////////
//
//	Constraint.cpp
//  Base Constraint object.
//	
//	Authors: Joshua Davis
//	Copyright 2011, DigiPen Institute of Technology
//
///////////////////////////////////////////////////////////////////////////////////////
#include "Precompiled.h"

#include "Constraint.h"
#include "Body.h"
#include "DebugDraw.h"

/* In solving a constraint there are several knowns and one unknown.
   Each constraint will have a unique Jacobian (or multiple Jacobians) that
   is known ahead of time. The Jacobian is the direction of the impulse to
   apply. What we don't have is the impulse magnitude (lambda).
   The impulse magnitude for an individual constraint can be solved via the formula
   lambda = -(JV + b) / (J * M^-1 * J^T)
   Where:
      J is [L1 A1 L2 A2],
      V is [V1 W1 V2 W2],
      b is bias term defined as -baumgarte * ConstraintError
      and M^-1 is the mass matrix [M1^-1   0     0     0  ]
                                  [  0   I1^-1   0     0  ]
                                  [  0     0   M2^-1   0  ]
                                  [  0     0     0   I2^-1]
   The new velocities can then be found through the formula:
     V = V + lambda * M^-1 * J^T

   This only solves one constraint though. In order to calculate the solution
   to the entire system of constraints, we solve iteratively. This is done
   by solving each constraint one after another, which will converge to a
   correct global solution given enough time.

   To converge quicker, warm starting can be used (although left out
   for this example). Warm starting is just applying a first guess to
   reduce the number of iterations required to reach a global solution.

   For more details on implementing a more robust constraint solver, see
   the constraint slides on the physics club page on distance.
*/

namespace Framework
{

  void Jacobian::Set(Vec2Param linear1, float angular1, Vec2Param linear2, float angular2)
  {
    Linear1 = linear1;
    Angular1 = angular1;
    Linear2 = linear2;
    Angular2 = angular2;
  }

  void Jacobian::Set(Vec2Param linear1, float angular1)
  {
    Linear1 = linear1;
    Angular1 = angular1;
    Linear2 = Vec2(0,0);
    Angular2 = 0;
  }

  void ConstraintVelocity::Set(Body* body1, Body* body2)
  {
    V1 = body1->Velocity;
    W1 = body1->AngularVelocity;
    V2 = body2->Velocity;
    W2 = body2->AngularVelocity;
  }

  void ConstraintVelocity::Set(Body* body1)
  {
    V1 = body1->Velocity;
    W1 = body1->AngularVelocity;
    V2 = Vec2(0,0);
    W2 = 0;
  }

  Constraint::Constraint() 
  { 
    Valid = true; 
    MaxForce = PositiveMax();
  }

  Constraint::~Constraint()
  {

  }

  void Constraint::SetBodies(Body* body1, Body* body2)
  {
    Bodies[0] = body1;
    Bodies[1] = body2;
  }

  void Constraint::ApplyConstraintImpulse(Jacobian& jacobian, float impulseMagnitude)
  {
    //The resultant impulse is the magnitude to apply while the jacobian is the direction.
    //Therefore, we can just apply after multiplying by the inverse mass.
    //[V1] = [lambda] * [M1^-1   0     0     0  ] * [L1]
    //[W1] = [lambda] * [  0   I1^-1   0     0  ] * [A1]
    //[V2] = [lambda] * [  0     0   M2^-1   0  ] * [L2]
    //[W2] = [lambda] * [  0     0     0   I2^-1] * [A2]
    Bodies[0]->Velocity += jacobian.Linear1 * impulseMagnitude * Bodies[0]->InvMass;
    Bodies[0]->AngularVelocity += jacobian.Angular1 * impulseMagnitude * Bodies[0]->InvInertia;
    Bodies[1]->Velocity += jacobian.Linear2 * impulseMagnitude * Bodies[1]->InvMass;
    Bodies[1]->AngularVelocity += jacobian.Angular2 * impulseMagnitude * Bodies[1]->InvInertia;
  }

  float Constraint::CalculateJV(Jacobian& Jacobian, ConstraintVelocity& velocities)
  {
    //J*V is just the "Dot product" of the two vectors [L1 A1 L2 A2] and [V1 W1 V2 W2]
    float linear = Dot(Jacobian.Linear1,velocities.V1) + Dot(Jacobian.Linear2,velocities.V2);
    float angular = Jacobian.Angular1 * velocities.W1 + Jacobian.Angular2 * velocities.W2;
    return linear + angular;
  }

  float Constraint::CalculateEffectiveMass(Jacobian& jacobian)
  {
    //The effective mass is a weighted average of masses signifying
    //how much can be applied in the linear and angular on each object.

    //The formula is (J * M^-1 * J^T) which when expanded is
    //[L1, A1, L2, A2] * [M1^-1   0     0     0  ] [L1]
    //                   [  0   I1^-1   0     0  ] [A1] 
    //                   [  0     0   M2^-1   0  ] [L2]
    //                   [  0     0     0   I2^-1] [A2]
    //can be simplified to L1^2 * M1^-1 + A1^2 * I1^-1 + L2^2 * M2^-1 + A2^2 * I2^-1
    float linearMass1 = Dot(jacobian.Linear1, jacobian.Linear1) * Bodies[0]->InvMass;
    float linearMass2 = Dot(jacobian.Linear2, jacobian.Linear2) * Bodies[1]->InvMass;
    float angularMass1 = jacobian.Angular1 * jacobian.Angular1 * Bodies[0]->InvInertia;
    float angularMass2 = jacobian.Angular2 * jacobian.Angular2 * Bodies[1]->InvInertia;
    float totalMass = linearMass1 + linearMass2 + angularMass1 + angularMass2;
    ErrorIf(totalMass == 0.0f,"Constraint is connected to two objects of infinite mass. Cannot connect two infinite mass objects.");
    return totalMass;
  }

}

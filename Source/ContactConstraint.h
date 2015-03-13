///////////////////////////////////////////////////////////////////////////////////////
//
//	ContactConstraint.h
//  A non-penetration contact constraint.
//	
//	Authors: Joshua Davis
//	Copyright 2011, DigiPen Institute of Technology
//
///////////////////////////////////////////////////////////////////////////////////////
#pragma once

#include "Constraint.h"

namespace Framework
{

  ///A non-penetration constraint, also known as a contact.
  ///This constraint resolves the collision between two objects.
  class ContactConstraint : public Constraint
  {
  public:
    ContactConstraint();
    virtual ~ContactConstraint();

    void Set(BodyManifold* contact);

    virtual void Update(float dt);
    virtual void SolveIteration(float dt);

  private:
    friend class Physics;

    float NormalMass, TangentMass;
    BodyManifold ContactPoint;
    Jacobian NormalJacobian;
    Jacobian TangentJacobian;
    float NormalBias;
  };

}

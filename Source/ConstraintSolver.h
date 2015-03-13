///////////////////////////////////////////////////////////////////////////////////////
///
///	\file ConstraintSolver.h
///	Sequential Impulse Constraint Solver system.
///	
///	Authors: Josh Davis
///	Copyright 2011, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////////////
#pragma once

#include "Constraint.h"
#include "ContactConstraint.h"
#include "StickConstraint.h"
#include "MouseConstraint.h"

namespace Framework
{
 
  ///An iterative impulse constraint solver based
  //upon Erin Catto's work with box2D.
  class ConstraintSolver
  {
  public: 

    ConstraintSolver();
    ~ConstraintSolver();

    void Clear();
    void ClearContacts();

    void AddContact(BodyManifold* contact);
    void AddConstraint(Constraint* constraint);
    void RemoveConstraint(Constraint* constraint);
    void RemoveConstraintsWithBody(Body* body);

    void Solve(float dt);
  private:
    void Update(float dt);
    void WarmStart(float dt);
    void SolveIteration(float dt);

    typedef ObjectLinkList<Constraint> ConstraintList;
    typedef ConstraintList::iterator ConstraintIterator;
    ConstraintList Constraints;
    unsigned int IterationCount;

    friend class Physics;
    static const int MaxContacts = 1024;
    ContactConstraint contactArray[MaxContacts];
    unsigned NumberOfContacts;
  };

}

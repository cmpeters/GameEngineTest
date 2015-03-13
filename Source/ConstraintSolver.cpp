#include "Precompiled.h"

#include "ConstraintSolver.h"

namespace Framework
{

  ConstraintSolver::ConstraintSolver()
  {
    IterationCount = 15;
  }

  ConstraintSolver::~ConstraintSolver()
  {
    Clear();
  }

  void ConstraintSolver::Clear()
  {
    ConstraintIterator constraintStart = Constraints.begin();
    ConstraintIterator constraintEnd = Constraints.end();
    while(constraintStart != constraintEnd)
    {
      ConstraintList::iterator c = constraintStart;
      ++constraintStart;
      Constraints.erase(c);
      Constraint* constraint = &*c;
      delete constraint;
    }
  }

  void ConstraintSolver::ClearContacts()
  {
    NumberOfContacts = 0;
  }

  void ConstraintSolver::AddContact(BodyManifold* contact)
  {
    ErrorIf(NumberOfContacts==MaxContacts,"Maximum number of contacts reached. There is too many colliding objects in the simulation.");
    ContactConstraint& contactConstraint = contactArray[NumberOfContacts];
    ++NumberOfContacts;

    contactConstraint.Set(contact);
  }

  void ConstraintSolver::AddConstraint(Constraint* constraint)
  {
    Constraints.push_back(constraint);
  }

  void ConstraintSolver::RemoveConstraint(Constraint* constraint)
  {
    Constraints.erase(constraint);
  }

  void ConstraintSolver::RemoveConstraintsWithBody(Body* body)
  {
    //we need to find all constraints where one of the two
    //objects connected were the body passed in
    ConstraintIterator start = Constraints.begin();
    ConstraintIterator end = Constraints.end();
    while(start != end)
    {
      ConstraintList::iterator c = start;
      ++start;
      if(c->Bodies[0] == body || c->Bodies[1] == body)
      {
        Constraints.erase(c);
        Constraint* constraint = &*c;
        delete constraint;
      }
    }
  }

  void ConstraintSolver::Solve(float dt)
  {
    Update(dt);
    WarmStart(dt);
    //This solver is iterative, that means it takes several full iterations
    //over the entire set to converge to a correct answer.
    for(unsigned int i = 0; i < IterationCount; ++i)
      SolveIteration(dt);
  }

  void ConstraintSolver::Update(float dt)
  {
    //first we need to update all of the constraints.
    //This involves calculating non changing values.
    ConstraintIterator start = Constraints.begin();
    ConstraintIterator end = Constraints.end();
    for(; start != end; ++start)
      start->Update(dt);

    for(unsigned int i = 0; i < NumberOfContacts; ++i)
      contactArray[i].Update(dt);
  }

  void ConstraintSolver::WarmStart(float dt)
  {
    //Warm starting has been left out for simplicity. Warm starting
    //is the process of applying your best guess up front so it takes less
    //iterations to achieve good results. What's a good guess to use? How
    //about the end result of last frame!
    //In order to have a robust constraint solver, this must be added!!!
  }

  void ConstraintSolver::SolveIteration(float dt)
  {
    //Note: we iterate through all constraints fully before the next iteration.
    ConstraintIterator start = Constraints.begin();
    ConstraintIterator end = Constraints.end();
    for(; start != end; ++start)
      start->SolveIteration(dt);

    for(unsigned int i = 0; i < NumberOfContacts; ++i)
      contactArray[i].SolveIteration(dt);
  }
}

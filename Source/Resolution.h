///////////////////////////////////////////////////////////////////////////////////////
///
///	\file Resolution.h
///	Iterative impulse collision resolution system.
///	
///	Authors: Joshua Davis
///	Copyright 2010, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////////////
#pragma once
#include "Collision.h"

namespace Framework
{
	///A Set of contacts that need to be resolved.
	class ContactSet
	{
	public:
    ContactSet();

		BodyManifold * GetNextContact();
		void ResolveContacts(float dt);
		void Reset();
	private:
		void ResolveVelocities(float dt);
		void ResolvePositions(float dt);

    friend class Physics;
    static const int MaxContacts = 1024;
    unsigned int IterationCount;
    BodyManifold contactArray[MaxContacts];
    unsigned NumberOfContacts;
	};

}
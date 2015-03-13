///////////////////////////////////////////////////////////////////////////////////////
///
///	\file Body.h  Define Body GameComponent
///	
///	Authors: Chris Peters
///	Copyright 2010, Digipen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////////////
#pragma once //Makes sure this header is only included once
#include "Composition.h"
#include "VMath.h"
#include "Collision.h"

namespace Framework
{
	///Body Component provides basic point physics dynamics including mass, 
	///velocity, forces, acceleration, and collision resolution.
	///Component will modify transform component attributes every frame.
	class Body : public GameComponent
	{
	public:
		Body();
		~Body();

		void AddForce(Vec2Param force);
		void Integrate(float dt);
		void SetPosition(Vec2Param);
		void SetVelocity(Vec2Param);
		void PublishResults();

    Vec2 GetBodyPointFromWorldPoint(Vec2Param worldPoint);
    Vec2 GetWorldPointFromBodyPoint(Vec2Param bodyPoint);
    Vec2 GetWorldOffsetFromBodyPoint(Vec2Param bodyPoint);
    Vec2 GetPointVelocity(Vec2Param pointOffset);

		//Draw the object using the debug drawer
		void DebugDraw();

		virtual void Initialize();
		virtual void Serialize(ISerializer& stream);

		Body * Next;
		Body * Prev;

		Vec2 Position;
		Vec2 PrevPosition;
    float Rotation;
		Vec2 Velocity;
    float AngularVelocity;
		Vec2 Acceleration;
		float Density;
		float InvMass;
    float InvInertia;
		float Restitution;
		float Friction;
		float Damping;
		Vec2 AccumulatedForce;

		//Transform for this body
		Transform * tx;
		//Shape used for collision with this body
		Shape * BodyShape;
		//Static object are immovable fixed objects
		bool IsStatic;


	};
}
///////////////////////////////////////////////////////////////////////////////////////
///
///	\file Collision.h
///	Provides shapes that are used by Body Component for collision detection.
///	
///	Authors:  Chris Peters
///	Copyright 2010, Digipen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////////////
#pragma once //Makes sure this header is only included once

#include "VMath.h"
#include "Engine.h"
#include "Intersection.h"
#include "Manifold.h"

namespace Framework
{
	class Body;

	///Base Shape class
	class Shape
	{
	public:
		enum ShapeId
		{
			SidCircle,
			SidBox,
			SidNumberOfShapes
		};
		ShapeId Id;
		Body * body;
		Shape( ShapeId pid ) : Id(pid) {};
		virtual void Draw()=0;
		virtual bool TestPoint(Vec2)=0;
    virtual void ComputeMassAndInertia(float density, float& mass, float& inertia) = 0;
	};

	///Circle shape.
	class ShapeCircle : public Shape
	{	
	public:
		ShapeCircle() : Shape(SidCircle){};
		float Radius;
		virtual void Draw();
		virtual bool TestPoint(Vec2);
    virtual void ComputeMassAndInertia(float density, float& mass, float& inertia);
	};

	///Axis Aligned Box Shape
	class ShapeAAB : public Shape
	{
	public:
		ShapeAAB() : Shape(SidBox){};
		Vec2 Extents;
		virtual void Draw();
		virtual bool TestPoint(Vec2);
    virtual void ComputeMassAndInertia(float density, float& mass, float& inertia);
	};

	class ContactSet;
	typedef bool (*CollisionTest)(Body* bodyA, Body* bodyB, Manifold* m);

	///The collision database provides collision detection between shape types.
	class CollsionDatabase
	{
	public:	
		CollsionDatabase();
		CollisionTest CollsionRegistry[Shape::SidNumberOfShapes][Shape::SidNumberOfShapes];

    bool GenerateContacts(Body* bodyA, Body* bodyB, Manifold* m);
    void RegisterCollsionTest(Shape::ShapeId a , Shape::ShapeId b, CollisionTest test);
	};

}
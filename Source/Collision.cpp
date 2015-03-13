///////////////////////////////////////////////////////////////////////////////////////
//
//	Collsion.cpp
//
//	Authors:  Chris Peters
//	Copyright 2010, DigiPen Institute of Technology
//
///////////////////////////////////////////////////////////////////////////////////////
#include "Precompiled.h"
#include "Collision.h"
#include "Physics.h"
#include "DebugDraw.h"

namespace Framework
{

	

	void ShapeCircle::Draw()
	{
		Drawer::Instance.DrawCircle( body->Position , Radius );
	}

	bool ShapeCircle::TestPoint(Vec2 testPoint)
	{
		Vec2 delta = body->Position - testPoint;
		float dis = Normalize(delta);
		if( dis < Radius )
			return true;
		else
			return false;
	}

  void ShapeCircle::ComputeMassAndInertia(float density, float& mass, float& inertia)
  {
    float radiusSquared = Radius * Radius;
    //mass = pi * density * r^2
    mass = density * 3.14159f * radiusSquared;
    //inertia = pi * density * .5 * r^3
    inertia = mass * (0.5f * radiusSquared);
  }


	void ShapeAAB::Draw()
	{
		Drawer::Instance.MoveTo( body->Position + Vec2( Extents.x, Extents.y) );
		Drawer::Instance.LineTo( body->Position + Vec2(-Extents.x, Extents.y) );
		Drawer::Instance.LineTo( body->Position + Vec2(-Extents.x,-Extents.y) );
		Drawer::Instance.LineTo( body->Position + Vec2( Extents.x,-Extents.y) );
		Drawer::Instance.LineTo( body->Position + Vec2( Extents.x, Extents.y) );
		//Drawer::Instance.Flush();
	}

	bool ShapeAAB::TestPoint(Vec2 testPoint)
	{
		Vec2 delta = body->Position - testPoint;
		if( fabs(delta.x) < Extents.x )
		{
			if( fabs(delta.y) < Extents.y )
			{
				return true;
			}
		}
		return false;
	}

  void ShapeAAB::ComputeMassAndInertia(float density, float& mass, float& inertia)
  {
    float width = Extents.x;
    float height = Extents.y;
    //mass = density * width * height
    mass = density * width * height;
    //inertia = (1/12) * mass * (width^2 + height^2)
    inertia = (1.0f / 12.0f) * mass * (width * width + height * height);
  }

	/////////////////////Collsion Detection Functions////////////////////

	bool DetectCollisionCircleCircle(Body*a, Body*b, Manifold* m)
	{
    ShapeCircle* circleA = (ShapeCircle*)a->BodyShape;
    Vec2 circleAPos = a->Position;
    float circleARadius = circleA->Radius;

    ShapeCircle* circleB = (ShapeCircle*)b->BodyShape;
    Vec2 circleBPos = b->Position;
    float circleBRadius = circleB->Radius;

    return CircleCirlce(circleAPos,circleARadius,circleBPos,circleBRadius,m);
	}

	bool  DetectCollisionAABoxAABox(Body*a, Body*b, Manifold* m)
	{
    ShapeAAB* boxA = (ShapeAAB*)a->BodyShape;
    ShapeAAB* boxB = (ShapeAAB*)b->BodyShape;
    Vec2 boxAPos = a->Position;
    Vec2 boxAHalfExtents = boxA->Extents;
    Mat2 boxARot;
    boxARot.BuildRotation(a->Rotation);
    Vec2 boxAAxes[2];
    boxARot.GetBases(boxAAxes[0],boxAAxes[1]);
    
    Vec2 boxBPos = b->Position;
    Vec2 boxBHalfExtents = boxB->Extents;
    Mat2 boxBRot;
    boxBRot.BuildRotation(b->Rotation);
    Vec2 boxBAxes[2];
    boxBRot.GetBases(boxBAxes[0],boxBAxes[1]);


    return BoxBox(boxAPos,boxAHalfExtents,boxAAxes,boxBPos,boxBHalfExtents,boxBAxes,m);
	}


	//Auxiliary
	bool  DetectCollisionBoxCircle(Body*a, Body*b, Manifold* m)
	{
    ShapeCircle* circle = (ShapeCircle*)b->BodyShape;
    Vec2 circlePos = b->Position;
    float circleRadius = circle->Radius;
    ShapeAAB* box = (ShapeAAB*)a->BodyShape;
    Vec2 boxPos = a->Position;
    Vec2 boxHalfExtents = box->Extents;
    Mat2 boxRot;
    boxRot.BuildRotation(a->Rotation);
    Vec2 boxAxes[2];
    boxRot.GetBases(boxAxes[0],boxAxes[1]);

    return BoxCircle(boxPos,boxHalfExtents,boxAxes,circlePos,circleRadius,m);
	}

  bool  DetectCollisionCircleAABox(Body*a, Body*b, Manifold* m)
	{
    //since we swapped the shape ordering we have to swap the normal direction too
    if(DetectCollisionBoxCircle(b,a,m))
    {
      m->Normal *= -1;
      return true;
    }
    return false;
	}


	CollsionDatabase::CollsionDatabase()
	{
		//Register collision tests for all the shape types
		RegisterCollsionTest( Shape::SidCircle , Shape::SidCircle , DetectCollisionCircleCircle );
		RegisterCollsionTest( Shape::SidBox , Shape::SidBox , DetectCollisionAABoxAABox );
		RegisterCollsionTest( Shape::SidCircle , Shape::SidBox , DetectCollisionCircleAABox );
		RegisterCollsionTest( Shape::SidBox , Shape::SidCircle , DetectCollisionBoxCircle );
	}

  bool CollsionDatabase::GenerateContacts(Body* bodyA, Body* bodyB, Manifold* m)
  {
    return (*CollsionRegistry[bodyA->BodyShape->Id][bodyB->BodyShape->Id])(bodyA,bodyB,m);
  }

  void CollsionDatabase::RegisterCollsionTest(Shape::ShapeId a , Shape::ShapeId b, CollisionTest test)
  {
    CollsionRegistry[a][b] = test;
  }
}
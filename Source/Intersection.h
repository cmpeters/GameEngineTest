///////////////////////////////////////////////////////////////////////////////////////
//
//	Collision.h
//
//	Authors:  Benjamin Strukus
//	Copyright 2011, DigiPen Institute of Technology
//
///////////////////////////////////////////////////////////////////////////////////////
#pragma once

#include "VMath.h"

namespace Framework
{

  struct Manifold
  {
    struct ContactPoint
    {
      ///Points of intersection in world coordinates. Point 0 corresponds to the 
      ///point on the first shape furthest along the intersection axis and point 1
      ///corresponds to the point on the second shape.
      Vec2 Points[2];
      ///Amount of overlap occurring in the direction of the normal.
      float Depth;
    };

    ///Pairs of points of intersection describing the entire touching regions of
    ///two objects.
    ContactPoint Points[2];
    ///Direction of intersection in world coordinates. By convention, this always
    ///points in the direction away from the first shape to the second (in the
    ///function name).
    Vec2 Normal;

    ///Number of points in the manifold. This is used to tell the intersection 
    ///tests the maximum number of points that are available/should be returned.
    ///This value will be changed by the intersection tests to reflect the actual
    ///number of points that were generated.
    uint PointCount;

    ContactPoint& PointAt(uint index) { return Points[index]; }
  };


  bool CircleCirlce(Vec2Param circleCenterA, float circleRadiusA,
                    Vec2Param circleCenterB, float circleRadiusB, 
                    Manifold* manifold);

  bool  BoxCircle(Vec2Param boxCenter, Vec2Param boxHalfExtents, 
                  const Vec2* boxAxes, Vec2Param circleCenter, 
                  float circleRadius, Manifold* manifold);

  bool BoxBox(Vec2Param boxCenterA, Vec2Param boxHalfExtentsA, 
              const Vec2* boxAxesA, Vec2Param boxCenterB, 
              Vec2Param boxHalfExtentsB, const Vec2* boxAxesB, 
              Manifold* manifold);

}
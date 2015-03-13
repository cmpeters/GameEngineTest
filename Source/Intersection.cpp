///////////////////////////////////////////////////////////////////////////////////////
//
//	Collsion.cpp
//
//	Authors:  Benjamin Strukus
//	Copyright 2011, DigiPen Institute of Technology
//
///////////////////////////////////////////////////////////////////////////////////////
#include "Precompiled.h"
#include "Intersection.h"
#include "Physics.h"
#include "DebugDraw.h"

namespace Framework
{
  //Compute the distance between the point and the line segment. Saves the closest
  //point in the passed in pointer.
  void ClosestPointOnSegmentToPoint(Vec2Param segmentStart, Vec2Param segmentEnd, 
    Vec2* point)
  {
    //Vector from the segment's start to the segment's end
    Vec2 segment = segmentEnd - segmentStart;

    //Project the point onto the line segment, computing parameterized position
    //p(t) = segmentStart + t * (segmentEnd - segmentStart)
    float t = Dot(*point - segmentStart, segment) / Dot(segment, segment);

    //If the point is outside the segment, clamp t to the closest endpoint
    if(t < 0.0f)
      t = 0.0f;
    else if(t > 1.0f)
      t = 1.0f;

    //Compute closest point on the line segment from the clamped t
    *point = segmentStart + (segment * t);
  }

  //Find the intersection (the shared values) between the two intervals and return
  //the difference between the minimum and maximum value of the resulting 
  //interval. A negative result indicates that the two intervals are not 
  //intersecting.
  float IntersectIntervals(Vec2Param intervalA, Vec2Param intervalB)
  {
    const uint minIndex = 0;
    const uint maxIndex = 1;
    //get the largest min value
    float minVal = Max(intervalA[minIndex], intervalB[minIndex]);
    //get the smallest max
    float maxVal = Min(intervalA[maxIndex], intervalB[maxIndex]);
    //this result is the min and max of the overlap
    return maxVal - minVal;
  }

  //Project a box onto the given axis, storing its (absolute-valued) minimum and
  //maximum projections in the return vector.
  Vec2 ProjectBoxOntoAxis(Vec2Param boxCenter, Vec2Param boxHalfExtents, 
    const Vec2* boxAxes, Vec2Param axis)
  {
    //Compute all 4 points of the box
    Vec2 boxPoints[4] = { boxCenter + (boxAxes[0] * boxHalfExtents[0])
                                    + (boxAxes[1] * boxHalfExtents[1]),
                          boxCenter - (boxAxes[0] * boxHalfExtents[0])
                                    + (boxAxes[1] * boxHalfExtents[1]),
                          boxCenter - (boxAxes[0] * boxHalfExtents[0])
                                    - (boxAxes[1] * boxHalfExtents[1]),
                          boxCenter + (boxAxes[0] * boxHalfExtents[0])
                                    - (boxAxes[1] * boxHalfExtents[1]) };

    //Resulting minimum and maximum projections onto the axis
    float minProj =  PositiveMax();
    float maxProj = -PositiveMax();

    //Project all 4 points onto the axes, saving off the results
    for(uint i = 0; i < 4; ++i)
    {
      float projection = Dot(boxPoints[i], axis);
      if(projection < minProj)
      {
        minProj = projection;
      }
      if(projection > maxProj)
      {
        maxProj = projection;
      }
    }
    return Vec2(minProj, maxProj);
  }
 
  bool CircleCirlce(Vec2Param circleCenterA, float circleRadiusA,
                    Vec2Param circleCenterB, float circleRadiusB, 
                    Manifold* manifold)
  {
    //Represents the min distance that the centers of the circles can be
    //before they begin to overlap
    float radiiSum = circleRadiusA + circleRadiusB;

    //The direction and distance between the centers of the two circles
    Vec2 aToB = circleCenterB - circleCenterA;
    float centerDistance = aToB.Length();

    //If the distance between the two circle centers is less that the sum
    //of their radii, then the circles must be intersecting.
    if(centerDistance < radiiSum)
    {
      //if there is no detection info requested, then don't compute it
      if(manifold != NULL)
      {
        //Unit length direction vector pointing from circle A to circle B
        Vec2 normal = aToB / centerDistance;

        //Point on circle A deepest into circle B
        Vec2 pointOnA = circleCenterA + (normal * circleRadiusA);

        //Point on circle B deepest into circle A
        Vec2 pointOnB = circleCenterB - (normal * circleRadiusB);

        //Maximum amount that the two circles are overlapping
        float overlapAmount = radiiSum - centerDistance;

        manifold->PointAt(0).Points[0] = pointOnA;
        manifold->PointAt(0).Points[1] = pointOnB;
        manifold->PointAt(0).Depth = overlapAmount;
        manifold->Normal = normal;
        manifold->PointCount = 1;
      }

      //Only one point is furthest inside of each circle
      return true;
    }

    //Circles are not intersecting
    return false;
  }

  bool  BoxCircle(Vec2Param boxCenter, Vec2Param boxHalfExtents, 
                  const Vec2* boxAxes, Vec2Param circleCenter, 
                  float circleRadius, Manifold* manifold)
  {
    //As an optimization, we can rotate everything into the space of the box
    //(so the box is axis aligned and centered at the origin). This would
    //allows us to make assumptions and more easily find the closest point
    //on a segment (since the segments are axis aligned).
    //This is left as an exercise.

    //Compute all 4 points of the box
    const Vec2 boxPoints[4] = { boxCenter + boxAxes[0] * boxHalfExtents[0]
                                          + boxAxes[1] * boxHalfExtents[1],
                                boxCenter + boxAxes[0] * boxHalfExtents[0]
                                          - boxAxes[1] * boxHalfExtents[1],
                                boxCenter - boxAxes[0] * boxHalfExtents[0]
                                          - boxAxes[1] * boxHalfExtents[1],
                                boxCenter - boxAxes[0] * boxHalfExtents[0]
                                          + boxAxes[1] * boxHalfExtents[1] };

    //Find the closest point on the box to the circle
    Vec2 closestPoint;
    float shortestLength = PositiveMax();
    for(uint i = 0; i < 4; ++i)
    {
      //get the point on the box edge closest to the circle center
      Vec2 tempPoint = circleCenter;
      ClosestPointOnSegmentToPoint(boxPoints[i], boxPoints[(i + 1) % 4], 
        &tempPoint);

      //get the distance between that point and the circle center
      float tempLength = Length(circleCenter - tempPoint);
      //keep the closest point
      if(tempLength < shortestLength)
      {
        closestPoint = tempPoint;
        shortestLength = tempLength;
      }
    }

    //If the closest point is not within the circle radius,
    //then the circle isn't touching the box.
    if(shortestLength > circleRadius)
    {
      return false;
    }

    if(manifold != NULL)
    {
      //the normal is the vector from the box to the circle
      Vec2 normal = circleCenter - closestPoint;
      normal.Normalize();

      Vec2 circlePoint = circleCenter - (normal * circleRadius);

      Vec2 boxPoint = closestPoint;

      float overlapAmount = Length(circlePoint - boxPoint);

      manifold->PointAt(0).Points[0] = boxPoint;
      manifold->PointAt(0).Points[1] = circlePoint;
      manifold->PointAt(0).Depth = overlapAmount;
      manifold->Normal = normal;
      manifold->PointCount = 1;
    }

    return true;
  }

  ///Intersect a rotated box with a rotated box.
  bool BoxBox(Vec2Param boxCenterA, Vec2Param boxHalfExtentsA, 
              const Vec2* boxAxesA, Vec2Param boxCenterB, 
              Vec2Param boxHalfExtentsB, const Vec2* boxAxesB, 
              Manifold* manifold)
  {
    //This routine can be optimized by rotating everything into the space of
    //boxA (so that boxA is axis aligned and centered at the origin). This
    //will allow us to know boxA's face normals are the x and y axis. Look
    //at Real Time Collision Detection for more details.

    float minOverlap = PositiveMax();
    Vec2 minAxis;
    uint axisIndex = 5;

    //----------------------------------------------------------------------------
    //Project the boxes onto the axes of box A
    for(uint i = 0; i < 2; ++i)
    {
      Vec2 axis = boxAxesA[i];
      Vec2 intervalA = ProjectBoxOntoAxis(boxCenterA, boxHalfExtentsA, boxAxesA, 
                                          axis);
      Vec2 intervalB = ProjectBoxOntoAxis(boxCenterB, boxHalfExtentsB, boxAxesB,  
                                          axis);
      float overlapAmount = IntersectIntervals(intervalA, intervalB);

      //Overlap amount is negative, interval is invalid, no intersection
      if(overlapAmount < 0.0f)
      {
        return false;
      }
      if(overlapAmount < minOverlap)
      {
        minOverlap = overlapAmount;
        minAxis = axis;
        axisIndex = i;
      }
    }

    //----------------------------------------------------------------------------
    //Project the boxes onto the axes of box B
    for(uint i = 0; i < 2; ++i)
    {
      Vec2 axis = boxAxesB[i];
      Vec2 intervalA = ProjectBoxOntoAxis(boxCenterA, boxHalfExtentsA, boxAxesA, 
                                          axis);
      Vec2 intervalB = ProjectBoxOntoAxis(boxCenterB, boxHalfExtentsB, boxAxesB,  
                                          axis);
      float overlapAmount = IntersectIntervals(intervalA, intervalB);

      //Overlap amount is negative, interval is invalid, no intersection
      if(overlapAmount < 0.0f)
      {
        return false;
      }
      if(overlapAmount < minOverlap)
      {
        minOverlap = overlapAmount;
        minAxis = axis;
        axisIndex = i + 2;
      }
    }

    //----------------------------------------------------------------------------
    ErrorIf(axisIndex == 5, "Intersection - Axis index is invalid, impossible "\
                            "for this to break.");

    //No information needed & boxes are colliding
    if(manifold == NULL)
    {
      return true;
    }

    //Make sure that the normal is pointing from box A to box B
    Vec2 aToB = boxCenterB - boxCenterA;
    if(Dot(minAxis, aToB) < 0.0f)
    {
      minAxis *= -1.0f;
    }

    /*
         4-----------1
         |           |
         |           |
         |           |
         |           |
         |           |
         3-----------2
    */
    //make the x and y signs of each point on the box
    const float pointSigns[4][2] = { { 1.0f, 1.0f },
                                     { 1.0f,-1.0f }, 
                                     {-1.0f,-1.0f }, 
                                     {-1.0f, 1.0f } };

    Vec2 pointOnA, pointOnB;

    //Edge on box A, point/edge on box B
    if(axisIndex < 2)
    {
      //Find the point on box B that is furthest in the direction of box A
      float testValue = PositiveMax();
      for(uint i = 0; i < 4; ++i)
      {
        Vec2 tempPoint = boxAxesB[0] * pointSigns[i][0] * boxHalfExtentsB[0] +
                         boxAxesB[1] * pointSigns[i][1] * boxHalfExtentsB[1];
        float tempValue = Dot(-minAxis, tempPoint);
        if(tempValue < testValue)
        {
          pointOnB = tempPoint;
          testValue = tempValue;
        }
      }

      pointOnB = boxCenterB - pointOnB;

      //Find the edge of box A that's closest to box B
      //First find the side on box A that's facing box B
      Vec2 boxPointsA[2] = { boxCenterA, boxCenterA };
      float sign;
      if(Dot(boxAxesA[axisIndex], aToB) < 0.0)
      {
        sign = -1.0f;
      }
      else
      {
        sign = 1.0f;
      }
      uint altIndex = (axisIndex + 1) % 2;

      //All this is just to get points on the edge on box A closest to box B
      boxPointsA[0] += (sign * boxAxesA[axisIndex] * boxHalfExtentsA[axisIndex]) +
                       boxAxesA[altIndex] * boxHalfExtentsA[altIndex];

      boxPointsA[1] += (sign * boxAxesA[axisIndex] * boxHalfExtentsA[axisIndex]) -
                       boxAxesA[altIndex] * boxHalfExtentsA[altIndex];

      //Find the closest point on A's edge to B's point
      pointOnA = pointOnB;
      ClosestPointOnSegmentToPoint(boxPointsA[0], boxPointsA[1], &pointOnA);
      
    }
    //Edge on box B, point/edge on box A
    else
    {
      //Find the point on box A that is furthest in the direction of box B
      float testValue = -PositiveMax();
      for(uint i = 0; i < 4; ++i)
      {
        Vec2 tempPoint = boxAxesA[0] * pointSigns[i][0] * boxHalfExtentsA[0] +
                         boxAxesA[1] * pointSigns[i][1] * boxHalfExtentsA[1];
        float tempValue = Dot(minAxis, tempPoint);
        if(tempValue > testValue)
        {
          pointOnA = tempPoint;
          testValue = tempValue;
        }
      }

      pointOnA = boxCenterA + pointOnA;

      //Find the edge of box B that's closest to box A
      //First find the side on box B that's facing box A
      Vec2 boxPointsB[2] = { boxCenterB, boxCenterB };
      float sign;
      axisIndex -= 2;
      if(Dot(boxAxesB[axisIndex], aToB) > 0.0f)
      {
        sign = -1.0f;
      }
      else
      {
        sign = 1.0f;
      }
      uint altIndex = (axisIndex + 1) % 2;

      //All this is just to get points on the edge on box A closest to box B
      boxPointsB[0] += (sign * boxAxesB[axisIndex] * boxHalfExtentsB[axisIndex]) +
                       boxAxesB[altIndex] * boxHalfExtentsB[altIndex];

      boxPointsB[1] += (sign * boxAxesB[axisIndex] * boxHalfExtentsB[axisIndex]) -
                       boxAxesB[altIndex] * boxHalfExtentsB[altIndex];

      //Find the closest point on B's edge to A's point
      pointOnB = pointOnA;
      ClosestPointOnSegmentToPoint(boxPointsB[0], boxPointsB[1], &pointOnB);
    }

    manifold->PointAt(0).Points[0] = pointOnA;
    manifold->PointAt(0).Points[1] = pointOnB;
    manifold->PointAt(0).Depth = minOverlap;
    manifold->Normal = minAxis;
    manifold->PointCount = 1;

    return true;
  }

}

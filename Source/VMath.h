///////////////////////////////////////////////////////////////////////////////////////
///
///	\file VMath.h
///	Typedefs the DirectX Extension math library and provides some utility functions.
///
///	Authors: Chris Peters
///	Copyright 2010, Digipen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////////////
#pragma once //Makes sure this header is only included once

//Include our math headers
#include <d3dx9.h>
#include <cmath>
#include "Serialization.h"
#include "Vector2.hpp"
#include "Vector3.hpp"
#include "Matrix2.hpp"

//#include "Vector2.hpp"
//typedef D3DXVECTOR2 Vec2;
//typedef D3DXVECTOR3 Vec3;
typedef D3DXVECTOR4 Vec4;
typedef D3DXMATRIXA16 Mat4;
typedef D3DXVECTOR2 DxVec2;
typedef D3DXVECTOR3 DxVec3;

namespace Framework
{
	//Extended serialization operators of compound math types.
	inline void StreamRead(ISerializer& stream,Vec2& v)
	{
		StreamRead(stream,v.x);
		StreamRead(stream,v.y);
	}

	inline void StreamRead(ISerializer& stream,Vec3& v)
	{
		StreamRead(stream,v.x);
		StreamRead(stream,v.y);
		StreamRead(stream,v.z);
	}

	inline void StreamRead(ISerializer& stream,Vec4& v)
	{
		StreamRead(stream,v.x);
		StreamRead(stream,v.y);
		StreamRead(stream,v.z);
		StreamRead(stream,v.w);
	}


	/*inline float Dot(const Vec2& a, const Vec2& b)
	{
		return a.x * b.x + a.y * b.y;
	}

	inline float LengthSquared(const Vec2& a)
	{
		return a.x * a.x + a.y * a.y;
	}

	inline float Normalize(Vec2& a)
	{
		float len = sqrt( LengthSquared(a) );	
		a.x /= len;
		a.y /= len;
		return len;
	}

  inline float Cross2D(Vec2& v1, Vec2& v2)
  {
    return v1.x * v2.y - v2.x * v1.y;
  }

  inline Vec2 Cross2D(Vec2& v1, float w)
  {
    return Vec2(-v1.y,v1.x) * w;
  }*/

  inline float Max(float val1, float val2)
  {
    return val1 > val2 ? val1 : val2;
  }

  inline float Min(float val1, float val2)
  {
    return val1 < val2 ? val1 : val2;
  }

  inline float Clamp(float val, float minVal, float maxVal)
  {
    return Max(Min(val,maxVal),minVal);
  }

  inline float PositiveMax()
  {
    return FLT_MAX;
  }

  inline Vec2 TransformNormal(const Mat2& m, const Vec2& v)
  {
    Vec2 result;
    result[0] = m.Data[0] * v[0] + m.Data[1] * v[1];
    result[1] = m.Data[2] * v[0] + m.Data[3] * v[1];
    return result;
  }

	template< typename RefType >
	void SafeRelease( RefType& interfacePtr )
	{
		if( interfacePtr ) interfacePtr->Release();
		interfacePtr = NULL;
	}

	template< typename RefType >
	void SafeDelete( RefType& objectPtr )
	{
		if( objectPtr ) delete objectPtr;
		objectPtr = NULL;
	}

	typedef unsigned int uint;
}

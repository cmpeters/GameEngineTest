#pragma once

#include "Vector2.hpp"

namespace Framework
{

  struct Vector3
  {
    Vector3()
    {
      x = y = z = 0.0f;
    }

    Vector3(float xx, float yy, float zz)
    {
      x = xx;
      y = yy;
      z = zz;
    }

    Vector3(const Vector2& v2)
    {
      x = v2.x;
      y = v2.y;
      z = 0.0f;
    }

    float& operator[](unsigned int index)
    {
      return data[index];
    }

    float operator[](unsigned int index) const
    {
      return data[index];
    }

    Vector3 operator-()
    {
      Vector3 ret = *this;
      ret.x *= -1;
      ret.y *= -1;
      ret.z *= -1;
      return ret;
    }

    Vector3& operator+=(const Vector3& rhs)
    {
      x += rhs.x;
      y += rhs.y;
      z += rhs.z;
      return *this;
    }

    Vector3& operator-=(const Vector3& rhs)
    {
      x -= rhs.x;
      y -= rhs.y;
      z -= rhs.z;
      return *this;
    }

    Vector3 operator+(const Vector3& rhs) const
    {
      Vector3 ret = *this;
      ret += rhs;
      return ret;
    }

    Vector3 operator-(const Vector3& rhs) const
    {
      Vector3 ret = *this;
      ret -= rhs;
      return ret;
    }

    Vector3& operator*=(float rhs)
    {
      x *= rhs;
      y *= rhs;
      return *this;
    }

    Vector3& operator/=(float rhs)
    {
      x /= rhs;
      y /= rhs;
      return *this;
    }

    Vector3 operator*(float rhs) const
    {
      Vector3 ret = *this;
      ret *= rhs;
      return ret;
    }

    Vector3 operator/(float rhs) const
    {
      Vector3 ret = *this;
      ret /= rhs;
      return ret;
    }

    float LengthSq() const
    {
      return x * x + y * y * z * z;
    }

    float Normalize()
    {
      float len = sqrt( LengthSq() );	
      x /= len;
      y /= len;
      return len;
    }

    union
    {
      struct
      {
        float x,y,z;
      };
      struct {float data[3];};
    };
  };

  inline float Dot(const Vector3& lhs, const Vector3& rhs)
  {
    return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
  }

  inline float LengthSquared(const Vector3& lhs)
  {
    return lhs.LengthSq();
  }

  inline float Normalize(Vector3& lhs)
  {
    return lhs.Normalize();
  }

  inline Vector3 operator*(float lhs, const Vector3& rhs)
  {
    return rhs * lhs;
  }

  typedef Vector3 Vec3;
}

#pragma once

namespace Framework
{

  struct Vector2
  {
    Vector2()
    {
      x = y = 0.0f;
    }

    Vector2(float xx, float yy)
    {
      x = xx;
      y = yy;
    }

    //operator float* ()
    //{
    //  return &x;
    //}

    //operator const float* () const
    //{
    //  return &x;
    //}

    float& operator[](unsigned int index)
    {
      return data[index];
    }

    float operator[](unsigned int index) const
    {
      return data[index];
    }

    Vector2 operator-()
    {
      Vector2 ret = *this;
      ret.x *= -1;
      ret.y *= -1;
      return ret;
    }

    Vector2& operator+=(const Vector2& rhs)
    {
      x += rhs.x;
      y += rhs.y;
      return *this;
    }

    Vector2& operator-=(const Vector2& rhs)
    {
      x -= rhs.x;
      y -= rhs.y;
      return *this;
    }

    Vector2 operator+(const Vector2& rhs) const
    {
      Vector2 ret = *this;
      ret += rhs;
      return ret;
    }

    Vector2 operator-(const Vector2& rhs) const
    {
      Vector2 ret = *this;
      ret -= rhs;
      return ret;
    }

    Vector2& operator*=(float rhs)
    {
      x *= rhs;
      y *= rhs;
      return *this;
    }

    Vector2& operator/=(float rhs)
    {
      x /= rhs;
      y /= rhs;
      return *this;
    }

    Vector2 operator*(float rhs) const
    {
      Vector2 ret = *this;
      ret *= rhs;
      return ret;
    }

    Vector2 operator/(float rhs) const
    {
      Vector2 ret = *this;
      ret /= rhs;
      return ret;
    }

    float LengthSq() const
    {
      return x * x + y * y;
    }

    float Length() const
    {
      return sqrt(LengthSq());
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
        float x,y;
      };
      struct {float data[2];};
    };
  };

  inline float Dot(const Vector2& lhs, const Vector2& rhs)
  {
    return lhs.x * rhs.x + lhs.y * rhs.y;
  }

  //Represented by a 3D cross product where the z is zero. The result is the z of the new vector.
  inline float Cross2D(const Vector2& lhs, const Vector2& rhs)
  {
    return lhs.x * rhs.y - lhs.y * rhs.x;
  }

  //Represented by a 3D cross product where the second vector is just the z axis.
  inline Vector2 Cross2D(const Vector2& lhs, float z)
  {
    return Vector2(-lhs.y,lhs.x) * z;
  }

  inline float LengthSquared(const Vector2& lhs)
  {
    return lhs.LengthSq();
  }

  inline float Length(const Vector2& lhs)
  {
    return lhs.Length();
  }

  inline float Normalize(Vector2& lhs)
  {
    return lhs.Normalize();
  }

  inline Vector2 operator*(float lhs, const Vector2& rhs)
  {
    return rhs * lhs;
  }

  inline Vector2 TangentVector(const Vector2& rhs)
  {
    return Vector2(-rhs.y,rhs.x);
  }

  typedef Vector2 Vec2;
  typedef const Vector2& Vec2Param;
}
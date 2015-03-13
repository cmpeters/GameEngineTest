#pragma once

namespace Framework
{

  struct Matrix2
  {
    void BuildRotation(float angle)
    {
      float cosTheta = cos(angle);
      float sinTheta = sin(angle);
      Data[0] = cosTheta;
      Data[1] = -sinTheta;
      Data[2] = sinTheta;
      Data[3]  = cosTheta;
    }

    void Transpose()
    {
      float temp = Data[1];
      Data[1] = Data[2];
      Data[2] = temp;
    }

    void GetBases(Vec2& basisX, Vec2& basisY)
    {
      basisX = Vec2(Data[0],Data[2]);
      basisY = Vec2(Data[1],Data[3]);
    }

    float Data[4];
  };

  typedef Matrix2 Mat2;
}

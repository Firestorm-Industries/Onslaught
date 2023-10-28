#ifndef VECTOR2D_H
#define VECTOR2D_H

class Vector2d
{
  public:
    float x;
    float y;

    Vector2d() {;}

    Vector2d(float x, float y)
    : x(x), y(y) {;}

    Vector2d unit();
    Vector2d multiply(float toMultiplyBy);
    Vector2d add(Vector2d toAdd);
    Vector2d subtract(Vector2d toSubtract);

    float length();
    float distanceTo(Vector2d to);
};


#endif
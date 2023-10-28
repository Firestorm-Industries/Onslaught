#include "Vector2d.h"
#include <math.h>

Vector2d Vector2d::unit()
{
    float length = sqrt(x * x + y * y);
    return Vector2d(x / length, y / length);
}
Vector2d Vector2d::multiply(float toMultiplyBy)
{
    return Vector2d(x * toMultiplyBy, y * toMultiplyBy);
}


Vector2d Vector2d::add(Vector2d toAdd)
{
    return Vector2d(x + toAdd.x, y + toAdd.y);
}

Vector2d Vector2d::subtract(Vector2d toSubtract)
{
    return Vector2d(x - toSubtract.x, y - toSubtract.y);
}

float Vector2d::length()
{
    return sqrt(x * x + y * y);
}

float Vector2d::distanceTo(Vector2d to)
{
    float dX = to.x - x;
    float dY = to.y - y;
    return sqrt(dX * dX + dY - dY);
}
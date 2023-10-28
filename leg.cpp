#include <math.h>
#include <HardwareSerial.h>

#include "leg.h"

static int clamp(int value, int min, int max)
{
    if (value < min)
    {
        Serial.print("Clamped ");
        Serial.print(value);
        Serial.print(" to ");
        Serial.println(min);

        return min;
    }
    else if (value > max)
    {
        Serial.print("Clamped ");
        Serial.print(value);
        Serial.print(" to ");
        Serial.println(max);
        return max;
    }
    else return value;
}
static float clamp(float value, float min, float max)
{
    if (value < min) return min;
    else if (value > max) return max;
    else return value;
}


StepPoint StepPath::getPoint(int* index)
{

    if (*index >= currentPointCount)
    {
        *index = 0;
    }

    if (*index < 0)
    {
        *index = currentPointCount - 1;
    }

    return points[*index];
}

int StepPath::appendPoint(StepPoint newPoint)
{
    if (currentPointCount >= maxPointCount) 
    {
        return 1; //Invalid index
    }

    points[currentPointCount] = newPoint;

    currentPointCount ++;
    

    return 0;
}

int StepPath::appendPoint(float x, float y, bool rapidTo)
{
    StepPoint newPoint(x, y, rapidTo);
    appendPoint(newPoint);
}

void Leg::setAPosition(int newAPosition)
{
    aServoPosition = clamp(newAPosition, minServoPosition, maxServoPosition);
    A.write(aServoPosition);

    // Serial.print("A: ");
    // Serial.println(aServoPosition);
}

void Leg::setBPosition(int newBPosition)
{
    bServoPosition = clamp(newBPosition, minServoPosition, maxServoPosition);
    B.write(bServoPosition);
    
    // Serial.print("B: ");
    // Serial.println(bServoPosition);
}

void Leg::setXYActual(float x, float y)
{
    if (legLength > 0) x = clamp(x, -legLength, legLength);
    else x = clamp(x, legLength, -legLength);

    float theta = asin(x / legLength); // in radians
    theta = clamp(theta, minTheta, maxTheta);

    float centerHeight = y + legLength * cos(theta);

    // Serial.print(theta);
    // Serial.print(" ");
    // Serial.println(centerHeight);

    // Height of the a and b racks in mm
    float aHeightmm = centerHeight + legGearRadius * theta;
    aHeightmm = clamp(aHeightmm, minRackHeightmm, maxRackHeightmm);
    
    float bHeightmm = centerHeight - legGearRadius * theta;
    bHeightmm = clamp(bHeightmm, minRackHeightmm, maxRackHeightmm);

    // Height of the a and b racks in servo side teeth
    float aHeight = aHeightmm / (servoGearModulus * PI);
    float bHeight = bHeightmm / (servoGearModulus * PI);

    setAPosition( aHeight / servoGearTeeth * 360 + aMiddle );
    setBPosition( -bHeight / servoGearTeeth * 360 + bMiddle );
}

void Leg::begin(int APin, int BPin, int tAMiddle, int tBMiddle, StepPath* tPath)
{

    A.attach(APin);
    B.attach(BPin);

    aMiddle = tAMiddle;
    setAPosition(aMiddle);


    bMiddle = tBMiddle;
    setBPosition(bMiddle);

    legGearRadius = legGearModulus * legGearTeeth;

    path = tPath;

    currentPosition = path->getPoint(&nextPointIndex).position;

    // Serial.print("Position1: ");
    // Serial.print(currentPosition.x);
    // Serial.print(" ");
    // Serial.println(currentPosition.y);

    updateTarget();

}

void Leg::setXY(float x, float y)
{
    setXYActual(x, y);
    mode = position;
}

void Leg::setSpeed(float newSpeed)
{
    if (mode == velocity)
    {
        speed = newSpeed;
    }
    else
    {
        mode = velocity;
        speed = newSpeed;
    }
}

void Leg::setStepPath(StepPath* newPath)
{
    path = newPath;
}

void Leg::updateTarget()
{
    target = path->getPoint(&nextPointIndex);

    if (!upsideUp)
    {
        target.position.y *= -1;
    } //Flawed


    Vector2d dPosition = target.position.subtract(currentPosition);
    

    
    directionToTarget = dPosition.unit();
    distanceToTarget = dPosition.length();

    if (forwards)
    {
        rapid = target.rapid;
    }
    else{
        int temp = nextPointIndex + 1;
        rapid = path->getPoint(&temp).rapid;
    }

    nextPointIndex += indexDirection;
}

void Leg::update(float dt)
{
    switch (mode)
    {
    case velocity:
        // Serial.print("Position2: ");
        // Serial.print(currentPosition.x);
        // Serial.print(" ");
        // Serial.println(currentPosition.y);
        if (distanceToTarget <= 0)
        {
            updateTarget();
        }
        if (rapid)
        {
            distanceToTarget -= rapidSpeed * dt;
            currentPosition = currentPosition.add( directionToTarget.multiply(rapidSpeed * dt) );
        }
        else
        {
            distanceToTarget -= speed * dt;
            currentPosition = currentPosition.add( directionToTarget.multiply(speed * dt) );

        }

        // Serial.print("Position: ");
        // Serial.print(currentPosition.x);
        // Serial.print(" ");
        // Serial.print(currentPosition.y);
        // Serial.print(" Point Index: ");
        // Serial.print(nextPointIndex);
        // Serial.println();
        setXYActual(currentPosition.x, currentPosition.y);
        break;
    case position:

        break;
    
    case none:
        break;
    default:
        break;
    }


}

void Leg::flip()
{
    upsideUp = !upsideUp;

    legLength *= -1;

    nextPointIndex = 0;


    currentPosition = path->getPoint(&nextPointIndex).position;

    if (!upsideUp)
    {
        currentPosition.y *= -1;
    }


    updateTarget();

    

}

void Leg::reverse()
{
    forwards = !forwards;
    indexDirection *= -1;

    nextPointIndex += 2 * indexDirection;

    updateTarget();
}
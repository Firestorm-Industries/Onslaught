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

int StepPath::appendPoint(float aPosition, float angle, bool rapid)
{
    StepPoint newPoint(aPosition, angle, rapid);
    appendPoint(newPoint);
}

void Leg::setAAngle(int newAAngle)
{
    aServoAngle = clamp(newAAngle, minServoAngle, maxServoAngle);
    A.write(aServoAngle);

    // Serial.print("A: ");
    // Serial.println(aServoPosition);
}

void Leg::setBAngle(int newBAngle)
{
    bServoAngle = clamp(newBAngle, minServoAngle, maxServoAngle);
    B.write(bServoAngle);

    
    // Serial.print("B: ");
    // Serial.println(bServoPosition);
}

void Leg::setARackPosition(float newARackPosition) {
    aRackPosition_mm = clamp(newARackPosition, minRackPosition_mm, maxRackPosition_mm);
    aServoAngle = aRackPosition_mm / maxRackPosition_mm * 180;
    setAAngle(aServoAngle);
}

void Leg::setBRackPosition(float newBRackPosition) {
    bRackPosition_mm = clamp(newBRackPosition, minRackPosition_mm, maxRackPosition_mm);
    bServoAngle = bRackPosition_mm / maxRackPosition_mm * 180;
    setBAngle(bServoAngle);
}

float Leg::calculateBPosition(float aPosition, float angle) {
    float bPosition = aPosition;
    float bToothOffsetFromA = legGearTeeth * angle / 360;
    float bPositionOffsetFromA = bToothOffsetFromA * rackGearModulus * PI;
    bPosition += bPositionOffsetFromA;
    bPosition = clamp(bPosition, minRackPosition_mm, maxRackPosition_mm);

    return bPosition;
}

void Leg::setALegAngle(float aPosition_mm, float angle)
{
    aPosition_mm = clamp(aPosition_mm, minRackPosition_mm, maxRackPosition_mm);
    setARackPosition(aPosition_mm);

    setBRackPosition(calculateBPosition(aPosition_mm, angle));

}

void Leg::begin(int APin, int BPin, StepPath* tPath)
{


    A.attach(APin);
    B.attach(BPin);
    
    aRackPosition_mm = stoppedAPosition;
    currentAngle = stoppedAngle;
    bRackPosition_mm = calculateBPosition(aRackPosition_mm, currentAngle);

    legGearRadius = legGearModulus * legGearTeeth;

    path = tPath;

    // Serial.print("Position1: ");
    // Serial.print(currentPosition.x);
    // Serial.print(" ");
    // Serial.  println(currentPosition.y);

}

void Leg::setSpeed(float newSpeed)
{
    speed = clamp(newSpeed, -1.0, 1.0);
    if (newSpeed >= 0) {
        setForwards();
    } else {
        setBackwards();
    }
}

void Leg::setStepPath(StepPath* newPath)
{
    path = newPath;
}

void Leg::updateTarget()
{
    setARackPosition(targetAPosition);
    setBRackPosition(targetBPosition);

    switch (mode)
    {
    case(stopped):

        break;
    case(walking):
        
        StepPoint target = path->getPoint(&nextPointIndex);

        if (forwards)
        {
            rapiding = target.rapid;
            nextPointIndex ++;
        }
        else{
            int temp = nextPointIndex + 1;
            rapiding = path->getPoint(&temp).rapid;
            nextPointIndex --;
        }        
        
        break;
    case(stopping):

        targetAPosition = stoppedAPosition;
        targetAngle = stoppedAngle;
        targetBPosition = calculateBPosition(targetAPosition, targetAngle);
        
        mode = stopped;

        break;
    default:
        break;
    }
    
}

void Leg::update(float dt)
{
    float da = targetAPosition - aRackPosition_mm;
    float db = targetBPosition - bRackPosition_mm;

    float currentRackSpeed = maxRackSpeed_mmPs;
    if (!rapiding) {
        currentRackSpeed *= speed;
    }

    float rackMovement = currentRackSpeed * dt;

    if (da > db) {
        setARackPosition(aRackPosition_mm + rackMovement);
        float bMovement = rackMovement / da * db;
        setBRackPosition(bRackPosition_mm + bMovement);
    } else {
        setBRackPosition(bRackPosition_mm + rackMovement);
        float aMovement = rackMovement / db * da;
        setBRackPosition(aRackPosition_mm + aMovement);
    }
}

void Leg::setForwards()
{
    if (!forwards) {
        forwards = true;

        nextPointIndex += 2;

        updateTarget();
    } 
}

void Leg::setBackwards()
{
    if (forwards) {
        forwards = false;

        nextPointIndex -= 2;

        updateTarget();
    } 
}
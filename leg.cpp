#include <math.h>
#include <HardwareSerial.h>

#include "leg.h"

int sign(int num) {
    if (num > 0) {
        return 1;
    } else if (num < 0) {
        return -1;
    } else return 0;
}

static int clamp(int value, int min, int max)
{
    if (value < min) return min;
    else if (value > max) return max;
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
}

void Leg::setBAngle(int newBAngle)
{
    bServoAngle = clamp(newBAngle, minServoAngle, maxServoAngle);
    B.write(bServoAngle);
}

void Leg::setARackPosition(float newARackPosition) {
    aRackPosition_mm = clamp(newARackPosition, minRackPosition_mm, maxRackPosition_mm);
    aServoAngle = aRackPosition_mm / maxRackPosition_mm * 180;
    setAAngle(aServoAngle);
}

void Leg::setBRackPosition(float newBRackPosition) {
    bRackPosition_mm = clamp(newBRackPosition, minRackPosition_mm, maxRackPosition_mm);
    bServoAngle = 180 - bRackPosition_mm / maxRackPosition_mm * 180;
    setBAngle(bServoAngle);
}

float Leg::calculateBPosition(float aPosition, float angle) {
    float bPosition = aPosition;
    float bToothOffsetFromA = legGearTeeth * angle / 360;
    float bPositionOffsetFromA = bToothOffsetFromA * rackGearModulus * PI;
    bPosition += bPositionOffsetFromA;
    bPosition = clamp(bPosition, minRackPosition_mm, maxRackPosition_mm);
    // Serial.println(bPositionOffsetFromA);
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

    targetAPosition = aRackPosition_mm;
    targetBPosition = bRackPosition_mm;

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

    switch (mode)
    {
    case(stopped):

        break;
    case(walking):
        
        StepPoint target = path->getPoint(&nextPointIndex);
        targetAPosition = target.aPosition;
        targetAngle = target.angle;
        targetBPosition = calculateBPosition(targetAPosition, targetAngle);

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
        
        rapiding = true;

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

    bool complete = false;

    //Following doesnt work at all.. needs rerwrite
    float movement = -10;
    if (abs(da) < rackMovement and abs(db) < rackMovement) {
            setARackPosition(targetAPosition);
            setBRackPosition(targetBPosition);
            updateTarget();
    } else if (abs(da) > abs(db)) {
        setARackPosition(aRackPosition_mm + rackMovement * sign(da));
        movement = rackMovement / abs(da) * db;


        setBRackPosition(bRackPosition_mm + movement);
    } else {
        setBRackPosition(bRackPosition_mm + rackMovement * sign(db));
        movement = rackMovement / abs(db) * da;


        setARackPosition(aRackPosition_mm + movement);
    }


    

    // Serial.print("A: ");
    // Serial.print(aRackPosition_mm);
    // Serial.print(" B: ");
    // Serial.print(bRackPosition_mm);    
    // Serial.print(" Mode: ");
    // Serial.print(mode);
    // Serial.print(" Target A: ");
    // Serial.print(targetAPosition);
    // Serial.print(" target B: ");
    // Serial.print(targetBPosition);

    // Serial.println();

    Serial.print(aRackPosition_mm);
    Serial.print(" ");
    Serial.print(bRackPosition_mm);
    Serial.print(" ");
    Serial.print(targetAPosition);
    Serial.print(" ");
    Serial.print(targetBPosition);
    Serial.print(" ");
    Serial.print(nextPointIndex * 10);

    Serial.println();
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

void Leg::stop() {
    if (mode == walking) {
        mode = stopping;
        targetAPosition = aRackPosition_mm;
        targetAngle = 30;
        targetBPosition = calculateBPosition(targetAPosition, targetAngle);
    }

}

void Leg::start() {
    if (mode != walking) {
        rapiding = true;
        mode = walking;
        targetAPosition = aRackPosition_mm;
        targetAngle = 0;
        targetBPosition = calculateBPosition(targetAPosition, targetAngle);
    }    
}
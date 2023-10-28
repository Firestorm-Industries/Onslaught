#ifndef INVERSE_KINEMATICS_H
#define INVERSE_KINEMATICS_H
#include <Servo.h>

#include "Vector2d.h"

#define PI 3.1416



class StepPoint
{
  public:
    Vector2d position;
    bool rapid;
    StepPoint() {;}
    StepPoint(float x, float y, bool rapid)
    : position(x, y), rapid(rapid) {;}

};


class StepPath
{
    static constexpr int maxPointCount = 10;

    int currentPointCount = 0;

    StepPoint points[maxPointCount];

    bool isValid = false;

  public:
    StepPath() {;}
    StepPoint getPoint(int* index);
    int appendPoint(StepPoint newPoint);
    int appendPoint(float x, float y, bool rapidTo);

};

class Leg
{
    enum Modes {
      none,
      position,
      velocity
    };

    Servo A;
    Servo B;

    Vector2d currentPosition;

    int aServoPosition;
    int aMiddle; //Angle when rack is centered

    int bServoPosition;
    int bMiddle; //Angle when rack is centered

    int minServoPosition = 0;
    int maxServoPosition = 180;

    float minTheta = -0.5;
    float maxTheta = 0.5;

    float minRackHeightmm = -10;
    float maxRackHeightmm = 10;

    float legLength = 30;
    int legGearTeeth = 8; //Number of teeth of the gear attached to the leg
    float legGearModulus = 1.25; //Modulus of the gear attached to the leg
    float legGearRadius;

    int servoGearTeeth = 12;
    float servoGearModulus = 1.25;
    
    float rapidSpeed = 50;
    float speed;

    bool rapid;

    StepPoint target;
    int nextPointIndex = 0;
    float distanceToTarget;
    Vector2d directionToTarget;


    StepPath* path;
    Modes mode = none;

    bool upsideUp = true; //1 for up 1 for down

    
    bool forwards = true;
    int indexDirection = 1;


    void setAPosition(int newAPosition);
    void setBPosition(int newBPosition);
    void setXYActual(float x, float y);
    void updateTarget();

  public:

    Leg() {;}
    void begin(int APin, int BPin, int tAMiddle, int tBMiddle, StepPath* tPath);
    void setXY(float x, float y);
    void setSpeed(float newSpeed);
    void setStepPath(StepPath* newPath);
    void update(float dt);

    void flip();
    void reverse();
};


#endif
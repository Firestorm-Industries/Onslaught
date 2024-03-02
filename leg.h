#ifndef INVERSE_KINEMATICS_H
#define INVERSE_KINEMATICS_H
#include <Servo.h>

#include "Vector2d.h"

#define PI 3.1416


class StepPoint
{
  public:
    float aPosition;
    float angle;
    bool rapid; //Rapids to this point in forward or from this point in reverse

    StepPoint() {;}
    StepPoint(float aPosition, float angle, bool rapid)
      : aPosition{aPosition}, angle{angle}, rapid(rapid) {;}

};


class StepPath
{
    static constexpr int maxPointCount = 4;

    int currentPointCount = 0;

    StepPoint points[maxPointCount];

    bool isValid = false;

  public:
    StepPath() {;}
    StepPoint getPoint(int* index);
    int appendPoint(StepPoint newPoint);
    int appendPoint(float aPosition, float angle, bool rapid);

};

class Leg
{
    enum Modes {
      stopped,
      walking,
      stopping
    };

    Modes mode = stopped;


    Servo A;
    Servo B;

    float stoppedAPosition = 20;
    float stoppedAngle = 30;

    float currentAngle = stoppedAngle;

    int aServoAngle;
    float aRackPosition_mm;

    int bServoAngle;
    float bRackPosition_mm;

    float rackGearModulus = 1; //Modulus of the rack


    int minServoAngle = 0;
    int maxServoAngle = 180;

    float minRackPosition_mm = 0;
    float maxRackPosition_mm = 12.5 * PI;
    int legGearTeeth = 12; //Number of teeth of the gear attached to the leg
    float legGearModulus = 1; //Modulus of the gear attached to the leg
    float legGearRadius;

    int servoGearTeeth = 25;
    float servoGearModulus = 1;
    
    float maxServoSpeed_degPs = 600;//600;
    float maxRackSpeed_mmPs = maxServoSpeed_degPs / 260 * 25 * PI;
    
    float speed = 0; //0 to 1

    bool rapiding;

    float targetAPosition;
    float targetBPosition;
    float targetAngle;

    int nextPointIndex = 0;



    StepPath* path;
    
    bool forwards = true;


    void setAAngle(int newAAngle);
    void setBAngle(int newBAngle);

    void setARackPosition(float newARackPosition);
    void setBRackPosition(float newBRackPosition);

    void updateTarget();

    float calculateBPosition(float aPosition, float angle);

  public:

    Leg() {;}
    void begin(int APin, int BPin, StepPath* tPath);
    void setALegAngle(float aPosition, float angle);
    void setSpeed(float newSpeed); //0 to 1 of max
    void setStepPath(StepPath* newPath);
    void update(float dt);

    void setForwards();
    void setBackwards();
    void stop();
    void start();
};


#endif
#include <Servo.h>
#include "leg.h"

Leg test;

StepPath path;
unsigned long prevMicros = 0;
float x = -10;


// Servo testS;

float tipSpeed = 10; // mm/s

bool goingPositive = true;

void setup(){
    path.appendPoint(-10, -30, true);
    path.appendPoint(10, -30, false);
    path.appendPoint(10, -25, true);
    path.appendPoint(-10, -25, true);

    pinMode(13, OUTPUT);
    Serial.begin(9600);
    test.begin(9, 10, 110, 90, &path);

    test.setSpeed(10);
    
}

void loop(){
    unsigned long newMicros = micros();
    float dt = float(newMicros - prevMicros);// / 1000000.0;
    dt /= 1000000.0;


    if (prevMicros / 5000000  != newMicros / 5000000)
    {
        test.reverse();
        Serial.println("Reverse");
    }

    if (prevMicros / 10000000  != newMicros / 10000000)
    {
        test.flip();
        Serial.println("Flip");
    }

    // Serial.println(dt, 6);
    test.update(dt);

    prevMicros = newMicros;

}
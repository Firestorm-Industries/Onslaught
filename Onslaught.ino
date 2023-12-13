#include <Servo.h>
#include "leg.h"

Leg test;

StepPath path;
unsigned long prevMicros = 0;


// Servo testS;


bool goingPositive = true;

void setup(){
    path.appendPoint(0, 0, false);
    path.appendPoint(0, 90, true);
    path.appendPoint(30, 90, true);
    path.appendPoint(40, 0, true);


    pinMode(13, OUTPUT);
    Serial.begin(9600);
    test.begin(9, 10, &path);

    test.setSpeed(0/8);
    test.setSpeed(.2);
    test.start();
    
}

void loop(){
    unsigned long newMicros = micros();
    float dt = float(newMicros - prevMicros);// / 1000000.0;
    dt /= 1000000.0;



    test.update(dt);

    prevMicros = newMicros;

}
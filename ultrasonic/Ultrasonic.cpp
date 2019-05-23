//
// Created by Jamshid Mamatkulov on 2019-05-08.
//

#include "Ultrasonic.h"
#include <iostream>
#include <wiringPi.h>

void Ultrasonic::init(){
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    digitalWrite(TRIG_PIN, LOW);
    delay(500);
}

double Ultrasonic::getDistance(){
    delay(100);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long now = micros();

    while (digitalRead(ECHO_PIN) == LOW){
        if(now + 30000 < micros())
            return -1;
    }

    long startTimeUsec = micros();
    while ( digitalRead(ECHO_PIN) == HIGH );
    long endTimeUsec = micros();

    long travelTimeUsec = endTimeUsec - startTimeUsec;

    double distance = (travelTimeUsec)/29./2.;

    return distance;
}
//
// Created by Jamshid Mamatkulov on 2019-05-08.
//

#ifndef ADVANCED_IMAGE_PROC_ULTRASONIC_H
#define ADVANCED_IMAGE_PROC_ULTRASONIC_H


#define TRIG_PIN 28
#define ECHO_PIN 29


class Ultrasonic {
public:
    void init();
    double getDistance();
};


#endif //ADVANCED_IMAGE_PROC_ULTRASONIC_H

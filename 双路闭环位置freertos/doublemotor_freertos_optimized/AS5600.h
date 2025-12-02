#ifndef AS5600_H
#define AS5600_H

#include "Wire.h"

class Sensor_AS5600 {
public:
    Sensor_AS5600(); // Constructor
    void Sensor_init(int sda_pin, int scl_pin);
    void Sensor_update();
    float getAngle();
    float getMechanicalAngle();
    float getVelocity();

private:
    double getSensorAngle();
    
    TwoWire wire;
    float angle_prev;
    long full_rotations;
    unsigned long angle_prev_ts;
    float vel_angle_prev;
    long vel_full_rotations;
    unsigned long vel_angle_prev_ts;
};

#endif
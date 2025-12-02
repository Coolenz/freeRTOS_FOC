#include "AS5600.h"
#include <Arduino.h>

#define _2PI 6.28318530718f

Sensor_AS5600::Sensor_AS5600() 
    : wire(0) // Initialize TwoWire with bus ID 0, will be changed in init
{
    // Initialize variables
    angle_prev = 0;
    full_rotations = 0;
    angle_prev_ts = 0;
    vel_angle_prev = 0;
    vel_full_rotations = 0;
    vel_angle_prev_ts = 0;
}

// A more robust init function
void Sensor_AS5600::Sensor_init(int sda_pin, int scl_pin) {
    // Determine which I2C bus to use based on pins
    // This is a simple way, assuming default pins are for Wire (bus 0)
    if (sda_pin == 23 && scl_pin == 5) { // Default I2C_1 pins
        wire = TwoWire(1);
    } else { // Default to bus 0
        wire = TwoWire(0);
    }
    
    wire.begin(sda_pin, scl_pin, 400000UL);
    delay(500);

    // Prime the angle and velocity readings
    Sensor_update();
    angle_prev = getMechanicalAngle();
    vel_angle_prev = angle_prev;
    angle_prev_ts = micros();
    vel_angle_prev_ts = angle_prev_ts;
}

void Sensor_AS5600::Sensor_update() {
    float val = getSensorAngle();
    angle_prev_ts = micros();
    float d_angle = val - angle_prev;
    if (abs(d_angle) > (0.8f * _2PI)) {
        full_rotations += (d_angle > 0) ? -1 : 1;
    }
    angle_prev = val;
}                 

double Sensor_AS5600::getSensorAngle() {
    byte readArray[2];
    wire.beginTransmission(0x36);
    wire.write(0x0C); // Raw angle register
    wire.endTransmission(false);
    wire.requestFrom(0x36, (uint8_t)2);
    for (byte i = 0; i < 2; i++) {
        readArray[i] = wire.read();
    }
    // 12-bit resolution, raw value is in bits 11:0
    uint16_t readValue = (readArray[0] << 8) | readArray[1];
    return (readValue / 4096.0) * _2PI;
}

float Sensor_AS5600::getMechanicalAngle() {
    return angle_prev;
}

float Sensor_AS5600::getAngle() {
    return (float)full_rotations * _2PI + angle_prev;
}

float Sensor_AS5600::getVelocity() {
    float Ts = (angle_prev_ts - vel_angle_prev_ts) * 1e-6;
    if (Ts <= 0) Ts = 1e-3f;
    float vel = ((full_rotations - vel_full_rotations) * _2PI + (angle_prev - vel_angle_prev)) / Ts;
    vel_angle_prev = angle_prev;
    vel_full_rotations = full_rotations;
    vel_angle_prev_ts = angle_prev_ts;
    return vel;
}
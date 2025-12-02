#ifndef DENG_FOC_H
#define DENG_FOC_H

#include "AS5600.h"
#include "pid.h"      // You must have a PID class implementation for this to work
#include "lowpass_filter.h" // You must have a filter class implementation for this to work

// The main class that encapsulates everything a motor needs
class MotorFOC {
public:
    // Constructor: Takes all the hardware-specific parameters
    MotorFOC(int pwmA, int pwmB, int pwmC, int en, int channel_offset, int sda_pin, int scl_pin);

    void init();
    void alignSensor(int pp, int dir);

    // Main control loop to be called repeatedly
    void loopFOC(float target_angle);

    // Public getters
    float getAngle();
    float getVelocity();

private:
    // Private methods for internal FOC logic
    float _electricalAngle();
    void setPwm(float Ua, float Ub, float Uc);
    void setTorque(float Uq, float angle_el);

    // Hardware pins and configs
    int _pwmA, _pwmB, _pwmC, _en;
    int _channel_offset; // 0 for motor0 (channels 0,1,2), 3 for motor1 (channels 3,4,5)
    int _sda, _scl;

    // Motor parameters
    int PP;
    int DIR;
    float zero_electric_angle;
    
    // Static members are shared across all instances of the class
    static float voltage_power_supply;

    // Embedded objects for sensor and controllers
    Sensor_AS5600 sensor;
    PIDController angle_loop;
    PIDController vel_loop;
    LowPassFilter vel_filter;

    // This makes the global DFOC_Init function a "friend" of this class,
    // allowing it to access private members like voltage_power_supply.
    friend void DFOC_Init(float voltage);
};

// Global function to initialize shared resources
void DFOC_Init(float voltage);

// Global functions for serial communication
void serialReceiveUserCommand();
float serial_motor0_target();
float serial_motor1_target();

#endif // This is the correct closing tag for the #ifndef at the top
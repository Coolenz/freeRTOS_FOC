#include "DengFOC.h"
#include <Arduino.h>

// Initialize static member variable
float MotorFOC::voltage_power_supply = 12.0;

// =========================================================================
// ==           Define the two global motor objects here                  ==
// =========================================================================
// MotorFOC(pwmA, pwmB, pwmC, EnablePin, PwmChannelOffset, sda, scl)
MotorFOC motor0(32, 33, 25, 12, 0, 19, 18);
MotorFOC motor1(26, 27, 14, 12, 3, 23, 5); // Note: EnablePin can be shared or separate
// =========================================================================


// Constructor implementation
MotorFOC::MotorFOC(int pwmA, int pwmB, int pwmC, int en, int channel_offset, int sda_pin, int scl_pin)
    : sensor(), // Call default constructor for sensor
        angle_loop(1.0, 0.02, 0, 100000, 100),   // P, I, D, Ramp, Limit
      vel_loop(0.1, 0, 0, 100000, 6.0),      // P, I, D, Ramp, Limit
      vel_filter(0.01) // 10ms time constant for filter
{
    // Store hardware pins
    _pwmA = pwmA;
    _pwmB = pwmB;
    _pwmC = pwmC;
    _en = en;
    _channel_offset = channel_offset;
    _sda = sda_pin;
    _scl = scl_pin;

    // Default motor parameters
    PP = 0;
    DIR = 1;
    zero_electric_angle = 0;
}

void MotorFOC::init() {
    // Setup PWM pins
    pinMode(_pwmA, OUTPUT);
    pinMode(_pwmB, OUTPUT);
    pinMode(_pwmC, OUTPUT);
    ledcSetup(0 + _channel_offset, 30000, 8);
    ledcSetup(1 + _channel_offset, 30000, 8);
    ledcSetup(2 + _channel_offset, 30000, 8);
    ledcAttachPin(_pwmA, 0 + _channel_offset);
    ledcAttachPin(_pwmB, 1 + _channel_offset);
    ledcAttachPin(_pwmC, 2 + _channel_offset);

    // Initialize the sensor on its specific I2C pins
    sensor.Sensor_init(_sda, _scl);
}

void MotorFOC::alignSensor(int pp, int dir) {
    PP = pp;
    DIR = dir;
    Serial.println("Aligning sensor... Motor will move.");
    setTorque(3.0, 4.71238898038f); // Apply voltage at 3PI/2 electrical angle
    delay(1000);
    sensor.Sensor_update(); // Update sensor to get a stable reading
    zero_electric_angle = _electricalAngle();
    setTorque(0, 4.71238898038f); // Release motor
    Serial.print("Alignment complete. Zero electric angle: ");
    Serial.println(zero_electric_angle);
}


void MotorFOC::loopFOC(float target_angle) {
    sensor.Sensor_update();

    // Position loop
    float angle_error = target_angle - getAngle();
    float target_velocity = angle_loop(angle_error);

    // Velocity loop
    float velocity_error = target_velocity - getVelocity();
    float target_voltage = vel_loop(velocity_error);

    // Set the final torque
    setTorque(target_voltage, _electricalAngle());
}

float MotorFOC::_electricalAngle() {
    return fmod((DIR * PP * sensor.getMechanicalAngle() - zero_electric_angle), 6.28318530718f);
}

void MotorFOC::setPwm(float Ua, float Ub, float Uc) {
    float dc_a = constrain(Ua / voltage_power_supply, 0.0f, 1.0f);
    float dc_b = constrain(Ub / voltage_power_supply, 0.0f, 1.0f);
    float dc_c = constrain(Uc / voltage_power_supply, 0.0f, 1.0f);

    ledcWrite(0 + _channel_offset, dc_a * 255);
    ledcWrite(1 + _channel_offset, dc_b * 255);
    ledcWrite(2 + _channel_offset, dc_c * 255);
}

void MotorFOC::setTorque(float Uq, float angle_el) {
    Uq = constrain(Uq, -voltage_power_supply / 2, voltage_power_supply / 2);

    float Ualpha = -Uq * sin(angle_el);
    float Ubeta = Uq * cos(angle_el);

    float Ua = Ualpha + voltage_power_supply / 2;
    float Ub = (sqrt(3) * Ubeta - Ualpha) / 2 + voltage_power_supply / 2;
    float Uc = (-Ualpha - sqrt(3) * Ubeta) / 2 + voltage_power_supply / 2;

    setPwm(Ua, Ub, Uc);
}

float MotorFOC::getAngle() {
    return DIR * sensor.getAngle();
}

float MotorFOC::getVelocity() {
    return DIR * vel_filter(sensor.getVelocity());
}

// ===============================================
// == Global and Serial Functions               ==
// ===============================================

void DFOC_Init(float voltage) {
    // Set static voltage shared by all motor objects
    MotorFOC::voltage_power_supply = voltage;
    
    // Setup shared enable pin
    pinMode(12, OUTPUT);
    digitalWrite(12, HIGH);
    
    Serial.println("PWM and global settings initialized.");
}

// Target variables for serial control
float M0_target = 0;
float M1_target = 0;

void serialReceiveUserCommand() {
    static String received_chars;
    while (Serial.available()) {
        char inChar = (char)Serial.read();
        received_chars += inChar;
        if (inChar == '\n') {
            int commaPosition = received_chars.indexOf(',');
            if (commaPosition != -1) {
                String target0_str = received_chars.substring(0, commaPosition);
                String target1_str = received_chars.substring(commaPosition + 1);
                M0_target = target0_str.toFloat();
                M1_target = target1_str.toFloat();
            }
            received_chars = "";
        }
    }
}

float serial_motor0_target() { return M0_target; }
float serial_motor1_target() { return M1_target; }
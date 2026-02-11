#ifndef SERVO_MOTOR_H
#define SERVO_MOTOR_H

#include "IMotor.h"
#include <Arduino.h>
#include <ESP32Servo.h>
#pragma once
#include <Arduino.h>
#include <Dynamixel2Arduino.h>

class ServoMotor {
public:
    // Avant: ServoMotor(uint8_t pwmPin)
    // Maintenant: on injecte l'objet dxl + l'ID du moteur
    ServoMotor(Dynamixel2Arduino& dxl, uint8_t dxl_id);

    void init(uint32_t baudrate = 57600, float protocol = 2.0f);

    void setSpeed(float speed_deg_s);   // vitesse en deg/s (rampe logicielle)
    void moveToAngle(float angle_deg);  // consigne en degrés [0..180] (clamp comme avant)
    float getCurrentAngle();

    void enable();   // torque ON
    void disable();  // torque OFF

    bool isEnabled();
    bool isMoving();
    void stop();

    void update();   // applique la rampe et envoie setGoalPosition()

private:
    Dynamixel2Arduino& dxl;
    uint8_t id;

    float currentAngle;
    float targetAngle;
    float speed;             // deg/s (rampe logicielle)
    bool enabled;
    bool isMovingFlag;

    unsigned long lastUpdateTime;

    // utilitaire interne
    float readPresentAngleDeg();
};

#endif
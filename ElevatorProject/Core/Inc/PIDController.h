/*
 * PIDController.h
 *
 *  Created on: Dec 3, 2023
 *      Author: Wouter Swinkels
 */

#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

typedef struct {
    double Kp;       // Proportional gain
    double Ki;       // Integral gain
    double Kd;       // Derivative gain
    double maxError; // Maximum expected error for normalization
} PIDConstants;

typedef struct {
    double currentError;  // Current error
    double previousError; // Previous error (for derivative)
    double integral;      // Integral (sum) value
    double derivative;    // Derivative value
    double timeDelta;     // Time delta for calculations (in seconds)
    double targetStep;    // Desired target step
} PIDProcessValues;

typedef struct {
    PIDConstants constants;
    PIDProcessValues values;
} PIDController;

PIDController* initPIDController(double Kp, double Ki, double Kd, double timeDelta, double maxError);

double calculatePIDControlValue(PIDController *pid);

double calculateNormalizedPIDControlValue(PIDController *pid);

int updatePIDError(PIDController *pid, double new_error);

void setPIDStep(PIDController* pid, double targetStep, double currentProcessValue);

#endif // PIDCONTROLLER_H

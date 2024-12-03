/*
 * PIDController.c
 *
 *  Created on: Dec 5, 2023
 *      Author: Wouter Swinkels
 */

#include <stdlib.h>
#include <math.h>

#include "PIDController.h"

PIDController* initPIDController(double Kp, double Ki, double Kd, double timeDelta, double maxError) {
    PIDController *pid = (PIDController*)malloc(sizeof(PIDController));
    if (pid == NULL) {
        return NULL;
    }

    pid->constants.Kp = Kp;
    pid->constants.Ki = Ki;
    pid->constants.Kd = Kd;
    pid->constants.maxError = maxError;
    pid->values.timeDelta = timeDelta;

    // Initialize other members to zero, they're
    // going to be calculated during operation.
    pid->values.currentError = 0.0;
    pid->values.previousError = 0.0;
    pid->values.integral = 0.0;
    pid->values.derivative = 0.0;

    return pid;
}

int updatePIDError(PIDController *pid, double currentProcessValue) {
    if (pid == NULL) {
        return -1;
    }
    pid->values.currentError = pid->values.targetStep - currentProcessValue;
    return 0;
}


void setPIDStep(PIDController* pid, double targetStep, double currentProcessValue) {
    if (pid == NULL) {
        return;
    }

    pid->values.targetStep = targetStep;
    pid->values.currentError = targetStep - currentProcessValue;
    pid->values.previousError = 0.0; // reset to not mess up initial derivative calculation
}


double getPIDTimeDelta(PIDController *pid) {
    if (pid == NULL) {
        return -1.0;
    }
    return pid->values.timeDelta;
}


double calculateNormalizedPIDControlValue(PIDController *controller) {
    if (controller == NULL) {
        return 0.0; // does nothing, i.e., pauses given actuator.
    }

    // Calculate difference between the current and previous error:
    double errorDifference = controller->values.currentError - controller->values.previousError;

    // Update integral term:
    controller->values.integral += controller->values.currentError * controller->values.timeDelta;

    // Update derivative term using time delta:
    controller->values.derivative = errorDifference / controller->values.timeDelta;

    // Control value calculation:
    double controlValue = (controller->constants.Kp * controller->values.currentError) +
                          (controller->constants.Ki * controller->values.integral) +
                          (controller->constants.Kd * controller->values.derivative);

    // Normalize control value based on maximum control effort:
    double maxControlEffort = controller->constants.Kp * fabs(controller->constants.maxError);
    if (maxControlEffort == 0.0) {
        return 0.0; // Prevent division by zero if maxError is zero.
    }

    double normalizedControlValue = controlValue / maxControlEffort;
    normalizedControlValue = fmax(fmin(normalizedControlValue, 1.0), -1.0);

    // Update the previous error for next iteration:
    controller->values.previousError = controller->values.currentError;

    return normalizedControlValue;
}

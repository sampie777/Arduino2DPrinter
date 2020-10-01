//
// Created by S. Jansen.
//

#include "StepperMotorBipolar.h"


void StepperMotorBipolar::blindStep(int8_t direction, uint16_t stepDelay) {
    if (lastStepTime + stepDelay > micros()) {
        // Wait till last step is complete
        return;
    }
    lastStepTime = micros();

    if (direction == SM_STEP_DIRECTION_UP) {
        currentStepCoil <<= 1;
        currentStep++;
    } else {
        currentStepCoil >>= 1;
        currentStep--;
    }

    if (currentStepCoil > 8) {
        currentStepCoil = 1;
    } else if (currentStepCoil == 0) {
        currentStepCoil = 8;
    }

    digitalWrite(coilA, currentStepCoil & (1 | 2));
    digitalWrite(coilC, currentStepCoil & (2 | 4));   // First C
    digitalWrite(coilB, currentStepCoil & (4 | 8));   // Then B
    digitalWrite(coilD, currentStepCoil & (8 | 1));
}

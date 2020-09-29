//
// Created by S. Jansen.
//

#include "StepperMotorBipolar.h"


void StepperMotorBipolar::blindStep(int8_t direction, uint16_t stepDelay) {
    static uint8_t currentStepCoil = 1;

    if (lastStepTime + stepDelay > micros()) {
        // Wait till last step is complete
        return;
    }
    lastStepTime = micros();

    if (direction == SM_STEP_DIRECTION_UP) {
        currentStepCoil <<= 1U;
        currentStep++;
    } else {
        currentStepCoil >>= 1U;
        currentStep--;
    }

    if (currentStepCoil > 0x08U) {
        currentStepCoil = 1;
    } else if (currentStepCoil == 0) {
        currentStepCoil = 0x08U;
    }

    digitalWrite(coilA, currentStepCoil & 0x01U);
    digitalWrite(coilC, currentStepCoil & 0x02U);   // First C
    digitalWrite(coilB, currentStepCoil & 0x04U);   // Then B
    digitalWrite(coilD, currentStepCoil & 0x08U);
}

void StepperMotorBipolar::step() {
    if (lastStepTime + stepDelay > micros()) {
        // Wait till last step is complete
        return;
    }

    StepperMotor::step();
}

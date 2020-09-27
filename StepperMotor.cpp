//
// Created by S. Jansen on 4/29/20.
//

#include "StepperMotor.h"

void StepperMotor::setupSMPins() const {
    pinMode(coilA, OUTPUT);
    pinMode(coilB, OUTPUT);
    pinMode(coilC, OUTPUT);
    pinMode(coilD, OUTPUT);
    pinMode(endpointSensor, INPUT_PULLUP);
}

bool StepperMotor::endpointReached() const {
    return !digitalRead(endpointSensor);
}

void StepperMotor::blindStep(int8_t direction, uint16_t stepDelay) {
    static uint8_t currentStepCoil = 1;

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
    digitalWrite(coilB, currentStepCoil & 0x02U);
    digitalWrite(coilC, currentStepCoil & 0x04U);
    digitalWrite(coilD, currentStepCoil & 0x08U);

    delayMicroseconds(stepDelay);

    digitalWrite(coilA, LOW);
    digitalWrite(coilB, LOW);
    digitalWrite(coilC, LOW);
    digitalWrite(coilD, LOW);
}

void StepperMotor::step(uint8_t force) {
    if (currentStep == targetStep) {
        stopAllCoils();
        return;
    }

    int8_t stepDirection = getStepDirectionForTargetStep();

    if (!force) {
        if (stepDirection == SM_STEP_DIRECTION_UP) {
            if (currentStep >= getMaxStep()) {
                Serial.println("[StepperMotor] Max step reached");
                stopAllCoils();
                return;
            }
        } else {
            // Prevent from pushing through endpoint
            if (endpointReached()) {
                Serial.println("[StepperMotor] Prevent pushing through endpoint!");
                currentStep = 0;
            }

            if (currentStep == 0) {
                Serial.println("[StepperMotor] Min step reached");
                stopAllCoils();
                return;
            }
        }
    }

    blindStep(stepDirection);

    // Automatically set max step on the go
    if (stepDirection == SM_STEP_DIRECTION_UP && currentStep > (getMaxStep() / 2) && endpointReached()) {
        findMaxPosition();
    }
}

int8_t StepperMotor::getStepDirectionForTargetStep() const {
    return targetStep > currentStep ? SM_STEP_DIRECTION_UP : SM_STEP_DIRECTION_DOWN;
}

void StepperMotor::stopAllCoils() const {
    digitalWrite(coilA, LOW);
    digitalWrite(coilB, LOW);
    digitalWrite(coilC, LOW);
    digitalWrite(coilD, LOW);
}

void StepperMotor::findResetPosition() {
    Serial.println("[StepperMotor] Finding reset position");

    // Move towards endpoint
    while (!endpointReached()) {
        blindStep(SM_STEP_DIRECTION_DOWN);
    }

    // Slowly move back towards the edge of the endpoint
    do {
        // Add extra delay to make steps slow and thus the endpoint sensor more precise
        blindStep(SM_STEP_DIRECTION_UP, getStepDelay() * 6);
    } while (endpointReached());

    blindStep(SM_STEP_DIRECTION_UP);

    stopAllCoils();
    resetPositionValues();
}

void StepperMotor::findMaxPosition() {
    Serial.println("[StepperMotor] Finding max position");

    // Move towards endpoint
    while (!endpointReached()) {
        blindStep(SM_STEP_DIRECTION_UP);
    }

    // Slowly move back towards the edge of the endpoint
    do {
        blindStep(SM_STEP_DIRECTION_DOWN);

        // Add extra delay to make steps slow and thus the endpoint sensor more precise
        delay(10);
    } while (endpointReached());

    blindStep(SM_STEP_DIRECTION_DOWN);

    stopAllCoils();
    setMaxStep(currentStep);

    Serial.print("[StepperMotor] New max position: ");
    Serial.println(getMaxStep());
}

void StepperMotor::resetPositionValues() {
    Serial.println("[StepperMotor] Resetting position values");
    targetStep = 0;
    currentStep = 0;
}
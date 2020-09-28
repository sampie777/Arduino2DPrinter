#include <Arduino.h>
#include "StepperMotor.h"
#include "pins.h"
#include "StepperMotorBipolar.h"
#include "conversion.h"

StepperMotor stepperMotorX(SM_X_COIL_A,
                           SM_X_COIL_B,
                           SM_X_COIL_C,
                           SM_X_COIL_D,
                           SM_X_ENDPOINT_SENSOR_PIN);

StepperMotorBipolar stepperMotorY(SM_Y_COIL_A,
                                  SM_Y_COIL_B,
                                  SM_Y_COIL_C,
                                  SM_Y_COIL_D,
                                  SM_Y_ENDPOINT_SENSOR_PIN);


void handleSweep(StepperMotor *stpMotor) {
    if (!stpMotor->sweep) return;

    if (stpMotor->getCurrentStep() >= stpMotor->getMaxStep()) {
        Serial.println("[Main] Sweep down");
        stpMotor->targetStep = 0;
        return;
    } else if (stpMotor->getCurrentStep() == 0) {
        Serial.println("[Main] Sweep up");
        stpMotor->targetStep = stpMotor->getMaxStep();
        return;
    }
}

void flushSerialReadBuffer() {
    while (Serial.available()) {
        Serial.read();
    }
}

long getNumberFromSerialBuffer(const char *buffer, uint8_t startIndex, uint8_t length) {
    long value = 0;
    for (unsigned int i = startIndex; i < startIndex + length; i++) {
        uint16_t realValue = (uint8_t) buffer[i] - 48;
        uint8_t factor = (startIndex + length) - 1 - i;
        value += realValue * round(pow(10, factor));
    }
    return value;
}

void processSerialInput() {
    if (!Serial.available()) {
        return;
    }

    char buffer[8] = {0};
    uint8_t length = Serial.readBytesUntil('\n', buffer, 8);
    flushSerialReadBuffer();

    if (length == 0) {
        return;
    }

    if (length == 8) {
        if (buffer[0] != 'x' || buffer[4] != 'y') {
            Serial.println("Invalid coordinates. Expected value: 'x000y000'");
            return;
        }

        stepperMotorX.targetStep = mmToSteps(&stepperMotorX, getNumberFromSerialBuffer(buffer, 1, 3));
        stepperMotorY.targetStep = mmToSteps(&stepperMotorY, getNumberFromSerialBuffer(buffer, 5, 3));

        Serial.println("New target steps:");
        Serial.print("\tX = ");
        Serial.println(stepperMotorX.targetStep, DEC);
        Serial.print("\tY = ");
        Serial.println(stepperMotorY.targetStep, DEC);
    }

    if (length == 1 && (buffer[0] < 48 || buffer[0] > 57)) {
        if (buffer[0] == 'r') {
            Serial.println("Reset position parameters");
            stepperMotorX.resetPositionValues();
            stepperMotorY.resetPositionValues();
            return;
        }
        if (buffer[0] == 's') {
            Serial.print("Toggling sweep mode ");
            stepperMotorX.sweep = !stepperMotorX.sweep;
            stepperMotorY.sweep = !stepperMotorY.sweep;

            if (stepperMotorX.sweep) {
                Serial.println("on");
                stepperMotorX.targetStep = 0;
                stepperMotorY.targetStep = 0;
            } else {
                Serial.println("off");
                stepperMotorX.targetStep = stepperMotorX.getCurrentStep();
                stepperMotorY.targetStep = stepperMotorY.getCurrentStep();
            }
            return;
        }
        if (buffer[0] == 'f') {
            Serial.print("Finding start point position... ");
            stepperMotorX.findResetPosition();
            stepperMotorY.findResetPosition();
            Serial.println("done");
            return;
        }

        Serial.println("Invalid input");
        return;
    }
//
//    uint16_t newTargetStep = 0;
//    for (uint8_t i = 0; i < length; i++) {
//        uint16_t realValue = (uint8_t) buffer[i] - 48;
//        uint8_t factor = length - 1 - i;
//        newTargetStep += realValue * round(pow(10, factor));
//    }
//    stepperMotorX.targetStep = newTargetStep;
//    stepperMotorY.targetStep = newTargetStep;
//
//    Serial.print("New targetStep value: ");
//    Serial.println(stepperMotorX.targetStep, DEC);
}

void printSystemInfo() {
    Serial.print("System info:\nStepper Motor X\n\tsteps per revolution = ");
    Serial.print(SM_STEPS_PER_REVOLUTION);
    Serial.print("\n\tstep limit = ");
    Serial.print(stepperMotorX.getMaxStep());
    Serial.print("\n\tstep speed = ");
    Serial.print(stepperMotorX.getStepDelay());
    Serial.print(" micro seconds");
    Serial.print("\n\tendpoint sensor pin = ");
    Serial.println(SM_X_ENDPOINT_SENSOR_PIN);
    Serial.print("-----\nStepper Motor Y\n\tsteps per revolution = ");
    Serial.print(SM_STEPS_PER_REVOLUTION);
    Serial.print("\n\tstep limit = ");
    Serial.print(stepperMotorY.getMaxStep());
    Serial.print("\n\tstep speed = ");
    Serial.print(stepperMotorY.getStepDelay());
    Serial.print(" micro seconds");
    Serial.print("\n\tendpoint sensor pin = ");
    Serial.println(SM_Y_ENDPOINT_SENSOR_PIN);
}

void handleActionButton() {
    if (digitalRead(ACTION_BUTTON_1)) {
        return;
    }

    digitalWrite(LED_BUILTIN, HIGH);

    stepperMotorX.targetStep = stepperMotorX.getMaxStep() + 2000;

    // Eject X axis
    while (stepperMotorX.getCurrentStep() < stepperMotorX.getMaxStep()) {
        stepperMotorX.step();
    }

    // Wait till user confirms
    while (digitalRead(ACTION_BUTTON_1)) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(100);
        digitalWrite(LED_BUILTIN, LOW);
        delay(900);
    }

    // Give user some time to handle
    delay(4000);
    digitalWrite(LED_BUILTIN, HIGH);

    // Throw it out
    while (stepperMotorX.getCurrentStep() < stepperMotorX.targetStep) {
        stepperMotorX.step(true);
    }

    digitalWrite(LED_BUILTIN, LOW);

    // Do nothing
    while (true) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(100);
        digitalWrite(LED_BUILTIN, LOW);
        delay(900);
    }
}

void setup() {
    pinMode(ACTION_BUTTON_1, INPUT_PULLUP);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    stepperMotorX.setupSMPins();
    stepperMotorY.setupSMPins();

    Serial.begin(115200);
    Serial.println("Ready.");
    printSystemInfo();

    stepperMotorX.findResetPosition();
    stepperMotorY.findResetPosition();
}

void loop() {
    processSerialInput();
    handleActionButton();

    stepperMotorX.step();
    stepperMotorY.step();

    handleSweep(&stepperMotorX);
    handleSweep(&stepperMotorY);

    stepperMotorY.update();
}

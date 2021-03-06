#include <Arduino.h>
#include "StepperMotor.h"
#include "pins.h"
#include "StepperMotorBipolar.h"
#include "conversion.h"

StepperMotor stepperMotorX(SM_X_COIL_A,
                           SM_X_COIL_B,
                           SM_X_COIL_C,
                           SM_X_COIL_D,
                           SM_X_ENDPOINT_SENSOR_PIN,
                           'X');

StepperMotorBipolar stepperMotorY(SM_Y_COIL_A,
                                  SM_Y_COIL_B,
                                  SM_Y_COIL_C,
                                  SM_Y_COIL_D,
                                  SM_Y_ENDPOINT_SENSOR_PIN,
                                  'Y');


void handleSweep(StepperMotor *stpMotor) {
    if (!stpMotor->sweep) return;

    if (stpMotor->getCurrentStep() >= stpMotor->getMaxStep()) {
        Serial.print("[Main] ");
        Serial.print(stpMotor->name);
        Serial.println(": Sweep down");
        stpMotor->targetStep = 0;
        return;
    } else if (stpMotor->getCurrentStep() == 0) {
        Serial.print("[Main] ");
        Serial.print(stpMotor->name);
        Serial.println(": Sweep up");
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

    char buffer[10] = {0};
    uint8_t length = Serial.readBytesUntil('\n', buffer, 10);
    flushSerialReadBuffer();

    if (length == 0) {
        return;
    }

    if (length == 10) {
        if (buffer[0] != 'x' || buffer[5] != 'y') {
            Serial.print("[Serial] Invalid coordinates. Expected value: 'x0000y0000', but got: ");
            Serial.write(buffer);
            Serial.println();
            return;
        }

        float value = getNumberFromSerialBuffer(buffer, 1, 4) / 10.0;
        stepperMotorX.targetStep = mmToSteps(&stepperMotorX, value);

        value = getNumberFromSerialBuffer(buffer, 6, 4) / 10.0;
        stepperMotorY.targetStep = mmToSteps(&stepperMotorY, value);

        Serial.println("[Serial] New target steps:");
        Serial.print("\tX = ");
        Serial.println(stepperMotorX.targetStep, DEC);
        Serial.print("\tY = ");
        Serial.println(stepperMotorY.targetStep, DEC);

        if (stepperMotorX.getCurrentStep() == stepperMotorX.targetStep) {
            stepperMotorX.announceTargetReached();
        }
        if (stepperMotorY.getCurrentStep() == stepperMotorY.targetStep) {
            stepperMotorY.announceTargetReached();
        }

        // Disable sweep
        stepperMotorX.sweep = false;
        stepperMotorY.sweep = false;
        return;
    }

    if (buffer[0] < 48 || buffer[0] > 57) {
        if (buffer[0] == 'r') {
            Serial.println("[Serial] Reset position parameters");
            stepperMotorX.resetPositionValues();
            stepperMotorY.resetPositionValues();
            return;
        }
        if (buffer[0] == 's') {
            Serial.print("[Serial] Toggling sweep mode ");

            if (length > 1) {
                uint8_t sweep = getNumberFromSerialBuffer(buffer, 1, 1);
                stepperMotorX.sweep = sweep;
                stepperMotorY.sweep = sweep;
            } else {
                stepperMotorX.sweep = !stepperMotorX.sweep;
                stepperMotorY.sweep = !stepperMotorY.sweep;
            }

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
            Serial.println("[Serial] Finding start point position...");
            stepperMotorX.findResetPosition();
            stepperMotorY.findResetPosition();
            Serial.println("[Serial] done");
            return;
        }

        Serial.print("[Serial] Invalid input: ");
        Serial.write(buffer);
        Serial.println();
        return;
    }
}

void printInfoForMotor(StepperMotor *stpMotor) {
    Serial.print("Stepper Motor ");
    Serial.println(stpMotor->name);
    Serial.print("\tsteps per millimeter = ");
    Serial.println(stpMotor->getStepsPerMm());
    Serial.print("\tstep limit = ");
    Serial.println(stpMotor->getMaxStep());
    Serial.print("\tstep speed = ");
    Serial.print(stpMotor->getStepDelay());
    Serial.println(" micro seconds per step");
    Serial.print("\tmax distance = ");
    Serial.print(stpMotor->getMaxDistance());
    Serial.println(" mm");
    Serial.print("\tendpoint sensor pin = ");
    Serial.println(stpMotor->endpointSensor);
}

void printSystemInfo() {
    Serial.println("--------");
    Serial.println("System info:");
    printInfoForMotor(&stepperMotorX);
    printInfoForMotor(&stepperMotorY);
    Serial.println("--------");
}

void blinkEternally() {
    while (true) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(100);
        digitalWrite(LED_BUILTIN, LOW);
        delay(900);
    }
}

void handleActionButton() {
    if (digitalRead(ACTION_BUTTON_1)) {
        return;
    }

    Serial.println("[Main] Ejecting X surface");

    digitalWrite(LED_BUILTIN, HIGH);

    stepperMotorX.targetStep = stepperMotorX.getMaxStep() + 2000;

    // Ejecet X axis
    while (stepperMotorX.getCurrentStep() < stepperMotorX.targetStep) {
        stepperMotorX.step(true);
    }

    // Do nothing
    blinkEternally();
}

void setup() {
    Serial.begin(115200);
    Serial.println("[Main] Booting.");

    pinMode(ACTION_BUTTON_1, INPUT_PULLUP);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    stepperMotorX.setupSMPins();
    stepperMotorY.setupSMPins();

    printSystemInfo();

    stepperMotorX.findResetPosition();
    stepperMotorY.findResetPosition();

    Serial.println("[Main] Boot done.");
}

void loop() {
    // Inputs
    processSerialInput();
    handleActionButton();

    // Processing
    handleSweep(&stepperMotorX);
    handleSweep(&stepperMotorY);

    // Outputs
    stepperMotorX.step();
    stepperMotorY.step();
}

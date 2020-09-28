//
// Created by S. Jansen on 4/29/20.
//

#ifndef STEPPERMOTORPROJECT1_STEPPERMOTOR_H
#define STEPPERMOTORPROJECT1_STEPPERMOTOR_H

#include <Arduino.h>

#define SM_STEPS_PER_REVOLUTION 2052
#define SM_STEP_DIRECTION_UP 1
#define SM_STEP_DIRECTION_DOWN -1

class StepperMotor {
public:
    StepperMotor(uint8_t coilA,
                 uint8_t coilB,
                 uint8_t coilC,
                 uint8_t coilD,
                 uint8_t endpointSensor) : coilA(coilA),
                                           coilB(coilB),
                                           coilC(coilC),
                                           coilD(coilD),
                                           endpointSensor(endpointSensor) {}

    bool sweep = true;
    uint16_t targetStep = 0;

    void setupSMPins() const;

    virtual void step() { step(false); }

    virtual void step(uint8_t force);

    void findResetPosition();

    void resetPositionValues();

    void findMaxPosition();

    bool endpointReached() const;

    uint16_t getCurrentStep() const { return currentStep; }

    void setMaxStep(uint16_t value) { maxStep = value; }

    virtual uint16_t getMaxStep() const { return maxStep; }

    virtual uint16_t getStepDelay() const { return stepDelay; }

    virtual float getStepsPerMm() const { return maxStep / maxDistance; }

    void announceTargetReached();

protected:
    uint8_t coilA, coilB, coilC, coilD, endpointSensor;
    uint16_t currentStep = 0;
    uint16_t maxStep = 14000;
    uint16_t stepDelay = 2135;
    float maxDistance = 108.0;

    int8_t getStepDirectionForTargetStep() const;

    void blindStep(int8_t direction) { blindStep(direction, getStepDelay()); }

    virtual void blindStep(int8_t direction, uint16_t stepDelay);

    void stopAllCoils() const;
};


#endif //STEPPERMOTORPROJECT1_STEPPERMOTOR_H

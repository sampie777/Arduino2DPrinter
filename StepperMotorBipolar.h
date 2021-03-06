//
// Created by S. Jansen.
//

#ifndef STEPPERMOTORPROJECT1_STEPPERMOTORBIPOLAR_H
#define STEPPERMOTORPROJECT1_STEPPERMOTORBIPOLAR_H


#include "StepperMotor.h"

class StepperMotorBipolar : public StepperMotor {
public:
    StepperMotorBipolar(uint8_t coilA,
                        uint8_t coilB,
                        uint8_t coilC,
                        uint8_t coilD,
                        uint8_t endpointSensor,
                        char name) : StepperMotor(coilA,
                                                  coilB,
                                                  coilC,
                                                  coilD,
                                                  endpointSensor,
                                                  name) {}


    void step() override;

    uint16_t getMaxStep() const override { return maxStep; }

    uint16_t getStepDelay() const override { return stepDelay; }

    float getStepsPerMm() const override { return maxStep / maxDistance; }

private:
    uint16_t maxStep = 160;
    uint16_t stepDelay = 20000;
    float maxDistance = 36.1;

    unsigned long lastStepTime = 0;

    void blindStep(int8_t direction, uint16_t stepDelay) override;

};


#endif //STEPPERMOTORPROJECT1_STEPPERMOTORBIPOLAR_H

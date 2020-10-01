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
                        char name,
                        uint16_t maxStep,
                        unsigned long stepDelay,
                        float maxDistance) : StepperMotor(coilA,
                                                          coilB,
                                                          coilC,
                                                          coilD,
                                                          endpointSensor,
                                                          name,
                                                          maxStep,
                                                          stepDelay,
                                                          maxDistance) {}


    uint16_t getMaxStep() const override { return maxStep; }

    unsigned long getStepDelay() const override { return stepDelay; }

    float getStepsPerMm() const override { return maxStep / maxDistance; }

protected:
    void blindStep(int8_t direction, uint16_t stepDelay) override;
};


#endif //STEPPERMOTORPROJECT1_STEPPERMOTORBIPOLAR_H

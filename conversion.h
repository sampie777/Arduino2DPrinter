//
// Created by S. Jansen.
//

#ifndef ARDUINO2DPRINTER_CONVERSION_H
#define ARDUINO2DPRINTER_CONVERSION_H

#include <Arduino.h>
#include "StepperMotor.h"

long mmToSteps(StepperMotor *stpMotor, float value) {
    return value * stpMotor->getStepsPerMm();
}

#endif //ARDUINO2DPRINTER_CONVERSION_H

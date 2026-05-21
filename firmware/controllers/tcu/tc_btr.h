#pragma once

#include "tcu_controller.h"

// Define our specific static solenoid index allocations for readability
#define BTR_SOL_1 0
#define BTR_SOL_2 1
#define BTR_SOL_6 2
#define BTR_SOL_7 3

class BtrController : public BaseTcuController {
public:
    BtrController();
    
    // Core rusEFI lifecycle overrides
    void init() override;
    void update(efitick_t nowNt) override;
    
protected:
    // Overrides determining output pins based on current calculated target gear
    void assignSolenoidOutputs(int gear) override;
    void updateLinePressure(float throttlePosition, float outputSpeed) override;

private:
    // Keep track of our PWM variable line pressure output pin
    OutputPin vpsS5Output;
    
    // Shift logic map configuration tracking parameters
    bool isM97Configuration = true; // Default to M97 setup (Normally Open layout)
};

#pragma once

#include "tcu_controller.h"

// Static solenoid map array indexing offsets for clarity
#define BTR_SOL_1 0
#define BTR_SOL_2 1
#define BTR_SOL_3 2  // Clutch Regulator 
#define BTR_SOL_4 3  // Band Regulator
#define BTR_SOL_6 4  // Line Boost
#define BTR_SOL_7 5  // TCC Lockup

class BtrTransmissionController : public BaseTcuController {
public:
    BtrTransmissionController();
    
    // Core rusEFI lifecycle overrides
    void init() override;
    void update(efitick_t nowNt) override;
    
protected:
    // Lifecycle hooks for handling gear changes and line pressure variations
    void assignSolenoidOutputs(int gear) override;
    void updateLinePressure(float throttlePosition, float outputSpeed) override;

private:
    // PWM output driver pin structure for S5 VPS
    OutputPin vpsS5Output;
    
    // Configuration tracking parameters
    bool isM97Configuration = true; // True = M93/M97 (Normally Open layout), False = M85/M91
    bool isShiftingPhase = false; 
    int previousGear = 1;
};

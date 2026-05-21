#include "tc_btr.h"
#include "engine.h"
#include "tcu_common.h"

BtrTransmissionController::BtrTransmissionController() {
    isShiftingPhase = false;
    previousGear = 1;
}

void BtrTransmissionController::init() {
    BaseTcuController::init();
    
    // Register outputs using the shared rusEFI TCU driver mapping array
    initDigitalOutput(&solenoids[BTR_SOL_1], "BTR S1 Shift");
    initDigitalOutput(&solenoids[BTR_SOL_2], "BTR S2 Shift");
    initDigitalOutput(&solenoids[BTR_SOL_3], "BTR S3 Clutch Reg"); 
    initDigitalOutput(&solenoids[BTR_SOL_4], "BTR S4 Band Reg");   
    initDigitalOutput(&solenoids[BTR_SOL_6], "BTR S6 Line Boost");
    initDigitalOutput(&solenoids[BTR_SOL_7], "BTR S7 TCC Lockup");
    
    // Initialize the S5 VPS high-current frequency driver
    vpsS5Output.initPin("BTR S5 VPS Line Pressure", &engineConfiguration->tcuVpsPin);
}

void BtrTransmissionController::assignSolenoidOutputs(int gear) {
    bool s1State = false;
    bool s2State = false;
    bool s3State = false; 
    bool s4State = false; 
    bool s7State = false;

    // Monitor for actively requested shift phase transitions
    if (gear != previousGear) {
        isShiftingPhase = true;
    }

    // Apply exact BTR hydraulic selection truth table logic
    switch (gear) {
        case 1:
            s1State = true;  s2State = true;
            break;
            
        case 2:
            s1State = false; s2State = true;
            // Shift 1->2 applies front band. Direct S5 damping via S4 gating.
            if (isShiftingPhase) s4State = true; 
            break;
            
        case 3:
            s1State = false; s2State = false;
            // Shift 2->3 applies C1 clutch. Direct S5 damping via S3 gating.
            if (isShiftingPhase) s3State = true; 
            // EF/M97 allows economy 3rd gear torque converter lockup
            if (isM97Configuration && currentTcuMode == TCU_MODE_ECONOMY) s7State = true;
            break;
            
        case 4:
            s1State = true;  s2State = false;
            // Shift 3->4 updates bands. Direct S5 damping via S4 gating.
            if (isShiftingPhase) s4State = true;
            s7State = true; // Standard continuous 4th gear lockup behavior
            break;
            
        case TCU_GEAR_REVERSE:
            s1State = true;  s2State = true;
            break;
            
        default:
            s1State = false; s2State = false;
            break;
    }

    // Invert states for M93/M97 series where all solenoids are structural Normally Open
    if (isM97Configuration) {
        s1State = !s1State;
        s2State = !s2State;
        s3State = !s3State;
        s4State = !s4State;
        s7State = !s7State;
    }

    // Direct assignment to physical driver gates
    setPinState(&solenoids[BTR_SOL_1], s1State);
    setPinState(&solenoids[BTR_SOL_2], s2State);
    setPinState(&solenoids[BTR_SOL_3], s3State);
    setPinState(&solenoids[BTR_SOL_4], s4State);
    setPinState(&solenoids[BTR_SOL_7], s7State);
}

void BtrTransmissionController::updateLinePressure(float throttlePosition, float outputSpeed) {
    int currentGear = currentGearSelectorPosition;
    bool forceHighPressure = (currentGear == TCU_GEAR_REVERSE || throttlePosition > 65.0f);
    
    // S6 High-pressure override deployment
    bool s6State = forceHighPressure ? false : true;
    if (isM97Configuration) s6State = !s6State;
    setPinState(&solenoids[BTR_SOL_6], s6State);

    // Dynamic S5 VPS Duty Cycle processing
    // NOTE: Max Current (100% Duty) = Min Line Pressure. 0% Duty = Max Pressure.
    float targetDutyCycle = 100.0f;
    
    if (forceHighPressure) {
        targetDutyCycle = 0.0f; 
    } else if (isShiftingPhase) {
        // Damp shift transition with an intermediate line pressure jump
        targetDutyCycle = 30.0f; 
    } else {
        // Linear load-scaling curve mapping base throttle position
        targetDutyCycle = 100.0f - throttlePosition;
    }

    vpsS5Output.setPWMDutyCycle(targetDutyCycle);
}

void BtrTransmissionController::update(efitick_t nowNt) {
    float currentTps = getThrottlePosition();
    float currentOssSpeed = getOutputShaftSpeed();
    
    int targetGear = calculateTargetGear(currentTps, currentOssSpeed);
    
    assignSolenoidOutputs(targetGear);
    updateLinePressure(currentTps, currentOssSpeed);
    
    // Reset shifting state markers once target parameters match system verification values
    if (targetGear == currentActualGear) {
        isShiftingPhase = false;
        previousGear = targetGear;
    }
}

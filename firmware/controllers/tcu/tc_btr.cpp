#include "tc_btr.h"
#include "engine.h"
#include "tcu_common.h"

BtrController::BtrController() {
    // Constructor initialisation
}

void BtrController::init() {
    BaseTcuController::init();
    
    // Initialise your digital outputs via standard rusEFI pin mapping
    // S1, S2 = Shift Solenoids. S6 = Line Modifier. S7 = Lockup Clutch Control.
    // Note: Adjust the exact indexing arrays to match rusEFI's physical hardware definitions
    initDigitalOutput(&solenoids[BTR_SOL_1], "BTR S1 Shift");
    initDigitalOutput(&solenoids[BTR_SOL_2], "BTR S2 Shift");
    initDigitalOutput(&solenoids[BTR_SOL_6], "BTR S6 Line Pressure Boost");
    initDigitalOutput(&solenoids[BTR_SOL_7], "BTR S7 TCC Lockup");
    
    // S5 (VPS) is a variable high frequency PWM valve
    vpsS5Output.initPin("BTR S5 VPS Line Pressure", &engineConfiguration->tcuVpsPin);
}

void BtrController::assignSolenoidOutputs(int gear) {
    bool s1State = false;
    bool s2State = false;
    bool s7State = false;

    // Apply exact BTR Table 1 Solenoid Logic Map
    switch (gear) {
        case 1: // First Gear
            s1State = true;
            s2State = true;
            break;
        case 2: // Second Gear
            s1State = false;
            s2State = true;
            break;
        case 3: // Third Gear
            s1State = false;
            s2State = false;
            // M97 specific implementation logic allows lockup in 3rd economy profile
            if (isM97Configuration && currentTcuMode == TCU_MODE_ECONOMY) {
                s7State = true; 
            }
            break;
        case 4: // Fourth Gear
            s1State = true;
            s2State = false;
            s7State = true; // Lockup engaged standardly in 4th
            break;
        case TCU_GEAR_REVERSE:
            s1State = true;
            s2State = true;
            break;
        default: // Neutral/Park or fallback configurations
            s1State = false;
            s2State = false;
            break;
    }

    // Adjust outputs depending on hardware generation rules
    // Remember: M93/M97 series uses completely inverted "Normally Open" (NO) solenoids
    if (isM97Configuration) {
        s1State = !s1State;
        s2State = !s2State;
        s7State = !s7State; 
    }

    // Push states directly to hardware low side drivers
    setPinState(&solenoids[BTR_SOL_1], s1State);
    setPinState(&solenoids[BTR_SOL_2], s2State);
    setPinState(&solenoids[BTR_SOL_7], s7State);
}

void BtrController::updateLinePressure(float throttlePosition, float outputSpeed) {
    // Read current physical selector status
    int currentGear = currentGearSelectorPosition; 
    bool forceHighPressure = (currentGear == TCU_GEAR_REVERSE || throttlePosition > 65.0f);
    
    // Command S6 line modifier solenoid state
    // De-activating S6 triggers high line pressure behavior 
    bool s6State = forceHighPressure ? false : true; 
    if (isM97Configuration) s6State = !s6State; // Account for Normally Open logic shift
    setPinState(&solenoids[BTR_SOL_6], s6State);

    // Calculate Variable Pressure Solenoid (S5) Duty Cycle
    // INVERTED LOGIC CRITICAL NOTE: Max current = minimum transmission line pressure.
    // Zero current/Low duty cycle = Maximum transmission line holding pressure.
    float targetDutyCycle = 100.0f; // Default safely to lowest line pressure at idle
    
    if (forceHighPressure) {
        targetDutyCycle = 0.0f; // Defaults instantly to raw absolute mechanical line pressure ceiling
    } else {
        // Interpolate line pressure requirements linearly across intermediate load values
        // To do this optimally, map this target out to a multi-point standard 2D table layout in TunerStudio
        targetDutyCycle = 100.0f - throttlePosition; 
    }

    // Command the rusEFI hardware timer to alter output frequency
    vpsS5Output.setPWMDutyCycle(targetDutyCycle);
}

void BtrController::update(efitick_t nowNt) {
    // Core periodic cyclic execution calculations hook loop
    float currentTps = getThrottlePosition();
    float currentOssSpeed = getOutputShaftSpeed();
    
    // Process gear shifting schedule state maps
    int targetGear = calculateTargetGear(currentTps, currentOssSpeed);
    
    assignSolenoidOutputs(targetGear);
    updateLinePressure(currentTps, currentOssSpeed);
}

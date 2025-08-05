#include "tc_btr95le.h"

Btr95leTcu::Btr95leTcu() {}

void Btr95leTcu::update(const TcuInputs& in, TcuOutputs* out) {
    int gear = selectGear(in);
    setSolenoids(gear, in, out);
}

// Simple gear selection based on speed and shifter position.
// You should replace this logic with a proper shift map for production.
int Btr95leTcu::selectGear(const TcuInputs& in) {
    if (in.prndl == PRNDL::Reverse)
        return -1; // Reverse
    if (in.vehicle_speed < 20)
        return 1;
    if (in.vehicle_speed < 40)
        return 2;
    if (in.vehicle_speed < 70)
        return 3;
    return 4;
}

void Btr95leTcu::setSolenoids(int gear, const TcuInputs& in, TcuOutputs* out) {
    // S1 = solenoids[0], S2 = solenoids[1], S3/S4 = solenoids[2/3], S5 = pwm_solenoids[4]
    // S6 = solenoids[5], S7 = solenoids[6]
    float tpsNorm = in.throttle_position / 100.0f;

    // S1 and S2: gear selection
    out->solenoids[0] = (gear == 1 || gear == 2 || gear == -1); // S1
    out->solenoids[1] = (gear == 1 || gear == -1 || gear == 2); // S2

    // S3, S4: Shift quality - placeholder logic
    out->solenoids[2] = false; // S3 (adjust as needed)
    out->solenoids[3] = false; // S4 (adjust as needed)

    // S5: Line pressure control (PWM), higher load = higher pressure
    out->pwm_solenoids[4] = 1.0f - tpsNorm; // S5: basic inverse TPS

    // S6: Light/heavy throttle pressure (simple example)
    out->solenoids[5] = (tpsNorm < 0.25f); // S6 ON at low throttle

    // S7: Torque converter lockup
    out->solenoids[6] = (gear == 4 && in.vehicle_speed > 80); // S7 ON in 4th above 80km/h
}

// Factory function
TransmissionController* createBtr95leTcu() {
    return new Btr95leTcu();
}

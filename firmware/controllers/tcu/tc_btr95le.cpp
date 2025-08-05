#include "tcu.h"

class Btr95leTcu : public TransmissionController {
public:
    Btr95leTcu() {}

    // Called every TCU cycle
    void update(const TcuInputs& in, TcuOutputs* out) override {
        // Determine desired gear based on shift map
        int gear = selectGear(in);

        // Set solenoids according to gear & logic table
        setSolenoids(gear, in, out);
    }

private:
    int selectGear(const TcuInputs& in) {
        // Simple example: could use rpm, tps, speed for shift maps
        if (in.prndl == PRNDL::Reverse)
            return -1; // reverse
        // automatic upshift logic
        if (in.vehicle_speed < 20)
            return 1;
        if (in.vehicle_speed < 40)
            return 2;
        if (in.vehicle_speed < 70)
            return 3;
        return 4;
    }

    void setSolenoids(int gear, const TcuInputs& in, TcuOutputs* out) {
        // S1, S2 gear selection
        out->solenoids[0] = (gear == 1 || gear == 2 || gear == -1); // S1
        out->solenoids[1] = (gear == 1 || gear == -1 || gear == 2); // S2

        // S3, S4 use for quality (simple logic, could be improved → see shift quality requirements)
        out->solenoids[2] = /* logic for S3, e.g. during gear change */;
        out->solenoids[3] = /* logic for S4, e.g. during gear change */;

        // S5 line pressure PWM: higher load = higher pressure = lower duty
        float tpsNorm = in.throttle_position / 100.0f;
        out->pwm_solenoids[4] = 1.0f - tpsNorm; // S5. 0=100% PWM=higher pressure

        // S6 line pressure boost
        out->solenoids[5] = (tpsNorm < 0.25f); // ON at low throttle for lower pressure

        // S7 Torque Converter Lockup
        out->solenoids[6] = (gear == 4 && in.vehicle_speed > 80); // Lockup in 4th at cruise
    }
};

// Registration for rusEFI core
TransmissionController* createBtr95leTcu() {
    return new Btr95leTcu();
}

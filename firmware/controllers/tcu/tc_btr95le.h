#pragma once

#include "tcu.h"

// The BTR 95LE transmission controller interface for rusEFI TCU
class Btr95leTcu : public TransmissionController {
public:
    Btr95leTcu();
    void update(const TcuInputs& in, TcuOutputs* out) override;

private:
    int selectGear(const TcuInputs& in);
    void setSolenoids(int gear, const TcuInputs& in, TcuOutputs* out);
};

// Factory registration function (for use in tcu registration tables)
TransmissionController* createBtr95leTcu();

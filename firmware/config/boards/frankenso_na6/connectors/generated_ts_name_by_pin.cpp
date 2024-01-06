//DO NOT EDIT MANUALLY, let automation work hard.

// auto-generated by PinoutLogic.java based on config/boards/frankenso_na6/connectors/internal.yaml
// auto-generated by PinoutLogic.java based on config/boards/frankenso_na6/connectors/main.yaml
#include "pch.h"

// see comments at declaration in pin_repository.h
const char * getBoardSpecificPinName(brain_pin_e brainPin) {
	switch(brainPin) {
		case Gpio::B7: return "Injector 3Z";
		case Gpio::B8: return "Injector 3Y";
		case Gpio::B9: return "Injector 3W";
		case Gpio::C13: return "Injector 2M";
		case Gpio::C7: return "Coil 1H";
		case Gpio::C9: return "Coil 1F";
		case Gpio::D3: return "Injector 3U";
		case Gpio::D5: return "Injector 3X";
		case Gpio::D7: return "Injector 2N";
		case Gpio::D8: return "Coil 1O";
		case Gpio::D9: return "Coil 1P";
		case Gpio::E10: return "Coil 1I";
		case Gpio::E12: return "Coil 1M";
		case Gpio::E14: return "Coil 1G";
		case Gpio::E2: return "Injector 3V";
		case Gpio::E3: return "Injector 3S";
		case Gpio::E4: return "Injector 3T";
		case Gpio::E5: return "Injector 2O";
		case Gpio::E6: return "Injector 2P";
		case Gpio::E8: return "Coil 1L";
		default: return nullptr;
	}
	return nullptr;
}

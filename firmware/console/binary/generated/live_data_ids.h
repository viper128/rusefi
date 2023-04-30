// generated by gen_live_documentation.sh / LiveDataProcessor.java
#pragma once

typedef enum {
LDS_output_channels,
LDS_fuel_computer,
LDS_ignition_state,
LDS_knock_controller,
LDS_throttle_model,
LDS_high_pressure_fuel_pump,
LDS_injector_model,
LDS_launch_control_state,
LDS_antilag_system_state,
LDS_boost_control,
LDS_ac_control,
LDS_fan_control,
LDS_fuel_pump_control,
LDS_main_relay,
LDS_engine_state,
LDS_tps_accel_state,
LDS_trigger_central,
LDS_trigger_state,
LDS_trigger_state_primary,
LDS_wall_fuel_state,
LDS_idle_state,
LDS_electronic_throttle,
LDS_wideband_state,
LDS_dc_motors,
LDS_sent_state,
} live_data_e;
#define OUTPUT_CHANNELS_BASE_ADDRESS 0
#define FUEL_COMPUTER_BASE_ADDRESS 776
#define IGNITION_STATE_BASE_ADDRESS 824
#define KNOCK_CONTROLLER_BASE_ADDRESS 872
#define THROTTLE_MODEL_BASE_ADDRESS 888
#define HIGH_PRESSURE_FUEL_PUMP_BASE_ADDRESS 888
#define INJECTOR_MODEL_BASE_ADDRESS 916
#define LAUNCH_CONTROL_STATE_BASE_ADDRESS 928
#define ANTILAG_SYSTEM_STATE_BASE_ADDRESS 936
#define BOOST_CONTROL_BASE_ADDRESS 948
#define AC_CONTROL_BASE_ADDRESS 972
#define FAN_CONTROL_BASE_ADDRESS 984
#define FUEL_PUMP_CONTROL_BASE_ADDRESS 988
#define MAIN_RELAY_BASE_ADDRESS 992
#define ENGINE_STATE_BASE_ADDRESS 996
#define TPS_ACCEL_STATE_BASE_ADDRESS 1116
#define TRIGGER_CENTRAL_BASE_ADDRESS 1164
#define TRIGGER_STATE_BASE_ADDRESS 1216
#define TRIGGER_STATE_PRIMARY_BASE_ADDRESS 1240
#define WALL_FUEL_STATE_BASE_ADDRESS 1244
#define IDLE_STATE_BASE_ADDRESS 1252
#define ELECTRONIC_THROTTLE_BASE_ADDRESS 1292
#define WIDEBAND_STATE_BASE_ADDRESS 1344
#define DC_MOTORS_BASE_ADDRESS 1356
#define SENT_STATE_BASE_ADDRESS 1368

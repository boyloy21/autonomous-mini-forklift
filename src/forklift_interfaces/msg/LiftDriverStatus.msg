# forklift_interfaces/msg/LiftDriverStatus.msg
# Published by lift_driver_node on /lift_driver_status.

bool connected       # serial/CAN link to STM32 is alive
bool fault           # driver-reported fault (overcurrent, stall, comms timeout...)
string message
float32 motor_current   # amps, 0 if unavailable/simulated

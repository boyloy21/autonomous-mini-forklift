# forklift_interfaces/msg/LiftEncoderHeight.msg
# Published by lift_driver_node on /encoder_height (raw reading from STM32).

float32 height       # meters, derived from encoder ticks
int64 raw_ticks      # raw encoder count, for diagnostics/calibration
builtin_interfaces/Time stamp

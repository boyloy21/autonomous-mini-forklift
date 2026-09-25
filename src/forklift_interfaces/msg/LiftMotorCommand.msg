# forklift_interfaces/msg/LiftMotorCommand.msg
# Published by lift_controller_node on /lift_motor_cmd.
# Signed velocity: positive = raise, negative = lower, 0 = hold.

float32 velocity   # m/s
bool enable        # false = cut motor power immediately (hard stop)

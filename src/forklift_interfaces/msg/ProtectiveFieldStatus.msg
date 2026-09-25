# forklift_interfaces/msg/ProtectiveFieldStatus.msg
# Published by protective_field_node on /protective_field.
# Zones nest: stop_zone implies slow_zone implies warning_zone.

bool warning_zone
bool slow_zone
bool stop_zone
float32 closest_distance
string message

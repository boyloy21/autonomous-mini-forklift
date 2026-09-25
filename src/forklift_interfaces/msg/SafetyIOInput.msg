# forklift_interfaces/msg/SafetyIOInput.msg
# Raw hardware safety inputs, subscribed by safety_io_node on /safety_input.

bool door_switch_closed
bool light_curtain_clear
bool safety_relay_ok
bool external_ok

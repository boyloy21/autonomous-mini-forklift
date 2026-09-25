# forklift_interfaces/msg/DiagnosticsAlert.msg
# Published by diagnostics_manager_node on /diagnostics/error and
# /diagnostics/warning (same type, different topic per severity).

string source    # e.g. "hardware.battery_voltage", "communication.can"
string code
string message
builtin_interfaces/Time stamp

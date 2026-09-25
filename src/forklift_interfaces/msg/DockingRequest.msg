# forklift_interfaces/msg/DockingRequest.msg
# Published by task_executor_node on /execute_docking.
# docking_client_node translates this into a DockRobot or UndockRobot
# action goal (undock=true selects UndockRobot).

uint8 DOCK_CHARGING_STATION=0
uint8 DOCK_PALLET_PICK=1
uint8 DOCK_PALLET_DROP=2

string task_id
string station_id
uint8 dock_type
bool undock

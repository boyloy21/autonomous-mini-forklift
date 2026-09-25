# forklift_interfaces/msg/UltrasonicArray.msg
# Published by whatever bridges the ultrasonic ring on /ultrasonic.

float32[] ranges   # meters, one per sensor
string[] labels     # e.g. "front_left", "front_right", "rear_left", "rear_right"

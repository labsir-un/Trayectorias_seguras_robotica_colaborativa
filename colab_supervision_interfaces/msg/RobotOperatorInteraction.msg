bool operator_detected # Deteccion of the operator in the scene

float32 distance # Distance between the operator and the robot

float32 operator_velocity # Velocity of the operator
float32 robot_velocity # Velocity of the robot
float32 relative_velocity # Relative velocity between the operator and the robot

float32 approach_velocity # Velocity of the operator approaching the robot

float32 time_to_collision # Time to collision between the operator and the robot, based on the relative velocity and distance

string interaction_state # State of the interaction between the operator and the robot
string risk_level # Level of risk associated with the interaction
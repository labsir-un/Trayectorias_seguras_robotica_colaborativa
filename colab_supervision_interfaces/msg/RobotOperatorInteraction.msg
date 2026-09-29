# Estado integrado de percepción e interacción operador-robot

std_msgs/Header header

bool operator_detected
bool robot_detected

bool operator_position_valid
bool robot_visual_position_valid
bool robot_reference_valid

float32 operator_confidence
float32 robot_confidence

geometry_msgs/Point operator_position
geometry_msgs/Point robot_visual_position
geometry_msgs/Point robot_reference_position

geometry_msgs/Vector3 operator_velocity
geometry_msgs/Vector3 robot_velocity

float32 distance
float32 relative_velocity
float32 approach_velocity

float32 time_to_collision
bool time_to_collision_valid

float32 collision_risk_score
string collision_risk
bool collision_imminent

bool interaction_valid
string status

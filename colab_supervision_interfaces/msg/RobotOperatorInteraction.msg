# Estado integrado de percepción e interacción operador-robot

std_msgs/Header header

# Detección 2D
bool operator_detected
bool robot_detected

# Validez de las posiciones 3D obtenidas mediante RGB-D
bool operator_position_valid
bool robot_position_valid

# Confianza de las detecciones 2D
float32 operator_confidence
float32 robot_confidence

# Posiciones 3D expresadas en el marco base del robot [m]
geometry_msgs/Point operator_position
geometry_msgs/Point robot_position

# Velocidades lineales estimadas [m/s]
geometry_msgs/Vector3 operator_velocity
geometry_msgs/Vector3 robot_velocity

# Variables de interacción y riesgo
float32 distance
float32 relative_velocity
float32 approach_velocity

float32 time_to_collision
bool time_to_collision_valid

float32 collision_risk_score
string collision_risk
bool collision_imminent

# Estado global de la evaluación
bool interaction_valid
string status

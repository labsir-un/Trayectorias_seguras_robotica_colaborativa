# Estado de interacción operador-robot preprocesado por control_supervisor.
#
# Este mensaje no contiene una decisión de actuación sobre el robot.
# Su función es entregar a control_decision información temporalmente
# acondicionada, evitando que este último consuma directamente los datos
# provenientes del procesamiento primario de las cámaras.

# Disponibilidad/detección
bool operator_detected
bool robot_detected

# Validez espacial de la observación actual
bool operator_position_valid
bool robot_position_valid
bool interaction_valid

# Confianzas de detección
float32 operator_confidence
float32 robot_confidence

# Distancia operador-robot
float32 distance
bool distance_valid
bool distance_from_memory

# Variables cinemáticas ya calculadas por robot_operator_interaction
float32 relative_velocity
float32 approach_velocity
float32 time_to_collision
bool time_to_collision_valid

# Riesgo ya calculado por robot_operator_interaction
float32 collision_risk_score
string collision_risk
bool collision_imminent

# Indicadores temporales calculados por control_supervisor
bool warning_in_window
bool danger_persistent

# Estado descriptivo del procesamiento supervisor
# Valores previstos:
#   VALID
#   DISTANCE_MEMORY
#   OPERATOR_NOT_DETECTED
#   ROBOT_NOT_DETECTED
#   INVALID_INTERACTION
string supervision_state

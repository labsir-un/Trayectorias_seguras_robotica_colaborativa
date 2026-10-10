# Métricas experimentales de la operación colaborativa
std_msgs/Header header

# 1. Distancia operador-robot
float32 distance_or_mm
bool distance_or_valid

# 2. Velocidad del robot estimada mediante RGB-D
float32 robot_speed_mm_s
bool robot_speed_valid

# 3. Variación de la posición relativa operador-robot
float32 relative_position_variation_mm
bool relative_position_variation_valid

# 4. Tiempo de adaptación de trayectoria
float32 adaptation_time_ms
bool adaptation_time_valid

# 5. Tiempo de procesamiento/latencia de percepción
float32 perception_time_ms
bool perception_time_valid

# 6. Tiempo de detención
float32 stop_time_ms
bool stop_time_valid

# 7. Factibilidad de trayectoria
bool trajectory_feasible
bool trajectory_feasible_valid

# 8. Calidad de percepción durante la ventana configurada
float32 perception_quality_percent
bool perception_quality_valid

# Indicadores auxiliares
bool operator_observation_valid
bool robot_observation_valid
bool adaptation_active
bool stop_active

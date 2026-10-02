# true cuando el operador ha sido detectado y visible
# durante los cinco frames de evaluación.
bool operator_present

# Condición predominante de atención:
# TASK / HMI / OTHER / UNKNOWN
string attention_condition

# Puntuación media de atención [0.0, 1.0]
float32 attention_score

# Nivel predominante de fatiga:
# LOW / MEDIUM / HIGH / UNKNOWN
string fatigue_condition

# Puntuación media de fatiga [0.0, 1.0]
float32 fatigue_score
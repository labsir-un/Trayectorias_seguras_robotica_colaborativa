# ============================================================
# OperatorPerception.msg
# Estado perceptual del operador
# ============================================================

bool person_detected
# True: se detectó una persona en la escena

bool person_visible
# True: se detectó una persona y existe una observación facial válida

float32 eye_openness
# Apertura ocular normalizada [0.0, 1.0]

float32 blink_frequency
# Frecuencia de parpadeo en blinks/min

bool blink_detected
# True durante la detección de un parpadeo

string facial_expression
# Expresión facial detectada:
# neutral, smile, frown, surprise, sleepy, tired, fatigued, yawn, unknown

float32 head_yaw
# Ángulo yaw de la cabeza en radianes
# positivo: derecha / negativo: izquierda

float32 head_pitch
# Ángulo pitch de la cabeza en radianes
# positivo: arriba / negativo: abajo

float32 head_roll
# Ángulo roll de la cabeza en radianes
# positivo: derecha / negativo: izquierda

float32 head_nodding_score
# Indicador normalizado de cabeceo [0.0, 1.0]

string attention_target
# Objetivo de atención: task, HMI, other, unknown

float32 attention_score
# Nivel de atención normalizado [0.0, 1.0]

float32 fatigue_score
# Nivel de fatiga normalizado [0.0, 1.0]

string fatigue_level
# Nivel de fatiga: LOW, MEDIUM, HIGH, UNKNOWN

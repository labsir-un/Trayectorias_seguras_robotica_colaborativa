bool person_detected # Deteccion of a person in the scene
bool person_visible # Deteccion of a person in the scene with a clear view of the face

float32 eye_openness # 0.0 to 1.0 porcentage of eye openness
float32 blink_frequency # 0.0 to 1.0 porcentage of blink frequency

string facial_expression # facial expression detected in the scene (neutral, happy, sad, angry, surprised, disgusted, fearful)

float32 head_yaw # angle of the head in radians, positive to the right, negative to the left
float32 head_pitch # angle of the head in radians, positive up, negative down
float32 head_roll # angle of the head in radians, positive to the right, negative to the left

string attention_target # target of the attention of the operator (task, HMI, other)
float32 attention_score # 0.0 to 1.0 porcentage of attention score

float32 fatigue_score # 0.0 to 1.0 porcentage of fatigue score, based on eye openness, blink frequency and head pose
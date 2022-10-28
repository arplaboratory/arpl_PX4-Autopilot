# Motor control message
uint64 timestamp			# time since system start (microseconds)

float32 thrust # thrust desired
float32[4] q_d			# Desired quaternion for quaternion control

float32 roll_body		# body angle in NED frame
float32 pitch_body		# body angle in NED frame
float32 yaw_body		# body angle in NED frame

float32 roll_body_rate		# body angle in NED frame
float32 pitch_body_rate		# body angle in NED frame
float32 yaw_body_rate		# body angle in NED frame
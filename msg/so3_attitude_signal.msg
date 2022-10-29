# Motor control message
uint64 timestamp			# time since system start (microseconds)

float32[3] kr_er # gain amplified attitude signal for geometric attitude controller

float32[3] rates_sp # rates set point after being transformed
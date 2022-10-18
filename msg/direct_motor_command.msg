# Motor control message
uint64 timestamp			# time since system start (microseconds)

uint8 num_controls = 4
float32[4] controls # range: [0, 1], where 1 means maximum positive thrust,
                    #  <0 maps to NaN,
                    # and NaN maps to disarmed (stop the motors)
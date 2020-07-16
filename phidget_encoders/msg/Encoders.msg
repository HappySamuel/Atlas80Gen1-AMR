time stamp

# time since previous message (as measured by the encoders)
float64 dt

# raw deltas on encoders
int64 d_count_left
int64 d_count_right

# corresponding traveled distance by each wheel
float64 d_left
float64 d_right

# corresponding traveled distance by the car
float64 d_dist
float64 d_th

# corresponding velocities
float64 v
float64 w
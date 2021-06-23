# Autopilot state enums
uint8 OFF=0
uint8 START=1
uint8 HOVER=2
uint8 LAND=3
uint8 EMERGENCY_LAND=4
uint8 BREAKING=5
uint8 GO_TO_POSE=6
uint8 VELOCITY_CONTROL=7
uint8 REFERENCE_CONTROL=8
uint8 TRAJECTORY_CONTROL=9
uint8 COMMAND_FEEDTHROUGH=10
uint8 RC_MANUAL=11


Header header

# Autopilot state as defined above. This reflects what is implemented in
# autopilot/include/autopilot/autopilot.h
uint8 autopilot_state

# Control command delay
duration control_command_delay

# Controller computation time [s]
duration control_computation_time

# Duration left of the trajectories in the queue
# Only valid in TRAJECTORY_CONTROL mode
duration trajectory_execution_left_duration

# Number of trajectories that were sent to the autopilot and are stored in its
# queue. Only valid in TRAJECTORY_CONTROL mode
uint8 trajectories_left_in_queue

# Low level feedback
quadrotor_msgs/LowLevelFeedback low_level_feedback

# Desired state used to compute the control command
quadrotor_msgs/TrajectoryPoint reference_state

# State estimate used to compute the control command
nav_msgs/Odometry state_estimate

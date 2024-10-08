# All of the data that can obtained from CAN
std_msgs/Header header

StateMachineState state_machine_state
float64 steering_angle_feedback
float64 vehicle_speed_feedback
float64 braking_feedback
float64 revolver_moment_feedback
float64 throttle_feedback

float64 state_of_charge # in Volts (V)
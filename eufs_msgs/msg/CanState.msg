# State of the Autonomous System
uint16 as_state

uint16 AS_OFF=0
uint16 AS_READY=1
uint16 AS_DRIVING=2
uint16 AS_EMERGENCY_BRAKE=3
uint16 AS_FINISHED=4

# Mission indicator
uint16 ami_state

uint16 AMI_NOT_SELECTED=10
uint16 AMI_PID_TEST=11
uint16 AMI_POINT_MPC=12
uint16 AMI_MANUAL=21

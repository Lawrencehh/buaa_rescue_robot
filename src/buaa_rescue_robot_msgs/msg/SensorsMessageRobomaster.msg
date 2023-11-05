int32[12] snake_motor_encorder_position # motor encorder of snake (65536 pulses per rotation)
int16 gripper_gm6020_position # gm6020 position feedback (360 per round)
int16 gripper_c610_position   # c610 position feedback (360*36 per round, the reduction ratio is 36:1)
int16 gripper_sts3032_position # sts3032 position feedback (360 per round)
int16 robomaster_reset # Reset state for robomaster motor encorders
# RoboMaster_1
uint8[12] snake_speed_control_1_array  # 固定长度为12的int32数组, RPM
int32[12] snake_position_control_1_array  # 固定长度为12的int32数组, pulse
int16 gripper_gm6020_position_1 # GM6020 postion control: 0-8091
int16 gripper_c610_position_1 # C610 postion control: 0-8091
int16 gripper_sts3032_position_1 # STC3032 postion control: 0-4095
int16 robomaster_1_mode # Mode control for robomaster 1 motor encorders

# RoboMaster_2
uint8[12] snake_speed_control_2_array  # 固定长度为12的int32数组, RPM
int32[12] snake_position_control_2_array  # 固定长度为12的int32数组, pulse
int16 gripper_gm6020_position_2 # GM6020 postion control: 0-8091
int16 gripper_c610_position_2 # C610 postion control: 0-8091
int16 gripper_sts3032_position_2 # STC3032 postion control: 0-4095
int8 robomaster_2_mode # Mode control for robomaster 2 motor encorders

# Master device
int16 elevator_control
int32 lower_linear_module_control
int32 upper_linear_module_control
int16 pull_push_sensors_reset # reset control for pull push sensors
int16 elevator_counter_reset # reset elevator counter
int16 lower_linear_module_encorder_reset # reset lower_linear_module_encorder
int16 upper_linear_module_encorder_reset # reset upper_linear_module_encorder

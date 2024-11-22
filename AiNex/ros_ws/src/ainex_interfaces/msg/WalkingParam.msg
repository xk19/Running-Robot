####### walking init pose #######
float32 init_x_offset
float32 init_y_offset
float32 init_z_offset
float32 init_roll_offset
float32 init_pitch_offset
float32 init_yaw_offset

####### time parameter #####
float32 period_time
float32 dsp_ratio
float32 step_fb_ratio

########## walking parameter ########
float32 x_move_amplitude
float32 y_move_amplitude
float32 z_move_amplitude
float32 angle_move_amplitude
bool move_aim_on
float32 arm_swing_gain

########## balance parameter ##########
bool balance_enable
float32 balance_hip_roll_gain
float32 balance_knee_gain
float32 balance_ankle_roll_gain
float32 balance_ankle_pitch_gain
float32 y_swap_amplitude
float32 z_swap_amplitude
float32 pelvis_offset
float32 hip_pitch_offset
float32 fb_p
float32 rl_p
float32 fb_i
float32 rl_i
float32 fb_d
float32 rl_d

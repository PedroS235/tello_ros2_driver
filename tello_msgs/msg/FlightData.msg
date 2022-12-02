# ============================================================
# |                    Fligh data msg type.                  |
# | Contains all the information that is sent from the drone |
# ============================================================

# =========================================================================

# - Battery data
int16 battery_low
int16 battery_lower
int16 battery_percentage
int16 drone_battery_left
int16 drone_fly_time_left

# =========================================================================

# - States
int16 battery_state
int16 camera_state
int16 electrical_machinery_state
int16 down_visual_state
int16 gravity_state
int16 imu_calibration_state
int16 imu_state
int16 power_state
int16 pressure_state
int16 wind_state

# =========================================================================

# - Stats
int16 drone_hover
int16 em_open
int16 em_sky
int16 em_ground
int16 factory_mode
int16 fly_mode
int16 fly_time
int16 front_in
int16 front_lsc
int16 front_out

# =========================================================================

# - Sensors
int16 fly_speed
int16 east_speed
int16 ground_speed
int16 height
int16 light_strength
int16 north_speed
int16 temperature_height

# =========================================================================

# - Other
int16 outage_recording
int16 smart_video_exit_mode
int16 throw_fly_timer

# =========================================================================

# - WiFi
int16 wifi_disturb
int16 wifi_strength

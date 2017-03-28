Header header

# Voltage of platform components, in volts
float64 battery_voltage

# Component temperatures, in C
float64 platform_temp

# drive pwm for each wheel in%
int8 lf_drive_pwm
int8 rf_drive_pwm
int8 lr_drive_pwm
int8 rr_drive_pwm

# turn pwm for each wheel in%
int8 lf_turn_pwm
int8 rf_turn_pwm
int8 lr_turn_pwm
int8 rr_turn_pwm

# error condition for each wheel
int8 lf_error
int8 rf_error
int8 lr_error
int8 rr_error

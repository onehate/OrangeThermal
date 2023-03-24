import logging

########################################################################
#
#   General options

### Logging
log_level = logging.INFO
log_format = '%(asctime)s %(levelname)s %(name)s: %(message)s'

### Server
listening_ip = "0.0.0.0"
listening_port = 8081

### Cost Estimate
kwh_rate        = 0.10  # Rate in currency_type to calculate cost to run job
currency_type   = "EUR"   # Currency Symbol to show when calculating cost to run job
oven_power      = 2000  # Average watts consumed by oven while running

########################################################################
#
#   GPIO Setup (Orange Pi Zero 2 GPIO.BOARD numbering)
#


##Enabled outputs/inputs
heat_enabled = True		# Enable control for heater
cool_enabled = False	# Enable control for cooler (exterior fan, etc.)
air_enabled = False		# Enable control for internal air circulation (interior fan)
door_enabled = False	# Enable sensor for door open


### Outputs
gpio_heat = 7  # Switches zero-cross SSR
gpio_cool = 11  # Regulates PWM for 12V DC Blower
gpio_air  = 13   # Switches 0-phase det. SSR

heater_invert = 0 # switches the polarity of the heater control

### Inputs
gpio_door = 12

### Thermocouple Connection (using bitbang interfaces)
gpio_sensor_cs = 24
gpio_sensor_miso = 21
gpio_sensor_mosi = 19
gpio_sensor_clock = 23

### amount of time, in seconds, to wait between reads of the thermocouple
# This is also the rate of the control loop?
sensor_read_period = 0.5

### Profile Adjustments for Kilns
# must_hit_temp adjusts for systems where the heater might not be able to keep up with the profile
# If false, the system will not adjust the timing of the profile
# If true, the system is guaranteed to hit the temperatures of the profile. It will wait until
# the target is reached before moving to the next segment of the profile
must_hit_temp = False

# Cone slope adj adjusts the target temperature when the segment takes longer than expected
# It's expressed in deg C per (deg C per hour), i.e. the shift in temperature target per shift in temperature rate
# For instance, setting to 0.4 will reduce the target temperature by 40 deg C if the final slope is 100 deg C/hour
# lower than the profile slope
# Only used if must_hit_temp is True
cone_slope_adj = 0.4

#the number of cycles the PID autone will complete before setting PID
tune_cycles = 5

#the temperature the autotune will use for calculation of PID
tune_target_temp = 200

########################################################################
#
#   PID parameters

pid_ki = 0.00009702  # Integration
pid_kd = 0.05007812  # Derivative
pid_kp = 0.00440847  # Proportional

#Tried-and-True
#pid_ki = 0.00011678  # Integration
#pid_kd = 0.09814581  # Derivative
#pid_kp = 0.00677092  # Proportional

########################################################################
#
#   Time and Temperature parameters

## These are defaults for new profiles, and can be modified on a per-profile basis

temp_scale          = "c" # c = Celsius | f = Fahrenheit - Unit to display 
time_scale_slope    = "s" # s = Seconds | m = Minutes | h = Hours - Slope displayed in temp_scale per time_scale_slope
time_scale_profile  = "s" # s = Seconds | m = Minutes | h = Hours - Enter and view target time in time_scale_profile

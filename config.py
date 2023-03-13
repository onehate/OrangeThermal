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
currency_type   = "USD"   # Currency Symbol to show when calculating cost to run job
oven_power      = 5000  # Average watts consumed by oven while running

########################################################################
#
#   GPIO Setup (BCM SoC Numbering Schema)
#
#   Check the RasPi docs to see where these GPIOs are
#   connected on the P1 header for your board type/rev.
#   These were tested on a Pi B Rev2 but of course you
#   can use whichever GPIO you prefer/have available.

##Enabled outputs
heat_enabled = True		# Enable control for heater
heat2_enabled = True	# Enabled control for second heater
cool_enabled = False	# Enable control for cooler (exterior fan, etc.)
air_enabled = False		# Enable control for internal air circulation (interior fan)

### Outputs
gpio_heat = 11  # Switches zero-cross solid-state-relay
gpio_heat2 = 12 # Second heater control
gpio_cool = 10  # Regulates PWM for 12V DC Blower
gpio_air  = 9   # Switches 0-phase det. solid-state-relay

heater_invert = 0 # switches the polarity of the heater control

### Inputs
door_enabled = False	# Enable sensor for door open
gpio_door = 18

### Thermocouple Adapter selection:
#   max31855 - bitbang SPI interface
#   max31855spi - kernel SPI interface
#   max6675 - bitbang SPI interface
max31855 = 1
max6675 = 0
max31855spi = 0 # Consumes pins 7,8,9,10,11

### Thermocouple Connection (using bitbang interfaces)
gpio_sensor_cs = 27
gpio_sensor_clock = 22
gpio_sensor_data = 17

### Thermocouple SPI Connection (using adafrut drivers + kernel SPI interface)
spi_sensor_chip_id = 0

### amount of time, in seconds, to wait between reads of the thermocouple
# This is also the rate of the control loop?
sensor_read_period = 1


# PWM Settings
### Default period of PWM, in seconds
PWM_Period_s = 30

# The period will be extended to meet the demanded duty cycle for DC close to 0 or 100%
# without violating the minimum onoff time, until the period reaches its max, at which point
# the system will go full off or on
# This allows more precise PWM near 0 or 1 without causing rapid relay switching

### Minimum On or Off time, in seconds. 
PWM_MinimumOnOff_s = 1

### Maximum allowed extended period
PWM_PeriodMax_s = 90

# PWM Offset Adjustment 
# For systems with multiple heaters and uneven heating, these factors will adjust the PWM to compensate
# For instance, if heater 1 needs to be on more often than heater 2, make heat1adj positive and heat2adj negative

heat1adj = 0		# heater 1 PWM offset, in percent
heat2adj = 0		# heater 2 PWM offset, in percent

### Profile Adjustments for Kilns
# must_hit_temp adjusts for systems where the heater might not be able to keep up with the profile
# If false, the system will not adjust the timing of the profile
# If true, the system is guaranteed to hit the temperatures of the profile. It will wait until
# the target is reached before moving to the next segment of the profile
must_hit_temp = True

# Cone slope adj adjusts the target temperature when the segment takes longer than expected
# It's expressed in deg C per (deg C per hour), i.e. the shift in temperature target per shift in temperature rate
# For instance, setting to 0.4 will reduce the target temperature by 40 deg C if the final slope is 100 deg C/hour
# lower than the profile slope
# Only used if must_hit_temp is True
cone_slope_adj = 0.4



########################################################################
#
#   PID parameters

pid_ki = 0.1  # Integration
pid_kd = 0.4  # Derivative
pid_kp = 0.5  # Proportional


########################################################################
#
#   Simulation parameters

sim_t_env      = 25.0   # deg C
sim_c_heat     = 100.0  # J/K  heat capacity of heat element
sim_c_oven     = 2000.0 # J/K  heat capacity of oven
sim_p_heat     = 3500.0 # W    heating power of oven
sim_R_o_nocool = 1.0    # K/W  thermal resistance oven -> environment
sim_R_o_cool   = 0.05   # K/W  " with cooling
sim_R_ho_noair = 0.1    # K/W  thermal resistance heat element -> oven
sim_R_ho_air   = 0.05   # K/W  " with internal air circulation


########################################################################
#
#   Time and Temperature parameters

## These are defaults for new profiles, and can be modified on a per-profile basis

temp_scale          = "c" # c = Celsius | f = Fahrenheit - Unit to display 
time_scale_slope    = "s" # s = Seconds | m = Minutes | h = Hours - Slope displayed in temp_scale per time_scale_slope
time_scale_profile  = "s" # s = Seconds | m = Minutes | h = Hours - Enter and view target time in time_scale_profile


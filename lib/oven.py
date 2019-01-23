##Comments - PT 1/23/19

import threading
import time
import random
import datetime
import logging
import json

import config

##This stuff gets done on import. Probably okay, but consider moving to init statement

log = logging.getLogger(__name__)

try:
    if config.max31855 + config.max6675 + config.max31855spi > 1:
        log.error("choose (only) one converter IC")
        exit()
    if config.max31855:
        from max31855 import MAX31855, MAX31855Error
        log.info("import MAX31855")
    if config.max31855spi:
        import Adafruit_GPIO.SPI as SPI
        from max31855spi import MAX31855SPI, MAX31855SPIError
        log.info("import MAX31855SPI")
        spi_reserved_gpio = [7, 8, 9, 10, 11]
        if config.gpio_air in spi_reserved_gpio:
            raise Exception("gpio_air pin %s collides with SPI pins %s" % (config.gpio_air, spi_reserved_gpio))
        if config.gpio_cool in spi_reserved_gpio:
            raise Exception("gpio_cool pin %s collides with SPI pins %s" % (config.gpio_cool, spi_reserved_gpio))
        if config.gpio_door in spi_reserved_gpio:
            raise Exception("gpio_door pin %s collides with SPI pins %s" % (config.gpio_door, spi_reserved_gpio))
        if config.gpio_heat in spi_reserved_gpio:
            raise Exception("gpio_heat pin %s collides with SPI pins %s" % (config.gpio_heat, spi_reserved_gpio))
    if config.max6675:
        from max6675 import MAX6675, MAX6675Error
        log.info("import MAX6675")
    sensor_available = True
except ImportError:
    log.exception("Could not initialize temperature sensor, using dummy values!")
    sensor_available = False

try:
    import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(config.gpio_heat, GPIO.OUT)
    GPIO.setup(config.gpio_cool, GPIO.OUT)
    GPIO.setup(config.gpio_air, GPIO.OUT)
    GPIO.setup(config.gpio_door, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    gpio_available = True
except ImportError:
    msg = "Could not initialize GPIOs, oven operation will only be simulated!"
    log.warning(msg)
    gpio_available = False


class Oven (threading.Thread):
    STATE_IDLE = "IDLE"
    STATE_RUNNING = "RUNNING"
    STATE_TUNING = "TUNING"

    def __init__(self, simulate=False, time_step=config.sensor_time_wait):
        threading.Thread.__init__(self) ##I don't have a strong idea of how threading works in python. Leave intact
        self.daemon = True
        self.simulate = simulate
        self.time_step = time_step
        self.reset()
        if simulate:  ##I don't see a strong use case for simulation, but leave it in for if this gets pulled into main code
            self.temp_sensor = TempSensorSimulate(self, 0.5, self.time_step)
        if sensor_available:
            self.temp_sensor = TempSensorReal(self.time_step)
        else:
            self.temp_sensor = TempSensorSimulate(self,
                                                  self.time_step,
                                                  self.time_step)
        self.temp_sensor.start()
        self.start()

    def reset(self):
        self.profile = None
        self.start_time = 0
        self.runtime = 0
        self.totaltime = 0
        self.target = 0
        self.heating = False
        self.heatpwm = 0;
        self.door = self.get_door_state()  ##Should be a way to disable the door sensors
        self.state = Oven.STATE_IDLE
        self.set_heat(0)
        self.set_cool(False) ##...and cooling
        self.set_air(False) ##...and... air? Not sure what this is for
        self.pid = PID(ki=config.pid_ki, kd=config.pid_kd, kp=config.pid_kp) ##PID values need to be readable or writable
                                                                                  ##store in config? Or elsewhere

    def run_profile(self, profile):
        ##Looks good
        log.info("Running profile %s" % profile.name)
        self.profile = profile
        self.totaltime = profile.get_duration()
        self.state = Oven.STATE_RUNNING
        self.start_time = datetime.datetime.now()
        log.info("Starting")

    def abort_run(self):
        self.reset()
        
        ##Right here is where I'd add a PID tuning algorithm. Create function to start tuning, and add a if.state=tuning to run algo
        ##Also add another Oven.state for "tuning"
    def run_tuning(self, temp_target, n_cycles):
        log.info("Running auto-tune algorithm. Target: %.0f deg C, Cycles: %", temp_target, n_cycles);
        self.state = Oven.STATE_TUNING
        self.start_time = datetime.datetime.now()
        self.heating = 0
        self.heatpwm = 0.5       #Marlin starts at 50%. What happens if this isn't enough to hit the target?
        self.bias = self.heatpwm    #Thinking these shouldn't be class variables? Need to brush up on python and namespace
        self.d = self.heatpwm
        self.target = temp_target
        self.maxtemp = -10000
        self.mintemp = 10000
        self.t1 = datetime.datetime.now()   #t1 is when the temp goes over target
        self.t2 = self.t1                   #t2 is when it goes under
        log.info("Starting")

    def run(self):
        temperature_count = 0
        last_temp = 0
        pid = 0
        now = datetime.datetime.now()
        while True:
            self.door = self.get_door_state() ##What is this variable for?
            
            if self.state == Oven.STATE_TUNING:
                #This algorithm is based off that used by Marlin (3-D printer control).
                #It essentially measures the overshoot and undershoot when turning the heat on and off,
                #Then applies some guideline formulas to come up with the K values
                temp = self.temp_sensor.temperature
                self.maxtemp = max(self.maxtemp, temp)
                self.mintemp = min(self.mintemp, temp)
                if (self.heating and temp > self.target):
                    #These events occur once we swing over the temperature target
                    if((now - self.t2).total_seconds > 5):  #debounce: prevent noise from triggering false transition
                                                            #This might need to be longer for large systems
                            self.heating = false
                            self.heatpwm = (self.bias - self.d)/2
                            self.t1 = now
                            self.t_high = (self.t1-self.t2).total_seconds
                            self.maxtemp = temp
                            
                if (self.heating == false and temp < self.target):
                    #This occurs when we swing below the target
                    if((now - self.t1).total_seconds > 5): #same debounce
                        self.heating = true
                        t2 = now
                        self.t_low = (self.t2-self.t1).total_seconds
                        if (self.cycles > 0):
                            self.bias = self.bias + (self.d * (self.t_high - self.t_low))/(self.t_low + self.t_high)
                            self.bias = sorted([20, self.bias, 80])[1]
                            self.d = self.bias if self.bias < 50 else 99 - self.bias       ##not sure what all this does
                            log.info("bias: %, d: %, min: %, max: %", self.bias, self.d, self.min, self.max)
                            if (self.cycles > 2):
                                #Magic formulas:
                                Ku = (4.0 * self.d)/ (math.Pi * (self.max - self.min) / 2)
                                Tu = (t_low + t_high)
                                log.info("Ku: %, Tu: %", Ku, Tu)
                                Kp = 0.6 * Ku
                                Ki = 2*Kp/Tu
                                Kd = Kp * Tu/8
                                log.info("Kp: %, Ki: %, Kd = %", Kp, Ki, Kd)
                                
                            self.heatpwm = (self.bias + self.d)/2
                            self.cycles++
                            self.min = self.target
                            
                #TODO: End conditions
                                
                            
                            
                
            
            ## I might rewrite a lot of this
            if self.state == Oven.STATE_RUNNING:
                if self.simulate: ##Probably just won't touch the simulation
                    self.runtime += 0.5
                else:
                    runtime_delta = datetime.datetime.now() - self.start_time
                    self.runtime = runtime_delta.total_seconds()
                log.info("running at %.1f deg C (Target: %.1f) , heat %.2f, cool %.2f, air %.2f, door %s (%.1fs/%.0f)" % 
                         (self.temp_sensor.temperature, self.target, self.heat, self.cool, self.air, self.door, self.runtime, 
                          self.totaltime))  ##Consider being able to display deg F
                self.target = self.profile.get_target_temperature(self.runtime)
                pid = self.pid.compute(self.target, self.temp_sensor.temperature)
                
                ##Should we store the current run time in case of power failure?

                log.info("pid: %.3f" % pid)

                self.set_cool(pid <= -1) ##cooling might not be enabled
                if(pid > 0):
                    ##keep this... but double check that it won't cause false failure in large systems
                    # The temp should be changing with the heat on
                    # Count the number of time_steps encountered with no change and the heat on
                    if last_temp == self.temp_sensor.temperature:
                        temperature_count += 1 ##rename this variable
                    else:
                        temperature_count = 0
                    # If the heat is on and nothing is changing, reset
                    # The direction or amount of change does not matter
                    # This prevents runaway in the event of a sensor read failure                   
                    if temperature_count > 20:
                        log.info("Error reading sensor, oven temp not responding to heat.")
                        self.reset()
                else:
                    temperature_count = 0
                    
                #Capture the last temperature value.  This must be done before set_heat, since there is a sleep in there now.
                last_temp = self.temp_sensor.temperature
                
                self.set_heat(pid)
                
                #if self.profile.is_rising(self.runtime):
                #    self.set_cool(False)
                #    self.set_heat(self.temp_sensor.temperature < self.target)
                #else:
                #    self.set_heat(False)
                #    self.set_cool(self.temp_sensor.temperature > self.target)

                ##These should be configurable... assuming "air" is used. Still not sure what it does
                if self.temp_sensor.temperature > 200:
                    self.set_air(False)
                elif self.temp_sensor.temperature < 180:
                    self.set_air(True)

                if self.runtime >= self.totaltime:
                    self.reset()

            ##Don't like that this sleep is here
            if pid > 0:
                time.sleep(self.time_step * (1 - pid))
            else:
                time.sleep(self.time_step)

    def set_heat(self, value):
        ##Not a fan of bit-banging in the same thread as the control algorithm
        ##for one thing, it forces the pwm period to be the same as the control cycle time
        ##Move this to a different thread or use hardware PWM
        if value > 0:
            self.heat = 1.0
            if gpio_available:
               if config.heater_invert:
                 GPIO.output(config.gpio_heat, GPIO.LOW)
                 time.sleep(self.time_step * value)
                 GPIO.output(config.gpio_heat, GPIO.HIGH)   
               else:
                 GPIO.output(config.gpio_heat, GPIO.HIGH)
                 time.sleep(self.time_step * value)
                 GPIO.output(config.gpio_heat, GPIO.LOW)   
        else:
            self.heat = 0.0
            if gpio_available:
               if config.heater_invert:
                 GPIO.output(config.gpio_heat, GPIO.HIGH)
               else:
                 GPIO.output(config.gpio_heat, GPIO.LOW)

    def set_cool(self, value):
        ##This should also be in the other thread. Add option to not have cooling
        if value:
            self.cool = 1.0
            if gpio_available:
                GPIO.output(config.gpio_cool, GPIO.LOW)
        else:
            self.cool = 0.0
            if gpio_available:
                GPIO.output(config.gpio_cool, GPIO.HIGH)

    def set_air(self, value):
        ##Ditto
        if value:
            self.air = 1.0
            if gpio_available:
                GPIO.output(config.gpio_air, GPIO.LOW)
        else:
            self.air = 0.0
            if gpio_available:
                GPIO.output(config.gpio_air, GPIO.HIGH)

    def get_state(self):
        state = {
            'runtime': self.runtime,
            'temperature': self.temp_sensor.temperature,
            'target': self.target,
            'state': self.state,
            'heat': self.heat,
            'cool': self.cool,
            'air': self.air,
            'totaltime': self.totaltime,
            'door': self.door
        }
        return state

    def get_door_state(self):
        ##have option for not having a door switch
        if gpio_available:
            return "OPEN" if GPIO.input(config.gpio_door) else "CLOSED"
        else:
            return "UNKNOWN"

##Not sure how this class is used
class TempSensor(threading.Thread):
    def __init__(self, time_step):
        threading.Thread.__init__(self)
        self.daemon = True
        self.temperature = 0
        self.time_step = time_step


class TempSensorReal(TempSensor):
    def __init__(self, time_step):
        TempSensor.__init__(self, time_step)
        if config.max6675:
            log.info("init MAX6675")
            self.thermocouple = MAX6675(config.gpio_sensor_cs,
                                     config.gpio_sensor_clock,
                                     config.gpio_sensor_data,
                                     config.temp_scale)

        if config.max31855:
            log.info("init MAX31855")
            self.thermocouple = MAX31855(config.gpio_sensor_cs,
                                     config.gpio_sensor_clock,
                                     config.gpio_sensor_data,
                                     config.temp_scale)

        if config.max31855spi:
            log.info("init MAX31855-spi")
            self.thermocouple = MAX31855SPI(spi_dev=SPI.SpiDev(port=0, device=config.spi_sensor_chip_id))

    def run(self):
        while True:
            try:
                self.temperature = self.thermocouple.get()
            except Exception:
                log.exception("problem reading temp")
            time.sleep(self.time_step)

##Cool math. I'll leave it alone
class TempSensorSimulate(TempSensor):
    def __init__(self, oven, time_step, sleep_time):
        TempSensor.__init__(self, time_step)
        self.oven = oven
        self.sleep_time = sleep_time

    def run(self):
        t_env      = config.sim_t_env
        c_heat     = config.sim_c_heat
        c_oven     = config.sim_c_oven
        p_heat     = config.sim_p_heat
        R_o_nocool = config.sim_R_o_nocool
        R_o_cool   = config.sim_R_o_cool
        R_ho_noair = config.sim_R_ho_noair
        R_ho_air   = config.sim_R_ho_air

        t = t_env  # deg C  temp in oven
        t_h = t    # deg C temp of heat element
        while True:
            #heating energy
            Q_h = p_heat * self.time_step * self.oven.heat

            #temperature change of heat element by heating
            t_h += Q_h / c_heat

            if self.oven.air:
                R_ho = R_ho_air
            else:
                R_ho = R_ho_noair

            #energy flux heat_el -> oven
            p_ho = (t_h - t) / R_ho

            #temperature change of oven and heat el
            t   += p_ho * self.time_step / c_oven
            t_h -= p_ho * self.time_step / c_heat

            #energy flux oven -> env
            if self.oven.cool:
                p_env = (t - t_env) / R_o_cool
            else:
                p_env = (t - t_env) / R_o_nocool

            #temperature change of oven by cooling to env
            t -= p_env * self.time_step / c_oven
            log.debug("energy sim: -> %dW heater: %.0f -> %dW oven: %.0f -> %dW env" % (int(p_heat * self.oven.heat), t_h, int(p_ho), t, int(p_env)))
            self.temperature = t

            time.sleep(self.sleep_time)

##I'd like to add units to profiles. Option for time (seconds, minutes, hours) and temperature (Celcius, Farenheit, Kelvin)
##The code will still run in celcius/seconds, so the profile has to get converted to that
class Profile():
    def __init__(self, json_data):
        obj = json.loads(json_data)
        self.name = obj["name"]
        self.data = sorted(obj["data"])
        ##TODO: Read unit data

    def get_duration(self):
        ##TODO: convert to seconds
        return max([t for (t, x) in self.data])


    def get_surrounding_points(self, time):
        ##todo: convert to seconds
        if time > self.get_duration():
            return (None, None)

        prev_point = None
        next_point = None
        
        ##TODO: convert to seconds/celcius
        for i in range(len(self.data)):
            if time < self.data[i][0]:
                prev_point = self.data[i-1]
                next_point = self.data[i]
                break

        return (prev_point, next_point)

    def is_rising(self, time):
        (prev_point, next_point) = self.get_surrounding_points(time)
        if prev_point and next_point:
            return prev_point[1] < next_point[1]
        else:
            return False

    def get_target_temperature(self, time):
        if time > self.get_duration():
            return 0

        (prev_point, next_point) = self.get_surrounding_points(time)

        incl = float(next_point[1] - prev_point[1]) / float(next_point[0] - prev_point[0])
        temp = prev_point[1] + (time - prev_point[0]) * incl
        return temp


class PID():
    def __init__(self, ki=1, kp=1, kd=1):
        self.ki = ki
        self.kp = kp
        self.kd = kd
        self.lastNow = datetime.datetime.now()
        self.iterm = 0
        self.lastErr = 0

    def compute(self, setpoint, ispoint):
        now = datetime.datetime.now()
        timeDelta = (now - self.lastNow).total_seconds()

        error = float(setpoint - ispoint)
        self.iterm += (error * timeDelta * self.ki)
        self.iterm = sorted([-1, self.iterm, 1])[1]
        dErr = (error - self.lastErr) / timeDelta

        output = self.kp * error + self.iterm + self.kd * dErr
        output = sorted([-1, output, 1])[1]
        self.lastErr = error
        self.lastNow = now

        return output

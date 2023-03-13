WORK IN PROGRESS!

OrangeThermal is a fork off of picoThermal by [Petronator], which is a fork off of picoReflow by [apollo-ng]. It is intended to make it work on Orange Pi Zero 2 for PCB reflow ovens:

The following was added by Petronator to the fork

* Ability to disable cooling, air, door sensor, etc.
* Handling of two separate heaters
* Automatic PID tuning
* Time and temperature units defined per profile
* Temperature PWM loop separated from control loop
* Option to wait for system to reach target temperature before going to next profile stage
* For ceramic kilns, dynamic calculation of end temperature based on heat rate

The following was added for this branch of the fork

* Works with Orange Pi Zero 2
* Works with Phyton 3
* Works with GPIO Board instaed of BCM
* Deleted MAX6675 to concentrate on MAX31855 and MAX31856 (near future)

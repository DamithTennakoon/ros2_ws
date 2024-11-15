# Import RPi libraries
import RPi.GPIO as GPIO # For PWM
import time
from gpiozero import AngularServo # For direct commands to servo

# Board setup
servoGpioPin = 18
GPIO.setmode(GPIO.BCM) # BCM: Broadcom SOC channel
GPIO.setup(servoGpioPin, GPIO.OUT)

pin = GPIO.PWM(servoGpioPin, 50) # Set GPIO pin 17 to operate at 50Hz 
pin.start(0) # Duty cycle for MG996R servo - units ms
try:
    while True:
        pin.ChangeDutyCycle(7.5) # 0
        time.sleep(2)
        pin.ChangeDutyCycle(10) # 90
        time.sleep(2)
        pin.ChangeDutyCycle(7.5) #0
        time.sleep(2)
        pin.ChangeDutyCycle(5) # -90
        time.sleep(2)
except KeyboardInterrupt:
    pin.stop()
    GPIO.cleanup()

'''
servo = AngularServo(18, min_pulse_width=0.0006, max_pulse_width=0.0023)

while (True):
    servo.angle = 90
    time.sleep(2)
    servo.angle = 0
    time.sleep(2)
    servo.angle = -90
    time.sleep(2)
'''

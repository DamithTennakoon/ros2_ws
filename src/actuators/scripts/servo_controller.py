# Import RPi libraries
import RPi.GPIO as GPIO # For PWM
import time
from gpiozero import AngularServo # For direct commands to servo

'''
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

# TEST
# Pin configuration
servo_pin = 18

# Setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(servo_pin, GPIO.OUT)

# Start PWM at 50Hz
pwm = GPIO.PWM(servo_pin, 50)
pwm.start(0)  # Initial duty cycle is 0

def set_angle(angle):
    # Calculate duty cycle
    duty_cycle = max(2.5, min(angle / 18 + 2.5, 12.5))  # Clamp to 2.5-12.5
    pwm.ChangeDutyCycle(duty_cycle)
    time.sleep(0.5)  # Give the servo time to move
    pwm.ChangeDutyCycle(0)  # Stop sending signal after movement

try:
    while True:
        # Move servo to -90°
        print("Moving to -90°")
        set_angle(-90)
        time.sleep(1)

        # Move servo to 0°
        print("Moving to 0°")
        set_angle(0)
        time.sleep(1)

        # Move servo to +90°
        print("Moving to +90°")
        set_angle(90)
        time.sleep(1)

except KeyboardInterrupt:
    print("Exiting program.")

finally:
    pwm.stop()
    GPIO.cleanup()

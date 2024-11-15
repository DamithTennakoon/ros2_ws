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
# Set up GPIO mode
GPIO.setmode(GPIO.BCM)

# Define the GPIO pin for the servo
servo_pin = 18

# Set the GPIO pin as an output
GPIO.setup(servo_pin, GPIO.OUT)

# Set up PWM on the servo pin with a frequency of 50Hz
pwm = GPIO.PWM(servo_pin, 50)  # MG996R typically uses a 50Hz signal
pwm.start(0)  # Start PWM with 0% duty cycle (servo at rest)

# Define a function to set servo angles
def set_servo_angle(angle):
    # The MG996R expects pulses between ~1ms (-90°) and ~2ms (+90°)
    # At 50Hz, 1ms corresponds to 5% duty cycle, and 2ms corresponds to 10% duty cycle
    duty_cycle = (angle / 18.0) + 2.5  # Convert angle to duty cycle
    pwm.ChangeDutyCycle(duty_cycle)
    time.sleep(0.5)  # Allow time for the servo to reach the position
    pwm.ChangeDutyCycle(0)  # Turn off signal to prevent jitter

try:
    while True:
        print("Moving to -90°")
        set_servo_angle(-90)  # Move to -90 degrees
        time.sleep(2)

        print("Moving to 0°")
        set_servo_angle(0)  # Move to 0 degrees
        time.sleep(2)

        print("Moving to +90°")
        set_servo_angle(90)  # Move to +90 degrees
        time.sleep(2)

except KeyboardInterrupt:
    print("Exiting program...")

finally:
    pwm.stop()
    GPIO.cleanup()
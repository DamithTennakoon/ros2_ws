# Import libraries
from pymycobot.mycobot import MyCobot
from pymycobot.genre import Angle
from pymycobot import PI_PORT, PI_BAUD
import time

# Create a MyCobot object
mc = MyCobot("/dev/ttyACM0", 115200)
mc.init_gripper()
mc.set_gripper_mode(0)

mc.set_gripper_state(0, 80)
time.sleep(3)
mc.set_gripper_state(1,80)
time.sleep(3)

mc.set_gripper_value(0, 50) # Move the gripper to full-close position
time.sleep(3)
mc.set_gripper_value(100, 50) # Move the gripper to full-open position
time.sleep(3)
mc.set_gripper_value(50, 50) # Move the gripper to functional position
time.sleep(3)
mc.send_coords([93, -120, 280, 180, 7, 95], 10, 1) # Move to  position 1 
print("completed")

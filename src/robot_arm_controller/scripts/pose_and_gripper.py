# Objective: Utilize both the send_coords() functionality and the gripper functionality to determine signal errors.
# Written by: Damith Tennakoon

# Notes:
# - Determine two testing poses for the arm (look at trajectory arrays from experiements)

# Import myCobot libraries
from pymycobot.mycobot import MyCobot
from pymycobot.genre import Angle
from pymycobot import PI_PORT, PI_BAUD

# Import signal libraries
import time

# Main method - execute robot arm commands
def main():
    # Initialize the robot arm connection
    mc = MyCobot("/dev/ttyACM0", 115200) # Instance of the MyCobot class
    time.sleep(1) 
    mc.set_gripper_mode(0) # Setting gripper to 485 mode 
    time.sleep(1)
    
    # Define state parameters
    move_delay = 10
    gripper_delay = 4

    # Initialize the robot arms movements
    mc.set_color(255, 0, 0) 
    time.sleep(0.5)
    mc.send_coords([93, -120, 280, 180, 7, 95], 10, 1) # Move to  position 1 
    time.sleep(move_delay)
    mc.set_grippper_vale(100, 50) # Full-open grippper 
    time.sleep(gripper_delay)
    mc.send_coords([275, 250, 209, 180, 7, 95], 10, 1) # Move to position 2
    time.sleep(move_delay)
    mc.set_grippper_vale(30, 50) # Near-close grippper 
    time.sleep(gripper_delay)
    mc.send_coords([93, -120, 280, 180, 7, 95], 10, 1) # Move to  position 1 
    time.sleep(move_delay)
    mc.set_grippper_vale(100, 50) # Full-open grippper 
    time.sleep(gripper_delay)


# Run main method
if __name__ == '__main__':
    main()
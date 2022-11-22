import sys, os

from urscript_interfaces.srv import UrScript
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

PACKAGE_NAME = "urscript_picker"
URSCRIPT_SERVICE = "urscript_service"

# Constant used to indicate whether to perform gripper open and close in demo mode
DEMO = False 


##################################################################
# POSITIONS CONSTANTS
##################################################################

# Home position - joints and positions
POS_HOME_JOINTS = [-1.57, -1.57, -1.57, -1.57, 1.57, 0]
POS_HOME_XYZ = [-0.171, -0.682, 0.428, 0, 3.148, 0]

# Boxes initial positions, as indexed in the task description
# BOXES_R = [2.224, -2.224, 0]
BOXES_R = [0.0, 3.148, 0.0]
POS_BOX_1 = [0.491, -0.134, -0.060] + BOXES_R
POS_BOX_2 = [0.487, 0.042, -0.060] + BOXES_R
POS_BOX_3 = [0.491, 0.240, -0.060] + BOXES_R
POS_BOX_4 = [0.690, -0.143, -0.060] + BOXES_R
POS_BOX_5 = [0.701, 0.039, -0.060] + BOXES_R
POS_BOX_6 = [0.690, 0.230, -0.060] + BOXES_R
POS_BOX_7 = [0.888, -0.144, -0.060] + BOXES_R
POS_BOX_8 = [0.900, 0.044, -0.060] + BOXES_R
POS_BOX_9 = [0.887, 0.226, -0.060] + BOXES_R
POS_BOX_10 = [0.491, -0.134, 0.050] + BOXES_R
POS_BOX_11 = [0.491, 0.240, 0.050] + BOXES_R
POS_BOX_12 = [0.701, 0.039, 0.050] + BOXES_R
POS_BOX_13 = [0.888, -0.144, 0.050] + BOXES_R
POS_BOX_14 = [0.887, 0.226, 0.050] + BOXES_R

# New positions of the boxes, in the order of placement
NEW_POS_OFFSET_Y = 0.080 
NEW_POS_INITIAL_Y = -0.800
BOX_WIDTH = 0.11

POS_NEW_1 = [0.0, NEW_POS_INITIAL_Y, -0.060] + BOXES_R
POS_NEW_2 = [0.0, NEW_POS_INITIAL_Y+BOX_WIDTH+NEW_POS_OFFSET_Y, -0.060] + BOXES_R
POS_NEW_3 = [0.0, NEW_POS_INITIAL_Y+BOX_WIDTH+NEW_POS_OFFSET_Y, 0.050] + BOXES_R
POS_NEW_4 = [0.0, NEW_POS_INITIAL_Y+BOX_WIDTH+2*NEW_POS_OFFSET_Y, -0.060] + BOXES_R
POS_NEW_5 = [0.0, NEW_POS_INITIAL_Y+BOX_WIDTH+2*NEW_POS_OFFSET_Y, 0.050] + BOXES_R
POS_NEW_6 = [0.0, NEW_POS_INITIAL_Y+BOX_WIDTH+2*NEW_POS_OFFSET_Y, 0.160] + BOXES_R
POS_NEW_7 = [0.0, NEW_POS_INITIAL_Y+BOX_WIDTH+3*NEW_POS_OFFSET_Y, -0.060] + BOXES_R
POS_NEW_8 = [0.0, NEW_POS_INITIAL_Y+BOX_WIDTH+3*NEW_POS_OFFSET_Y, 0.050] + BOXES_R
POS_NEW_9 = [0.0, NEW_POS_INITIAL_Y+BOX_WIDTH+3*NEW_POS_OFFSET_Y, 0.160] + BOXES_R
POS_NEW_10 = [0.0, NEW_POS_INITIAL_Y+BOX_WIDTH+3*NEW_POS_OFFSET_Y, 0.270] + BOXES_R


POS_ADD_1 = [0.0, -0.440, 0.380] + BOXES_R
POS_ADD_2 = [0.0, -0.440, 0.490] + BOXES_R
POS_ADD_3 = [0.0, -0.440, 0.600] + BOXES_R
POS_ADD_4 = [0.0, -0.440, 0.710] + BOXES_R

# Distance from which to approach the boxes before picking
ABOVE_BOX_Z_DIST = 0.22
ABOVE_BOX_Z_DIST_CLOSE = 0.02

INITIAL_POSITIONS = [POS_BOX_10, # 1
                     POS_BOX_12, # 2
                     POS_BOX_11, # 3
                     POS_BOX_1,  # 4
                     POS_BOX_2,  # 5
                     POS_BOX_4,  # 6
                     POS_BOX_5,  # 7
                     POS_BOX_6,  # 8
                     POS_BOX_3,  # 9
                     POS_BOX_13]  # 10

NEW_POSITIONS = [POS_NEW_1,
                 POS_NEW_2,
                 POS_NEW_3,
                 POS_NEW_4,
                 POS_NEW_5,
                 POS_NEW_6,
                 POS_NEW_7,
                 POS_NEW_8,
                 POS_NEW_9,
                 POS_NEW_10]

# Bonus positions
BONUS_POSITIONS_INITIAL = [POS_BOX_7, POS_BOX_8, POS_BOX_14, POS_BOX_9]
BONUS_POSITIONS_TARGET = [POS_ADD_1, POS_ADD_2, POS_ADD_3, POS_ADD_4]


##################################################################
# URSCRIPT CONSTANTS
##################################################################
# Paths to script files
SCRIPTS_DIR = os.path.join(get_package_share_directory(PACKAGE_NAME), "scripts")
SCRIPT_GRIPPER_OPEN = "gripper_open.script"
SCRIPT_GRIPPER_CLOSE = "gripper_close.script"



class URScriptClientAsync(Node):
    def __init__(self):
        super().__init__('urscript_client_async')
        self.cli = self.create_client(UrScript, URSCRIPT_SERVICE)
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = UrScript.Request()
        print("request", self.req)

    def send_request(self, command):
        self.req.data = command
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def send_urscript_command(urscript_client, urscript_command):
    urscript_client.get_logger().info("=>Executing " + urscript_command)
    response = urscript_client.send_request(urscript_command)
    urscript_client.get_logger().info('--> Result for URScript command for %s: %d : %s' %
        (urscript_command, int(response.success), response.error_reason))


def movej(urscript_client, joint_position, velocity="0.3"):
    position_string = ", ".join(map(str, joint_position))

    command =  "def move():\n"
    command += "\tmovej(["+position_string+"],a=1.0,v="+velocity+")\n"
    command += "end\n"
    send_urscript_command(urscript_client, command)


def movel(urscript_client, position, velocity="0.3", radius="0.0"):
    position_string = ", ".join(map(str, position))

    command =  "def move():\n"
    command += "\tmovel(p["+ position_string +"],a=1.0,v="+velocity+",t=0,r="+radius+")\n"
    command += "end\n"
    send_urscript_command(urscript_client, command)    


def read_script_from_file(script_file):
    print('Reading script from file', script_file)
    f = open(SCRIPTS_DIR + "/" + script_file, "r")
    script = f.read()
    return script 


def open_gripper(urscript_client):
    print('Opening gripper')
    command = read_script_from_file(SCRIPT_GRIPPER_OPEN)
    print(DEMO)
    if not DEMO:
        send_urscript_command(urscript_client, command)    


def close_gripper(urscript_client):
    print('Closing gripper')
    command = read_script_from_file(SCRIPT_GRIPPER_CLOSE)
    print(DEMO)
    if not DEMO:
        send_urscript_command(urscript_client, command)    


def go_home_j(urscript_client):
    movej(urscript_client, POS_HOME_JOINTS)
    

def go_home(urscript_client):
    movel(urscript_client, POS_HOME_XYZ)


def move_box(urscript_client, pos_from, pos_to):
    # Move above the initial position of the box
    pos_from_above = pos_from.copy()
    pos_from_above[2] += ABOVE_BOX_Z_DIST
    movel(urscript_client, pos_from_above)

    # Open gripper
    open_gripper(urscript_client)

    # Go down closer to box
    movel(urscript_client, pos_from)

    # Close gripper to get box
    close_gripper(urscript_client)

    # Raise the box
    movel(urscript_client, pos_from_above)

    # Move to new position - above
    pos_to_above = pos_to.copy()
    pos_to_above[2] += ABOVE_BOX_Z_DIST
    movel(urscript_client, pos_to_above)

    # Move down
    pos_to_close = pos_to.copy()
    pos_to_close[2] += ABOVE_BOX_Z_DIST_CLOSE
    movel(urscript_client, pos_to_close)

    # Open gripper to place box
    open_gripper(urscript_client)

    # Go up
    movel(urscript_client, pos_to_above)

    # Go to the home position
    # go_home(urscript_client)


def pick_and_place(urscript_client, initial_positions, target_positions):
    for i, (pos_from, pos_to) in enumerate(zip(initial_positions, target_positions)):
        print(f"moving box {i+1} from [{pos_from}] to [{pos_to}]")
        move_box(urscript_client, pos_from, pos_to)    


def main():
    print('IS DEMO?', DEMO)
    rclpy.init()

    urscript_client = URScriptClientAsync()
    
    # Move the robot to the home position before starting the process
    go_home_j(urscript_client)

    # Perform the whole pick-and-place process
    pick_and_place(urscript_client, INITIAL_POSITIONS, NEW_POSITIONS)

    # If the above succeeds, place the last 4 boxes
    pick_and_place(urscript_client, BONUS_POSITIONS_INITIAL, BONUS_POSITIONS_TARGET)
    
    urscript_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

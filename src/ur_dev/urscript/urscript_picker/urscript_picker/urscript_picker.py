import sys

from urscript_interfaces.srv import UrScript
import rclpy
from rclpy.node import Node

# Home position - joints and positions
POS_HOME_JOINTS = [-90, -90, -90, -90, 90, 0]
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
POS_NEW_1 = [0.0, -0.800, -0.060] + BOXES_R
POS_NEW_2 = [0.0, -0.680, -0.060] + BOXES_R
POS_NEW_3 = [0.0, -0.680, 0.050] + BOXES_R
POS_NEW_4 = [0.0, -0.560, -0.060] + BOXES_R
POS_NEW_5 = [0.0, -0.560, 0.050] + BOXES_R
POS_NEW_6 = [0.0, -0.560, 0.160] + BOXES_R
POS_NEW_7 = [0.0, -0.440, -0.060] + BOXES_R
POS_NEW_8 = [0.0, -0.440, 0.050] + BOXES_R
POS_NEW_9 = [0.0, -0.440, 0.160] + BOXES_R
POS_NEW_10 = [0.0, -0.440, 0.270] + BOXES_R

ABOVE_BOX_Z_DIST = 0.20
ABOVE_BOX_Z_DIST_CLOSE = 0.02

INITIAL_POSITIONS = [POS_BOX_10, #POS_BOX_10, # 1
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

URSCRIPT_SERVICE = "urscript_service"


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


def open_gripper(urscript_client):
    print('Opening gripper')


def close_gripper(urscript_client):
    print('Closing gripper')


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


def pick_and_place(urscript_client):
    for i, (pos_from, pos_to) in enumerate(zip(INITIAL_POSITIONS, NEW_POSITIONS)):
        print(f"moving box {i+1} from [{pos_from}] to [{pos_to}]")
        move_box(urscript_client, pos_from, pos_to)    


def main():
    rclpy.init()

    urscript_client = URScriptClientAsync()
    
    # Move the robot to the home position before starting the process
    # go_home(urscript_client)

    # Perform the whole pick-and-place process
    pick_and_place(urscript_client)    
    
    urscript_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
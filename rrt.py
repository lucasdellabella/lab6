# Saqib Ali Lucas Della Bella
import cozmo
import math
import sys
import time
import random
import copy
import asyncio
import numpy as np

from cmap import *
from gui import *
from utils import *

from cozmo.util import radians

MAX_NODES = 20000
MM_PER_INCH = 25.4
LOCAL_ORIGIN = Node((6 * MM_PER_INCH, 10 * MM_PER_INCH))

def step_ninety_pct(node0, node1):
    ########################################################################
    # TODO: please enter your code below.
    # 1. If distance between two nodes is less than limit, return node1
    # 2. Otherwise, return a node in the direction from node0 to node1 whose
    #    distance to node0 is limit. Recall that each iteration we can move
    #    limit units at most
    # 3. Hint: please consider using np.arctan2 function to get vector angle
    # 4. Note: remember always return a Node object
    y = (node1.y - node0.y) * 0.9
    x = (node1.x - node0.x) * 0.9
    return Node((node0.x + x, node0.y + y))

    ############################################################################

def step_from_to(node0, node1, limit=75):
    ########################################################################
    # TODO: please enter your code below.
    # 1. If distance between two nodes is less than limit, return node1
    # 2. Otherwise, return a node in the direction from node0 to node1 whose
    #    distance to node0 is limit. Recall that each iteration we can move
    #    limit units at most
    # 3. Hint: please consider using np.arctan2 function to get vector angle
    # 4. Note: remember always return a Node object
    hypotenuse = get_dist(node0, node1)
    if  hypotenuse < limit:
        return node1
    else:
        angle = math.atan2(node1.y - node0.y, node1.x - node0.x)
        y = math.sin(angle) * limit
        x = math.cos(angle) * limit
        assert limit - 1 <= get_dist(node0, Node((node0.x + x, node0.y + y))) <= limit + 1
        return Node((node0.x + x, node0.y + y))

    ############################################################################


def node_generator(cmap):
    rand_node = None
    ############################################################################
    # TODO: please enter your code below.
    # 1. Use CozMap width and height to get a uniformly distributed random node
    # 2. Use CozMap.is_inbound and CozMap.is_inside_obstacles to determine the
    #    legitimacy of the random node.
    # 3. Note: remember always return a Node object
    ############################################################################
    while (not rand_node):
        position = (random.uniform(0, cmap.width), random.uniform(0, cmap.height))
        new_node = Node(position)

        # 5% of time use goal as generated node
        if random.random() < 0.05 and cmap.get_goals():
            goal = random.sample(cmap.get_goals(), 1)[0]
            new_node = copy.deepcopy(goal)

        if cmap.is_inbound(new_node) and not cmap.is_inside_obstacles(new_node):
            rand_node = new_node
    return rand_node


def RRT(cmap, start):
    cmap.add_node(start)
    map_width, map_height = cmap.get_size()
    while (cmap.get_num_nodes() < MAX_NODES):
        ########################################################################
        # TODO: please enter your code below.
        # 1. Use CozMap.get_random_valid_node() to get a random node. This
        #    function will internally call the node_generator above
        # 2. Get the nearest node to the random node from RRT
        # 3. Limit the distance RRT can move
        # 4. Add one path from nearest node to random node
        #
        rand_node = cmap.get_random_valid_node()

        nearest_node = Node((float('inf'),  float('inf')))
        for cur_node in cmap.get_nodes():
            if get_dist(cur_node, rand_node) < get_dist(nearest_node, rand_node):
                nearest_node = cur_node
        rand_node = step_from_to(nearest_node, rand_node)
        if not cmap.is_inbound(rand_node) \
            or cmap.is_inside_obstacles(rand_node) \
            or cmap.is_collision_with_obstacles((nearest_node, rand_node)):
            continue

        cmap.add_node(rand_node)
        ########################################################################
        time.sleep(0.01)
        cmap.add_path(nearest_node, rand_node)
        if cmap.is_solved():
            print('cmap is solved')
            break

    path = cmap.get_path()
    smoothed_path = cmap.get_smooth_path()
    print ('Found smooth path')
    if cmap.is_solution_valid():
        print("A valid solution has been found :-) ")
        print("Nodes created: ", cmap.get_num_nodes())
        print("Path length: ", len(path))
        print("Smoothed path length: ", len(smoothed_path))
    else:
        print("Please try again :-(")

## MAY NEED TO PUT INTO GLOBAL COORDS
def robot_as_node(robot):
    return Node((robot.pose.position.x, robot.pose.position.y))

def global_robot_as_node(robot):
    return get_global_node(robot.pose_angle.radians, LOCAL_ORIGIN, robot_as_node(robot))

def angle_from_xaxis(node_a, node_b):
    return math.atan2(node_b.y - node_a.y, node_b.x - node_a.x)

def diff_heading_rad(heading1, heading2):
	dh = heading1 - heading2
	while dh > math.pi:
		dh -= 2 * math.pi
	while dh <= - math.pi:
		dh += 2 * math.pi
	return dh

async def CozmoPlanning(robot: cozmo.robot.Robot):
    # Allows access to map and stopevent, which can be used to see if the GUI
    # has been closed by checking stopevent.is_set()
    global cmap, stopevent

    ########################################################################
    # TODO: please enter your code below.
    # Description of function provided in instructions
    if stopevent.is_set():
        return

    await robot.set_head_angle(cozmo.util.degrees(0)).wait_for_completed()

    observed_cubes = {}

    goal_center = None
    cmap.set_start(LOCAL_ORIGIN)
    center = Node((cmap.width / 2, cmap.height / 2))
    #TODO: Make RRT work for lab6
    #  Add center obstacle
    await add_center_obstacle(robot, LOCAL_ORIGIN)
    #  set goal in front of cube
    (update_cmap, goal_center) = await detect_cube_and_add_goal(robot, observed_cubes, LOCAL_ORIGIN)

    # RRT to cube
    # .pickup() cube
    # set goal to target_marker
    # RRT to target_marker
    # .putdown() cube
    await drive_to_goal(cmap, robot, observed_cubes)
    print('ES MUDDAFUKNGETTITTT')


async def drive_to_goal(cmap, robot, observed_cubes):
    found_goal = False
    while not found_goal:
        cmap.reset()
        RRT(cmap, cmap.get_start())
        # 3. follow RRT path to target_cube face
        smooth_path = cmap.get_smooth_path()
        for i in range(1, len(smooth_path)):
            # get current and next node references
            current_node = smooth_path[i - 1]
            next_node = smooth_path[i]
            # get displacement:
            #   x and y components of difference vector.
            dx = next_node.x - current_node.x
            dy = next_node.y - current_node.y
            #   use angle between
            angle = angle_from_xaxis(current_node, next_node)
            # rotate to see direction to travel in
            #await robot.go_to_pose(cozmo.util.Pose(robot.pose.position.x, robot.pose.position.y, 0, angle_z=radians(angle)), relative_to_robot=False).wait_for_completed()
            await robot.turn_in_place(angle=radians(diff_heading_rad(angle, robot.pose_angle.radians)), speed=cozmo.util.Angle(90)).wait_for_completed()
            #await robot.go_to_pose(cozmo.util.Pose(robot.pose.position.x + dx, robot.pose.position.y + dy, 0, angle_z=radians(angle)), relative_to_robot=False).wait_for_completed()
            speed = cozmo.util.speed_mmps(100)
            await robot.drive_straight(distance=cozmo.util.distance_mm(get_dist(current_node, next_node)), speed=speed).wait_for_completed()

            if i == len(smooth_path) - 1:
                found_goal = True



def get_global_node(local_angle, local_origin, node):
    """Helper function: Transform the node's position (x,y) from local coordinate frame specified by local_origin and local_angle to global coordinate frame.
                        This function is used in detect_cube_and_update_cmap()
        Arguments:
        local_origin -- specify local coordinate frame's origin in global coordinate frame
        local_angle -- a single angle value
        node -- a Node object

        Outputs:
        new_node -- a Node object that decribes the node's position in global coordinate frame
    """
    ########################################################################
    # TODO: please enter your code below.
    axis_aligned_x = math.cos(local_angle) * node.x + -1 * math.sin(local_angle) * node.y
    axis_aligned_y = math.sin(local_angle) * node.x + math.cos(local_angle) * node.y
    # use local angle to rotate
    new_node = Node((axis_aligned_x + local_origin.x, axis_aligned_y + local_origin.y))
    return new_node

async def add_center_obstacle(robot, cozmo_pos):
    global cmap

    # Padding of objects and the robot for C-Space
    cube_padding = 60.
    cozmo_padding = 100.

    # Flags
    update_cmap = False
    goal_center = None

    object_pos = Node((cmap.width / 2, cmap.height / 2))
    object_angle = 0

    # Define an obstacle by its four corners in clockwise order
    obstacle_nodes = []
    obstacle_nodes.append(get_global_node(object_angle, object_pos, Node((cube_padding, cube_padding))))
    obstacle_nodes.append(get_global_node(object_angle, object_pos, Node((cube_padding, -cube_padding))))
    obstacle_nodes.append(get_global_node(object_angle, object_pos, Node((-cube_padding, -cube_padding))))
    obstacle_nodes.append(get_global_node(object_angle, object_pos, Node((-cube_padding, cube_padding))))
    cmap.add_obstacle(obstacle_nodes)
    update_cmap = True

    return update_cmap, goal_center

async def detect_cube_and_add_goal(robot, marked, cozmo_pos):
    global cmap

    # Padding of objects and the robot for C-Space
    cube_padding = 60.
    cozmo_padding = 100.

    # Flags
    update_cmap = False
    goal_center = None

    # Time for the robot to detect visible cubes
    time.sleep(1)

    for obj in robot.world.visible_objects:

        if obj.object_id in marked:
            continue

        # Calculate the object pose in G_Arena
        # obj.pose is the object's pose in G_Robot
        # We need the object's pose in G_Arena (object_pos, object_angle)
        dx = obj.pose.position.x - robot.pose.position.x
        dy = obj.pose.position.y - robot.pose.position.y

        object_pos = Node((cozmo_pos.x+dx, cozmo_pos.y+dy))
        object_angle = obj.pose.rotation.angle_z.radians

        # Calculate the approach position of the object
        local_goal_pos = Node((0, -cozmo_padding))
        goal_pos = get_global_node(object_angle, object_pos, local_goal_pos)
        goal_pos = step_ninety_pct(robot_as_node(robot), goal_pos)

        # Check whether this goal location is valid
        if cmap.is_inside_obstacles(goal_pos) or (not cmap.is_inbound(goal_pos)):
            print("The goal position is not valid. Please remove the goal cube and place in another position.")
        else:
            cmap.clear_goals()
            cmap.add_goal(goal_pos)
            goal_center = object_pos

        marked[obj.object_id] = obj
        update_cmap = True

    return update_cmap, goal_center

async def detect_cube_and_update_cmap(robot, marked, cozmo_pos):
    """Helper function used to detect obstacle cubes and the goal cube.
       1. When a valid goal cube is detected, old goals in cmap will be cleared and a new goal corresponding to the approach position of the cube will be added.
       2. Approach position is used because we don't want the robot to drive to the center position of the goal cube.
       3. The center position of the goal cube will be returned as goal_center.

        Arguments:
        robot -- provides the robot's pose in G_Robot
                 robot.pose is the robot's pose in the global coordinate frame that the robot initialized (G_Robot)
                 also provides light cubes
        cozmo_pose -- provides the robot's pose in G_Arena
                 cozmo_pose is the robot's pose in the global coordinate we created (G_Arena)
        marked -- a dictionary of detected and tracked cubes (goal cube not valid will not be added to this list)

        Outputs:
        update_cmap -- when a new obstacle or a new valid goal is detected, update_cmap will set to True
        goal_center -- when a new valid goal is added, the center of the goal cube will be returned
    """
    global cmap

    # Padding of objects and the robot for C-Space
    cube_padding = 60.
    cozmo_padding = 100.

    # Flags
    update_cmap = False
    goal_center = None

    # Time for the robot to detect visible cubes
    time.sleep(1)

    for obj in robot.world.visible_objects:

        if obj.object_id in marked:
            continue

        # Calculate the object pose in G_Arena
        # obj.pose is the object's pose in G_Robot
        # We need the object's pose in G_Arena (object_pos, object_angle)
        dx = obj.pose.position.x - robot.pose.position.x
        dy = obj.pose.position.y - robot.pose.position.y

        object_pos = Node((cozmo_pos.x+dx, cozmo_pos.y+dy))
        object_angle = obj.pose.rotation.angle_z.radians

        # The goal cube is defined as robot.world.light_cubes[cozmo.objects.LightCube1Id].object_id
        if robot.world.light_cubes[cozmo.objects.LightCube1Id].object_id == obj.object_id:

            # Calculate the approach position of the object
            local_goal_pos = Node((0, -cozmo_padding))
            goal_pos = get_global_node(object_angle, object_pos, local_goal_pos)

            # Check whether this goal location is valid
            if cmap.is_inside_obstacles(goal_pos) or (not cmap.is_inbound(goal_pos)):
                print("The goal position is not valid. Please remove the goal cube and place in another position.")
            else:
                cmap.clear_goals()
                cmap.add_goal(goal_pos)
                goal_center = object_pos

        # Define an obstacle by its four corners in clockwise order
        obstacle_nodes = []
        obstacle_nodes.append(get_global_node(object_angle, object_pos, Node((cube_padding, cube_padding))))
        obstacle_nodes.append(get_global_node(object_angle, object_pos, Node((cube_padding, -cube_padding))))
        obstacle_nodes.append(get_global_node(object_angle, object_pos, Node((-cube_padding, -cube_padding))))
        obstacle_nodes.append(get_global_node(object_angle, object_pos, Node((-cube_padding, cube_padding))))
        cmap.add_obstacle(obstacle_nodes)
        marked[obj.object_id] = obj
        update_cmap = True

    return update_cmap, goal_center


class RobotThread(threading.Thread):
    """Thread to run cozmo code separate from main thread
    """

    def __init__(self):
        threading.Thread.__init__(self, daemon=True)

    def run(self):
        # Please refrain from enabling use_viewer since it uses tk, which must be in main thread
        cozmo.run_program(CozmoPlanning,use_3d_viewer=False, use_viewer=False)
        stopevent.set()


class RRTThread(threading.Thread):
    """Thread to run RRT separate from main thread
    """

    def __init__(self):
        threading.Thread.__init__(self, daemon=True)

    def run(self):
        while not stopevent.is_set():
            RRT(cmap, cmap.get_start())
            time.sleep(100)
            cmap.reset()
        stopevent.set()


if __name__ == '__main__':
    global cmap, stopevent
    stopevent = threading.Event()
    robotFlag = False
    for i in range(0,len(sys.argv)):
        if (sys.argv[i] == "-robot"):
            robotFlag = True
    if (robotFlag):
        cmap = CozMap("maps/emptygrid.json", node_generator)
        robot_thread = RobotThread()
        robot_thread.start()
    else:
        cmap = CozMap("maps/map2.json", node_generator)
        sim = RRTThread()
        sim.start()
    visualizer = Visualizer(cmap)
    visualizer.start()
    stopevent.set()

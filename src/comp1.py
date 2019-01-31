#!/usr/bin/env python

'''
based on example from demo2 , from https://eclass.srv.ualberta.ca/pluginfile.php/4926642/mod_folder/content/0/demo2.py?forcedownload=1
ideation based on 
'''

import rospy
from nav_msgs.msg import Odometry
from kobuki_msgs.msg import BumperEvent, Sound
from tf.transformations import decompose_matrix, compose_matrix
from ros_numpy import numpify
from sensor_msgs.msg import Joy, LaserScan
from geometry_msgs.msg import Twist
import numpy as np
from smach import State, StateMachine
import angles as angles_lib
import smach_ros
import math
import random

# used global to set turn direction to avoid passing it between states
turn_direction = 1

HARD_TURN_DISTANCE = 0.7
TURN_DISTANCE = 1.1
FORWARD_CURRENT = 0
TURN_CURRENT = 0
STATE_CHANGE_TIME = None


def calc_delta_vector(start_heading, distance):
    dx = distance * np.cos(start_heading)
    dy = distance * np.sin(start_heading)
    return np.array([dx, dy])


def check_forward_distance(forward_vec, start_pos, current_pos):
    current = current_pos - start_pos
    # vector projection (project current onto forward_vec)
    delta = np.dot(current, forward_vec) / \
        np.dot(forward_vec, forward_vec) * forward_vec
    dist = np.sqrt(delta.dot(delta))
    return dist


START_EVADE = False
START_PURSUE = False


def joy_callback(msg):
    global START_EVADE
    global START_PURSUE

    if msg.buttons[0] == 1:  # button A
        START_EVADE = True
        START_PURSUE = False
    elif msg.buttons[1] == 1:  # button B
        START_EVADE = False
        START_PURSUE = False
    elif msg.buttons[2] == 1:  # button X
        START_EVADE = False
        START_PURSUE = True


class WaitForButton(State):
    """
    Wait for button press, pass start pose (translation and rotation)
    out through userdata
    """

    def __init__(self):
        State.__init__(self, outcomes=["evade", "pursue"],
                       output_keys=["start_pose_out"])
        # TODO: change it back to False
        self.START = True
        # TODO: change this too
        self.pose = [0, 0, 0, 0]

        # pub / sub
        # rospy.Subscriber("joy", Joy, callback=self.joy_callback)
        rospy.Subscriber("odom", Odometry, callback=self.odom_callback)

    def execute(self, userdata):
        global START_EVADE
        global START_PURSUE

        while not START_EVADE and not START_PURSUE and not rospy.is_shutdown():
            continue
        userdata.start_pose_out = self.pose
        # START = False
        if START_EVADE:
            return "evade"
        elif START_PURSUE:
            return "pursue"

    def joy_callback(self, msg):
        if msg.buttons[0] == 1 or msg.buttons[2] == 1:
            self.START = True

    def odom_callback(self, msg):
        tb_pose = msg.pose.pose
        __, __, angles, position, __ = decompose_matrix(numpify(tb_pose))
        self.pose = [position[0:2], angles[2]]


# class Goal(State):
#     """
#     Keep track of goal heading and drive parallel to that direction
#     """

#     def __init__(self, distance=3.38):
#         State.__init__(self, outcomes=["end", "collision"],
#                        input_keys=['start_pose_in'])
#         self.cmd_pub = rospy.Publisher(
#             "cmd_vel_mux/input/teleop", Twist, queue_size=1)
#         self.COLLISION = False
#         self.distance = distance
#         self.tb_position = None
#         self.tb_rot = None

#         # pub / sub
#         rospy.Subscriber("odom", Odometry, callback=self.odom_callback)
#         rospy.Subscriber("/mobile_base/events/bumper",
#                          BumperEvent, callback=self.bumper_callback)
#         self.sound_pub = rospy.Publisher(
#             "/mobile_base/commands/sound", Sound, queue_size=1)

#     def bumper_callback(self, msg):
#         if msg.state:
#             self.COLLISION = True

#     def odom_callback(self, msg):
#         tb_pose = msg.pose.pose
#         self.tb_pose = numpify(tb_pose)
#         __, __, angles, position, __ = decompose_matrix(numpify(tb_pose))
#         self.tb_position = position[0:2]
#         self.tb_rot = angles

#     def execute(self, userdata):
#         self.COLLISION = False
#         start_heading = userdata.start_pose_in[1]
#         start_pos = userdata.start_pose_in[0]

#         forward_vec = calc_delta_vector(start_heading, self.distance)
#         rate = rospy.Rate(10)

#         while not rospy.is_shutdown():
#             if self.COLLISION:
#                 msg = Twist()
#                 msg.linear.x = 0.0
#                 self.cmd_pub.publish(msg)
#                 self.COLLISION = False
#                 return "collision"
#             if check_forward_distance(forward_vec, start_pos, self.tb_position) > self.distance:
#                 # stop robot and signal using sound
#                 msg = Twist()
#                 msg.linear.x = 0.0
#                 self.cmd_pub.publish(msg)
#                 msg = Sound()
#                 msg.value = 4
#                 self.sound_pub.publish(msg)
#                 return "end"

#             msg = Twist()
#             msg.linear.x = 0.2
#             turn_error = angles_lib.normalize_angle(
#                 start_heading - self.tb_rot[2])
#             msg.angular.z = 0

#             # p controller for driving straight
#             msg.angular.z = 1.0 * turn_error

#             self.cmd_pub.publish(msg)
#             rate.sleep()


class Translate(State):
    def __init__(self, distance=0.15, linear=-0.2):
        State.__init__(self, outcomes=["success", "collision", "quit"])
        self.tb_position = None
        self.tb_rot = None
        self.distance = distance
        self.COLLISION = False
        self.linear = linear

        # pub / sub
        self.cmd_pub = rospy.Publisher(
            "cmd_vel_mux/input/teleop", Twist, queue_size=1)
        rospy.Subscriber("odom", Odometry, callback=self.odom_callback)
        rospy.Subscriber("/mobile_base/events/bumper",
                         BumperEvent, callback=self.bumper_callback)

    def odom_callback(self, msg):
        tb_pose = msg.pose.pose
        __, __, angles, position, __ = decompose_matrix(numpify(tb_pose))
        self.tb_position = position[0:2]
        self.tb_rot = angles

    def bumper_callback(self, msg):
        if msg.state:
            self.COLLISION = True

    def execute(self, userdata):
        global turn_direction
        global START_EVADE
        if not START_EVADE:
            return 'quit'
        self.COLLISION = False
        start_heading = self.tb_rot[2]
        start_pos = self.tb_position
        forward_vec = calc_delta_vector(start_heading, self.distance)
        rate = rospy.Rate(30)

        while not rospy.is_shutdown():
            dist = check_forward_distance(
                forward_vec, start_pos, self.tb_position)
            if dist > self.distance:
                return "success"
            if self.linear > 0 and self.COLLISION:
                msg = Twist()
                msg.linear.x = 0.0
                self.cmd_pub.publish(msg)
                self.COLLISION = False
                # hit something while driving perpendicular so swap turn direction
                turn_direction = -1 * turn_direction
                return "collision"

            msg = Twist()
            msg.linear.x = self.linear
            self.cmd_pub.publish(msg)
            rate.sleep()


class Rotate(State):
    def __init__(self, angle=90):
        State.__init__(self, outcomes=["success", "quit"],
                       input_keys=['start_pose_in'])
        self.tb_position = None
        self.tb_rot = None
        # angle defines angle target relative to goal direction
        self.angle = angle

        # pub / sub
        rospy.Subscriber("odom", Odometry, callback=self.odom_callback)
        self.cmd_pub = rospy.Publisher(
            "cmd_vel_mux/input/teleop", Twist, queue_size=1)

    def odom_callback(self, msg):
        tb_pose = msg.pose.pose
        __, __, angles, position, __ = decompose_matrix(numpify(tb_pose))
        self.tb_position = position[0:2]
        self.tb_rot = angles

    def execute(self, userdata):
        global turn_direction
        global START_EVADE
        if not START_EVADE:
            return 'quit'
        start_pose = userdata.start_pose_in
        if self.angle == 0:  # target is goal + 0
            goal = start_pose[1]
        elif self.angle == 90:  # target is goal + turn_direction * 90
            goal = start_pose[1] + np.pi/2 * turn_direction

        goal = angles_lib.normalize_angle(goal)

        cur = np.abs(angles_lib.normalize_angle(self.tb_rot[2]) - goal)
        speed = 0.55
        rate = rospy.Rate(30)

        direction = turn_direction

        if self.angle == 0:
            direction = turn_direction * -1

        while not rospy.is_shutdown():
            cur = np.abs(angles_lib.normalize_angle(self.tb_rot[2]) - goal)

            # slow down turning as we get closer to the target heading
            if cur < 0.1:
                speed = 0.15
            if cur < 0.0571:
                break
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = direction * speed
            self.cmd_pub.publish(msg)
            rate.sleep()

        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.cmd_pub.publish(msg)

        return 'success'


class Evade(State):
    def __init__(self, distance=3.38, v_forward=0.5, v_turn=0, turn=False, state='Evade'):
        State.__init__(self, outcomes=["end", "collision", "turn", "hard_turn", "evade", "quit"],
                       input_keys=['start_pose_in'])
        self.COLLISION = False
        self.distance = distance
        self.tb_position = None
        self.tb_rot = None
        self.g_range_ahead = None
        self.rate = rospy.Rate(30)
        self.forward_target = v_forward
        self.turn_target = v_turn
        self.turn = turn
        self.state = state
        # pub / sub
        self.cmd_pub = rospy.Publisher(
            "cmd_vel_mux/input/teleop", Twist, queue_size=1)
        rospy.Subscriber("odom", Odometry, callback=self.odom_callback)
        rospy.Subscriber("/mobile_base/events/bumper",
                         BumperEvent, callback=self.bumper_callback)
        rospy.Subscriber("scan", LaserScan, callback=self.scan_callback)
        # self.sound_pub = rospy.Publisher(
        #     "/mobile_base/commands/sound", Sound, queue_size=1)

    def execute(self, userdata):
        global TURN_DISTANCE
        global HARD_TURN_DISTANCE
        global STATE_CHANGE_TIME
        global START_EVADE

        while not rospy.is_shutdown():

            if not START_EVADE:
                return 'quit'
            # if collision, resolve collision
            if self.COLLISION:
                msg = Twist()
                msg.linear.x = 0.0
                self.cmd_pub.publish(msg)
                self.COLLISION = False
                return "collision"

            # else, go straight
            # random_turn_direction = random.randint(0, 1)
            # if random_turn_direction == 0:
            #     random_turn_direction = -1
            move(self.forward_target,
                 self.turn_target, self.cmd_pub)

            # if find wall/object in middle range, start turn
            if self.g_range_ahead < HARD_TURN_DISTANCE:
                STATE_CHANGE_TIME = rospy.Time.now() + rospy.Duration(3)
                if self.state != 'HardTurn':
                    return self.start_hard_turn()

            # if find wall/object in super close rage or, start turn
            #  if timer runs out and we've been going straight, turn
            elif self.g_range_ahead < TURN_DISTANCE or (not self.turn and rospy.Time.now() > STATE_CHANGE_TIME):
                # elif self.g_range_ahead < TURN_DISTANCE:
                STATE_CHANGE_TIME = rospy.Time.now() + rospy.Duration(3)
                if self.state != 'Turn':
                    return self.start_turn()

            elif self.state != 'Evade':
                return 'evade'

            self.rate.sleep()

    def start_turn(self):
        return 'turn'

    def start_hard_turn(self):
        return 'hard_turn'

    def bumper_callback(self, msg):
        if msg.state:
            self.COLLISION = True

    def odom_callback(self, msg):
        tb_pose = msg.pose.pose
        self.tb_pose = numpify(tb_pose)
        __, __, angles, position, __ = decompose_matrix(numpify(tb_pose))
        self.tb_position = position[0:2]
        self.tb_rot = angles

    def scan_callback(self, msg):
        validList = [x for x in msg.ranges if not math.isnan(x)]
        validList.append(float('Inf'))
        # g range ahead will be the minimal range
        self.g_range_ahead = min(validList)

class Pursue(State):
    def __init__(self):
        State.__init__(
                        self,
                        outcomes=["end", "quit"],
                    )

        self.linear_distance = 1.0
        self.angular_distance = 0
        self.ramped_rate = 0.3

        self.publish_rate = rospy.Rate(30)
        self.goal_z = rospy.get_param("~goal_z", 0.7) # The goal distance (in meters) to keep between the robot and the person
        self.z_threshold = rospy.get_param("~z_threshold", 0.05) # How far away from the goal distance (in meters) before the robot reacts
        self.x_threshold = rospy.get_param("~x_threshold", 0.05) # How far away from being centered (x displacement) on the person before the robot reacts
        self.max_angular_speed = rospy.get_param("~max_angular_speed", 0.8) # The maximum rotation speed in radians per second
        self.min_angular_speed = rospy.get_param("~min_angular_speed", 0.0) # The minimum rotation speed in radians per second
        self.max_linear_speed = rospy.get_param("~max_linear_speed", 0.6) # The max linear speed in meters per second
        self.min_linear_speed = rospy.get_param("~min_linear_speed", 0.0) # The minimum linear speed in meters per second
        self.slow_down_factor = rospy.get_param("~slow_down_factor", 0.0) # Slow down factor when stopping

        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=5) # Publisher to control the robot's movement
        self.depth_subscriber = rospy.Subscriber('scan', LaserScan, self.set_cmd_vel, queue_size=1) # Subscribe to the point cloud

    def execute(self, userdata):
        global FORWARD_CURRENT
        global TURN_CURRENT
        global START_PURSUE

        # Publish the movement command
        
        FORWARD_CURRENT = 0.0
        TURN_CURRENT = 0.0

        while not rospy.is_shutdown():
            if not START_PURSUE:
                FORWARD_CURRENT = 0
                TURN_CURRENT = 0

                self.cmd_vel_pub.publish(Twist())
                return "quit"

            if self.linear_distance > 0.88:
                FORWARD_CURRENT = FORWARD_CURRENT + 0.1
            elif self.linear_distance < 0.84:
                FORWARD_CURRENT = FORWARD_CURRENT - 0.1
            else:
                FORWARD_CURRENT = 0

            if self.angular_distance < 180: # evader to the left
                TURN_CURRENT = TURN_CURRENT - 0.05
            elif self.angular_distance > 260:
                TURN_CURRENT = TURN_CURRENT + 0.05
            else:
                TURN_CURRENT = 0
            
            FORWARD_CURRENT = math.copysign(max(self.min_linear_speed, min(abs(FORWARD_CURRENT), self.max_linear_speed)), FORWARD_CURRENT)
            TURN_CURRENT = math.copysign(max(self.min_angular_speed, min(abs(TURN_CURRENT), self.max_angular_speed)), TURN_CURRENT)

            FORWARD_CURRENT = abs(FORWARD_CURRENT)
            
            move(FORWARD_CURRENT, TURN_CURRENT, self.cmd_vel_pub, self.ramped_rate)
            self.publish_rate.sleep()

    def set_cmd_vel(self, msg):
        # Initialize the centroid coordinates point count

        # triPoint = [i for i in range (0,121)]
        # triPoint = []

        # for i in range(0,181):
        #     triPoint.append(msg.ranges[(len(msg.ranges)/2) - 90 + i])

        object_detected = False
        self.linear_distance = 100

        for i in range(440):
            index = i + 100
            if (msg.ranges[index] < self.linear_distance and not math.isnan(msg.ranges[index]) and msg.ranges[index] > msg.range_min and msg.ranges[index] < msg.range_max):
                self.linear_distance = msg.ranges[index]
                self.angular_distance = i
                object_detected = True

        if not object_detected:
            self.linear_distance = 0.80
            self.angular_distance = 220


def move(forward_target, turn_target, pub, ramp_rate = 0.5):
    """
    modified version of move(forward, turn) from https://github.com/erichsueh/LifePoints-412-Comp1/blob/2e9fc4701c3cdc8e4ab8b04ca1da8581cfdf0c5b/robber_bot.py#L25
    """
    global FORWARD_CURRENT
    global TURN_CURRENT

    twist = Twist()
    new_forward = ramped_vel(FORWARD_CURRENT, forward_target, ramp_rate)
    new_turn = ramped_vel(TURN_CURRENT, turn_target, ramp_rate)
    twist.linear.x = new_forward
    twist.angular.z = new_turn
    pub.publish(twist)

    FORWARD_CURRENT = new_forward
    TURN_CURRENT = new_turn


def ramped_vel(v_prev, v_target, ramp_rate):
    """
    get the ramped velocity
    from rom https://github.com/MandyMeindersma/Robotics/blob/master/Competitions/Comp1/Evasion.py
    """
    if abs(v_prev) > abs(v_target):
        ramp_rate *= 2
    step = ramp_rate * 0.1
    sign = 1.0 if (v_target > v_prev) else -1.0
    error = math.fabs(v_target - v_prev)
    if error < step:  # we can get there in this time so we are done
        return v_target
    else:
        return v_prev + sign*step


if __name__ == "__main__":
    rospy.init_node("comp1")
    rospy.Subscriber("/joy", Joy, callback=joy_callback)
    STATE_CHANGE_TIME = rospy.Time.now()

    sm = StateMachine(outcomes=['success', 'failure'])
    sm.userdata.start_pose = None
    with sm:
        StateMachine.add("Wait", WaitForButton(), transitions={'evade': 'Evade', 'pursue': 'Pursue'},
                         remapping={'start_pose_out': 'start_pose'})
        # StateMachine.add("Parallel", Goal(), transitions={'end': 'Wait', 'collision': 'Backup'},
        #                  remapping={'start_pose_in': 'start_pose'})
        StateMachine.add("Evade", Evade(), transitions={'end': 'Wait', 'collision': 'Backup', 'evade': 'Evade', 'turn': 'Turn', 'hard_turn': 'HardTurn', 'quit': 'Wait'},
                         remapping={'start_pose_out': 'start_pose'})
        StateMachine.add("Turn", Evade(v_forward=0.2, v_turn=0.8, turn=True, state='Turn'), transitions={'quit': 'Wait', 'end': 'Wait', 'collision': 'Backup', 'evade': 'Evade', 'turn': 'Turn', 'hard_turn': 'HardTurn'},
                         remapping={'start_pose_out': 'start_pose'})
        StateMachine.add("HardTurn", Evade(v_forward=0, v_turn=1, turn=True, state='HardTurn'), transitions={'quit': 'Wait', 'end': 'Wait', 'collision': 'Backup', 'evade': 'Evade', 'turn': 'Turn', 'hard_turn': 'HardTurn'},
                         remapping={'start_pose_out': 'start_pose'})
        StateMachine.add("Backup", Translate(distance=0.15, linear=-0.2),
                         transitions={'success': 'TurnPerpendicular', 'collision': 'failure', 'quit': 'Wait'})
        StateMachine.add("TurnPerpendicular", Rotate(angle=90), transitions={'success': 'Perpendicular', 'quit': 'Wait'},
                         remapping={'start_pose_in': 'start_pose'})
        StateMachine.add("Perpendicular", Translate(distance=0.40, linear=0.2), transitions={'success': 'TurnParallel',
                                                                                             'collision': 'Backup', 'quit': 'Wait'})
        StateMachine.add("TurnParallel", Rotate(angle=0), transitions={'success': 'Evade', 'quit': 'Wait'},
                         remapping={'start_pose_in': 'start_pose'})
        StateMachine.add("Pursue", Pursue(), transitions={'end': 'Wait', 'quit': 'Wait'},
                         remapping={'start_pose_out': 'start_pose'})

    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()
    outcome = sm.execute()
    sis.stop()

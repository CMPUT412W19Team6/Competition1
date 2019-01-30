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

# used global to set turn direction to avoid passing it between states
turn_direction = 1

HARD_TURN_DISTANCE = 0.7
TURN_DISTANCE = 1.5
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


class WaitForButton(State):
    """
    Wait for button press, pass start pose (translation and rotation)
    out through userdata
    """

    def __init__(self):
        State.__init__(self, outcomes=["success"],
                       output_keys=["start_pose_out"])
        # TODO: change it back to False
        self.START = True
        self.pose = None

        # pub / sub
        rospy.Subscriber("joy", Joy, callback=self.joy_callback)
        rospy.Subscriber("odom", Odometry, callback=self.odom_callback)

    def execute(self, userdata):
        while not self.START and not rospy.is_shutdown():
            continue
        userdata.start_pose_out = self.pose
        self.START = False
        return "success"

    def joy_callback(self, msg):
        if msg.buttons[0] == 1:
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
        State.__init__(self, outcomes=["success", "collision"])
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
        State.__init__(self, outcomes=["success"],
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
    def __init__(self, distance=3.38, v_forward=0.8, v_turn=0, turn=False, state='Evade'):
        State.__init__(self, outcomes=["end", "collision", "turn", "hard_turn", "evade"],
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

        while not rospy.is_shutdown():
            # if collision, resolve collision
            if self.COLLISION:
                msg = Twist()
                msg.linear.x = 0.0
                self.cmd_pub.publish(msg)
                self.COLLISION = False
                return "collision"

            # if find wall/object in middle range, start turn
            if self.g_range_ahead < HARD_TURN_DISTANCE:
                STATE_CHANGE_TIME = rospy.Time.now() + rospy.Duration(3)
                if self.state != 'HardTurn':
                    return self.start_hard_turn()

            # if find wall/object in super close rage or, start turn
            #  if timer runs out and we've been going straight, turn
            # elif self.g_range_ahead < TURN_DISTANCE or (not self.turn and rospy.Time.now() > STATE_CHANGE_TIME):
            elif self.g_range_ahead < TURN_DISTANCE:
                STATE_CHANGE_TIME = rospy.Time.now() + rospy.Duration(3)
                if self.state != 'Turn':
                    return self.start_turn()

            # else, go straight
            move(self.forward_target, self.turn_target, self.cmd_pub)

            self.rate.sleep()
            if self.state != 'Evade':
                return 'evade'

    def start_turn(self):
        return 'turn'

    def start_hard_turn(self):
        return 'hard_turn'

    def bumper_callback(self, msg):
        if msg.state:
            self.COLLISION = True

    def odom_callback(self, msg):
        pass

    def scan_callback(self, msg):
        self.g_range_ahead = min(msg.ranges)


def move(forward_target, turn_target, pub):
    """
    modified version of move(forward, turn) from https://github.com/erichsueh/LifePoints-412-Comp1/blob/2e9fc4701c3cdc8e4ab8b04ca1da8581cfdf0c5b/robber_bot.py#L25
    """
    global FORWARD_CURRENT
    global TURN_CURRENT

    twist = Twist()
    ramp_rate = 0.5
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
    step = ramp_rate * 0.1
    sign = 1.0 if (v_target > v_prev) else -1.0
    error = math.fabs(v_target - v_prev)
    if error < step:  # we can get there in this time so we are done
        return v_target
    else:
        return v_prev + sign*step


if __name__ == "__main__":
    rospy.init_node("demo2")

    STATE_CHANGE_TIME = rospy.Time.now()

    sm = StateMachine(outcomes=['success', 'failure'])
    sm.userdata.start_pose = None
    with sm:
        StateMachine.add("Wait", WaitForButton(), transitions={'success': 'Evade'},
                         remapping={'start_pose_out': 'start_pose'})
        # StateMachine.add("Parallel", Goal(), transitions={'end': 'Wait', 'collision': 'Backup'},
        #                  remapping={'start_pose_in': 'start_pose'})
        StateMachine.add("Evade", Evade(), transitions={'end': 'Wait', 'collision': 'Backup', 'evade': 'Evade', 'turn': 'Turn', 'hard_turn': 'HardTurn'},
                         remapping={'start_pose_out': 'start_pose'})
        StateMachine.add("Turn", Evade(v_forward=0.2, v_turn=0.8, turn=True, state='Turn'), transitions={'end': 'Wait', 'collision': 'Backup', 'evade': 'Evade', 'turn': 'Turn', 'hard_turn': 'HardTurn'},
                         remapping={'start_pose_out': 'start_pose'})
        StateMachine.add("HardTurn", Evade(v_forward=0, v_turn=1, turn=True, state='HardTurn'), transitions={'end': 'Wait', 'collision': 'Backup', 'evade': 'Evade', 'turn': 'Turn', 'hard_turn': 'HardTurn'},
                         remapping={'start_pose_out': 'start_pose'})
        StateMachine.add("Backup", Translate(distance=0.15, linear=-0.2),
                         transitions={'success': 'TurnPerpendicular', 'collision': 'failure'})
        StateMachine.add("TurnPerpendicular", Rotate(angle=90), transitions={'success': 'Perpendicular'},
                         remapping={'start_pose_in': 'start_pose'})
        StateMachine.add("Perpendicular", Translate(distance=0.40, linear=0.2), transitions={'success': 'TurnParallel',
                                                                                             'collision': 'Backup'})
        StateMachine.add("TurnParallel", Rotate(angle=0), transitions={'success': 'Evade'},
                         remapping={'start_pose_in': 'start_pose'})

    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()
    outcome = sm.execute()
    sis.stop()

#!/usr/bin/env python
# BEGIN ALL
import rospy
import smach
import smach_ros
import math
from roslib import message
from std_msgs.msg import String
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class Idle(smach.State):
    def __init__(self):
        smach.State.__init__(
                                self,
                                outcomes=["start"],
                            )
    def execute(self, userdata):
        global is_node_active

        is_node_active = True
        return "start"

class Pursuit(smach.State):
    def __init__(self):
        smach.State.__init__(
                                self,
                                outcomes=["stop", "lost"],
                            )
    def execute(self, userdata):
        global is_node_active, linear_distance, angular_distance, cmd_vel_pub, goal_z, move_cmd, rate, min_linear_speed, max_linear_speed, min_angular_speed, max_angular_speed

        # Publish the movement command
        ramped_rate = 0.3

        while is_node_active:
            if linear_distance > 0.82:
                move_cmd.linear.x = self.ramped_vel(move_cmd.linear.x, move_cmd.linear.x + 0.1, ramped_rate)
            elif linear_distance < 0.78:
                move_cmd.linear.x = self.ramped_vel(move_cmd.linear.x, move_cmd.linear.x - 0.1, ramped_rate)
            else:
                move_cmd.linear.x = self.ramped_vel(move_cmd.linear.x, 0, ramped_rate)

            if angular_distance < 180: # evader to the left
                move_cmd.angular.z = self.ramped_vel(move_cmd.angular.z, move_cmd.angular.z - 0.05, ramped_rate)
            elif angular_distance > 260:
                move_cmd.angular.z = self.ramped_vel(move_cmd.angular.z, move_cmd.angular.z + 0.05, ramped_rate)
            else:
                move_cmd.angular.z = self.ramped_vel(move_cmd.angular.z, 0, ramped_rate)
            
            move_cmd.linear.x = math.copysign(max(min_linear_speed, min(abs(move_cmd.linear.x), max_linear_speed)), move_cmd.linear.x)
            move_cmd.angular.z = math.copysign(max(min_angular_speed, min(abs(move_cmd.angular.z), max_angular_speed)), move_cmd.angular.z)

            move_cmd.linear.x = abs(move_cmd.linear.x)
            
            cmd_vel_pub.publish(move_cmd)
            rate.sleep()

    def ramped_vel(self, v_prev, v_target, ramp_rate):
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

class Lost(smach.State):
    def __init__(self):
        smach.State.__init__(
                                self,
                                outcomes=["found", "failed"],
                            )
    def execute(self, userdata):
        pass

def set_cmd_vel(msg):
    global linear_distance, angular_distance, debug_pub, goal_z, z_threshold, move_cmd, min_linear_speed, max_linear_speed, min_angular_speed, max_angular_speed, slow_down_factor, x_threshold
    # Initialize the centroid coordinates point count

    # triPoint = [i for i in range (0,121)]
    # triPoint = []

    # for i in range(0,181):
    #     triPoint.append(msg.ranges[(len(msg.ranges)/2) - 90 + i])

    object_detected = False
    linear_distance = 100

    for i in range(440):
        index = i + 100
        if (msg.ranges[index] < linear_distance and not math.isnan(msg.ranges[index]) and msg.ranges[index] > msg.range_min and msg.ranges[index] < msg.range_max):
            linear_distance = msg.ranges[index]
            angular_distance = i
            object_detected = True

    print angular_distance, msg.ranges[angular_distance + 100]

    if not object_detected:
        linear_distance = 0.80
        angular_distance = 220

if __name__ == "__main__":
    rospy.init_node("pursuer")

    is_node_active = False
    rate = rospy.Rate(30)

    linear_distance = 0
    angular_distance = 0

    goal_z = rospy.get_param("~goal_z", 0.7) # The goal distance (in meters) to keep between the robot and the person
    z_threshold = rospy.get_param("~z_threshold", 0.05) # How far away from the goal distance (in meters) before the robot reacts
    x_threshold = rospy.get_param("~x_threshold", 0.05) # How far away from being centered (x displacement) on the person before the robot reacts
    max_angular_speed = rospy.get_param("~max_angular_speed", 0.8) # The maximum rotation speed in radians per second
    min_angular_speed = rospy.get_param("~min_angular_speed", 0.0) # The minimum rotation speed in radians per second
    max_linear_speed = rospy.get_param("~max_linear_speed", 0.6) # The max linear speed in meters per second
    min_linear_speed = rospy.get_param("~min_linear_speed", 0.0) # The minimum linear speed in meters per second
    slow_down_factor = rospy.get_param("~slow_down_factor", 0.0) # Slow down factor when stopping

    move_cmd = Twist() # Initialize the movement command

    cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=5) # Publisher to control the robot's movement
    depth_subscriber = rospy.Subscriber('scan', LaserScan, set_cmd_vel, queue_size=1) # Subscribe to the point cloud
    debug_pub = rospy.Publisher('/debug', String, queue_size=5) # Publisher to control the robot's movement

    rospy.loginfo("Subscribing to point cloud...") # Wait for the pointcloud topic to become available
    rospy.wait_for_message('scan', LaserScan)
    rospy.loginfo("Ready to follow!")

    sm = smach.StateMachine(outcomes=[]) # Create a SMACH state machine
    
    with sm: # Open the container
        # Add states to the container
        smach.StateMachine.add("IDLE", Idle(), 
                               transitions={"start":"PURSUIT"})
        smach.StateMachine.add("PURSUIT", Pursuit(), 
                               transitions={"stop":"IDLE", "lost":"LOST"})
        smach.StateMachine.add("LOST", Lost(), 
                               transitions={"found":"PURSUIT", "failed":"IDLE"})

    sis = smach_ros.IntrospectionServer('pursuer', sm, '/SM_ROOT/pursuer')
    sis.start()

    outcome = sm.execute() # Execute SMACH plan

    rospy.spin()

#!/usr/bin/env python
# BEGIN ALL
import rospy
import smach
import smach_ros
from roslib import message
from std_msgs.msg import String
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from math import copysign

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
        global is_node_active, cmd_vel_pub, move_cmd, rate

        # Publish the movement command
        while is_node_active:
            cmd_vel_pub.publish(move_cmd)
            rate.sleep()

class Lost(smach.State):
    def __init__(self):
        smach.State.__init__(
                                self,
                                outcomes=["found", "failed"],
                            )
    def execute(self, userdata):
        pass

def set_cmd_vel(msg):
    global debug_pub, min_y, max_y, max_x, min_x, max_z, goal_z, z_threshold, z_scale, move_cmd, min_linear_speed, max_linear_speed, min_angular_speed, max_angular_speed, slow_down_factor, x_threshold, x_scale
    # Initialize the centroid coordinates point count
    x = y = z = n = 0
    
    # Read in the x, y, z coordinates of all points in the cloud
    for point in point_cloud2.read_points(msg, skip_nans=True):
        pt_x = point[0]
        pt_y = point[1]
        pt_z = point[2]
        
        if -pt_y > min_y and -pt_y < max_y and pt_x < max_x and pt_x > min_x and pt_z < max_z:
            x += pt_x
            y += pt_y
            z += pt_z
            n += 1
    
    # If we have points, compute the centroid coordinates
    if n:
        x /= n 
        y /= n 
        z /= n
        
        debug_pub.publish("X: " + str(x) + " Z: " + str(z))
        
        # Check our movement thresholds
        if (abs(z - goal_z) > z_threshold):
            # Compute the angular component of the movement
            linear_speed = (z - goal_z) * z_scale
            
            # Make sure we meet our min/max specifications
            move_cmd.linear.x = copysign(max(min_linear_speed, min(max_linear_speed, abs(linear_speed))), linear_speed)
        else:
            move_cmd.linear.x *= slow_down_factor
            
        if (abs(x) > x_threshold):     
            # Compute the linear component of the movement
            angular_speed = x * x_scale
            
            # Make sure we meet our min/max specifications
            move_cmd.angular.z = copysign(max(min_angular_speed, min(max_angular_speed, abs(angular_speed))), angular_speed)
        else:
            # Stop the rotation smoothly
            move_cmd.angular.z *= slow_down_factor
            
    else:
        # Stop the robot smoothly
        move_cmd.linear.x *= slow_down_factor
        move_cmd.angular.z *= slow_down_factor

if __name__ == "__main__":
    rospy.init_node("pursuer")

    is_node_active = False
    rate = rospy.Rate(10)

    min_x = rospy.get_param("~min_x", -0.2)
    max_x = rospy.get_param("~max_x", 0.2)
    min_y = rospy.get_param("~min_y", -0.5)
    max_y = rospy.get_param("~max_y", 0.5)
    max_z = rospy.get_param("~max_z", 2.0)

    goal_z = rospy.get_param("~goal_z", 0.25) # The goal distance (in meters) to keep between the robot and the person
    z_threshold = rospy.get_param("~z_threshold", 0.05) # How far away from the goal distance (in meters) before the robot reacts
    x_threshold = rospy.get_param("~x_threshold", 0.05) # How far away from being centered (x displacement) on the person before the robot reacts
    z_scale = rospy.get_param("~z_scale", 1.0) # How much do we weight the goal distance (z) when making a movement
    x_scale = rospy.get_param("~x_scale", 4.5) # How much do we weight x-displacement of the person when making a movement  
    max_angular_speed = rospy.get_param("~max_angular_speed", 2.0) # The maximum rotation speed in radians per second
    min_angular_speed = rospy.get_param("~min_angular_speed", 0.0) # The minimum rotation speed in radians per second
    max_linear_speed = rospy.get_param("~max_linear_speed", 1.0) # The max linear speed in meters per second
    min_linear_speed = rospy.get_param("~min_linear_speed", 0.0) # The minimum linear speed in meters per second
    slow_down_factor = rospy.get_param("~slow_down_factor", 0.0) # Slow down factor when stopping

    move_cmd = Twist() # Initialize the movement command

    cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=5) # Publisher to control the robot's movement
    depth_subscriber = rospy.Subscriber('camera/depth/points', PointCloud2, set_cmd_vel, queue_size=1) # Subscribe to the point cloud
    debug_pub = rospy.Publisher('/debug', String, queue_size=5) # Publisher to control the robot's movement

    rospy.loginfo("Subscribing to point cloud...") # Wait for the pointcloud topic to become available
    rospy.wait_for_message('camera/depth/points', PointCloud2)
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

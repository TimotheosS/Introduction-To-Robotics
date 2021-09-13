#!/usr/bin/env python2
import rospy
import actionlib
import irob_assignment_1.msg
from irob_assignment_1.srv import GetSetpoint, GetSetpointRequest, GetSetpointResponse
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
import tf2_ros
import tf2_geometry_msgs
from math import atan2, sqrt

# Use to transform between frames
tf_buffer = None
listener = None

# The exploration simple action client
goal_client = None
# The collision avoidance service client
control_client = None
# The velocity command publisher
pub = None

# The robots frame
robot_frame_id = "base_link"

# Max linear velocity (m/s)
max_linear_velocity = 0.5
# Max angular velocity (rad/s)
max_angular_velocity = 1.0


def move(path):
    global control_client, robot_frame_id, pub
    while path.poses: 
        # Call service client with path
        response = control_client(path)
        new_path = response.new_path
        setpoint = response.setpoint

        # Transform Setpoint from service client
        transform = tf_buffer.lookup_transform(robot_frame_id, 'map' , rospy.Time(),rospy.Duration(1.0))
        transformed_setpoint = tf2_geometry_msgs.do_transform_point(setpoint, transform)

        # Create Twist message from the transformed Setpoint
        msg = Twist()

        msg.angular.z = 4.0 * atan2(transformed_setpoint.point.y, transformed_setpoint.point.x)
        msg.linear.x = 0.5 * sqrt(transformed_setpoint.point.x ** 2 + transformed_setpoint.point.y ** 2)

        # if(msg.angular.z > max_angular_velocity):
        #     msg.angular.z = max_angular_velocity
        if(msg.linear.x > max_linear_velocity):
            msg.linear.x = max_linear_velocity
            
        # Publish Twist
        pub.publish(msg)
        rate.sleep()
        # Call service client again if the returned path is not empty and do stuff again
        path = new_path            

    # Send 0 control Twist to stop robot
    msg.angular.z = 0
    msg.linear.x = 0
    pub.publish(msg)
    rate.sleep()


def get_path():
    global goal_client

    # Get path from action server
    ## Wait for a response from server
    action_client.wait_for_server()

    ## Get goal from the server
    goal = irob_assignment_1.msg.GetNextGoalGoal()
    # print(goal)

    ## Send the goal back to server (Make a request)
    action_client.send_goal(goal)
    action_client.wait_for_result()
    res=action_client.get_result()
    print(res)
    
    # Call move with path from action server
    move(res.path)
    return res

    ## Store the response of the server in the variable called path
    res=action_client.get_result()

    # Call move with path from action server
    move(res.path)
    return res



if __name__ == "__main__":
    # Init node
    rospy.init_node("controller")
    rate = rospy.Rate(10.0)

    # Init publisher
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    # Init simple action client
    action_client = actionlib.SimpleActionClient('get_next_goal', irob_assignment_1.msg.GetNextGoalAction)

    # Init service client
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    rospy.wait_for_service('get_setpoint')
    control_client = rospy.ServiceProxy('get_setpoint', GetSetpoint)


    ###################### While loop
    while True:
        # Call get path
        res = get_path()
        if(res.gain < 2):
            move(res.path)

    # Spin
    rospy.spin()
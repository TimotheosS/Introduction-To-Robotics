#!/usr/bin/env python

import py_trees as pt, py_trees_ros as ptr, rospy
from behaviours_student import *
from reactive_sequence import RSequence
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from std_srvs.srv import Empty, SetBool, SetBoolRequest
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import std_msgs.msg
from math import sqrt

Reset = False

class tuckarm_reset(pt.behaviour.Behaviour):

    """
    Sends a goal to the tuck arm action server.
    Returns running whilst awaiting the result,
    success if the action was succesful, and v.v..
    """

    def __init__(self):

        # Set up action client
        self.play_motion_ac = SimpleActionClient("/play_motion", PlayMotionAction)

        # personal goal setting
        self.goal = PlayMotionGoal()
        self.goal.motion_name = 'home'
        self.goal.skip_planning = True

        # execution checker
        self.sent_goal = False
        self.finished = False

        pt.common.Status.RUNNING

        # become a behaviour
        super(tuckarm_reset, self).__init__("Tuck arm!")

    def update(self):
        global Reset
        if Reset == True:
            self.sent_goal = False
            self.finished = False
            return pt.common.Status.SUCCESS
        # already tucked the arm
        if self.finished:
            return pt.common.Status.SUCCESS
 
        # command to tuck arm if haven't already
        elif not self.sent_goal:

            # send the goal
            self.play_motion_ac.send_goal(self.goal)
            self.sent_goal = True

            # tell the tree you're running
            return pt.common.Status.RUNNING

        # if I was succesful! :)))))))))
        elif self.play_motion_ac.get_result():

            # than I'm finished!
            self.finished = True
            return pt.common.Status.SUCCESS

        # if failed
        elif not self.play_motion_ac.get_result():
            return pt.common.Status.FAILURE

        # if I'm still trying :|
        else:
            return pt.common.Status.RUNNING

class loc(pt.behaviour.Behaviour):

    def __init__(self):
        rospy.loginfo("loc")

        self.loc_srv_nm = rospy.get_param(rospy.get_name() + '/global_loc_srv') 
        rospy.wait_for_service(self.loc_srv_nm, timeout=30)
        self.loc_srv = rospy.ServiceProxy(self.loc_srv_nm, Empty)

        self.cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
        self.clr_costmap_srv = rospy.get_param(rospy.get_name() + '/clear_costmaps_srv')
        rospy.wait_for_service(self.clr_costmap_srv, timeout=30)

        self.clear_costmap_srv = rospy.ServiceProxy(self.clr_costmap_srv, Empty)
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)


        # execution checker
        self.localizing = False
        self.finished = False


        pt.common.Status.RUNNING
        super(loc, self).__init__("loc!")


    def update(self):


        # already picked the cube
        if self.finished: 
            return pt.common.Status.SUCCESS
        
        # command to pick the cube if haven't already
        elif not self.localizing:

       
            # send the goal
            self.local_req = self.loc_srv()
            self.move_msg = Twist()
            self.move_msg.angular.z = -2

            rate = rospy.Rate(10)
            cnt = 0
            while not rospy.is_shutdown() and cnt < 60:
                self.cmd_vel_pub.publish(self.move_msg)
                rate.sleep()
                cnt = cnt + 1
            
            rospy.loginfo("localizing")
            self.clearCostMapRequest = self.clear_costmap_srv()
            self.localizing = True
            self.finished = True

            # tell the tree you're running
            return pt.common.Status.SUCCESS

        # if I'm still trying :|
        else:
            return pt.common.Status.RUNNING

class verifyPosition(pt.behaviour.Behaviour):



    def __init__(self, name):

        rospy.loginfo("Initialising verify position behaviour.")
        self.estimatePos = rospy.get_param(rospy.get_name() + '/amcl_estimate')
        self.detecting_cube = rospy.Subscriber(self.estimatePos, PoseWithCovarianceStamped, self.position_msg)
        self.loc_srv_nm = rospy.get_param(rospy.get_name() + '/global_loc_srv')
        self.cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
        self.clr_costmap_srv = rospy.get_param(rospy.get_name() + '/clear_costmaps_srv')
        rospy.wait_for_service(self.loc_srv_nm, timeout=30)
        rospy.wait_for_service(self.clr_costmap_srv, timeout=30)

        self.loc_srv = rospy.ServiceProxy(self.loc_srv_nm, Empty)
        self.clear_costmap_srv = rospy.ServiceProxy(self.clr_costmap_srv, Empty)
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)

        # become a behaviour
        super(verifyPosition, self).__init__("Check Position using amcl")

    def update(self):
        rospy.loginfo("Check the position")
        self.cnt = 0
        self.updatePose = False
        while not rospy.is_shutdown() and self.updatePose == False and self.cnt < 6:
            
            rospy.loginfo("Detect if the robot knows its position")
            self.cnt += 1
            rospy.sleep(1.0)
        if self.cnt < 6:
            rospy.loginfo("We are on truck")
            return pt.common.Status.RUNNING
        else:
            rospy.loginfo("Relocalization")
            self.locRequest = self.loc_srv()
            self.move_msg = Twist()
            self.move_msg.angular.z = -2

            rate = rospy.Rate(10)
            self.count = 0
            while not rospy.is_shutdown() and self.count < 60:
                self.cmd_vel_pub.publish(self.move_msg)
                rate.sleep()
                self.count = self.count + 1

            rospy.loginfo("Clear Cost Map...")
            self.clearCostMapRequest = self.clear_costmap_srv()
            return pt.common.Status.RUNNING

    def position_msg(self, pose_msg):
        self.robot_pose = pose_msg
        self.updatePose = True

class movehead_r(pt.behaviour.Behaviour):

    """
    Lowers or raises the head of the robot.
    Returns running whilst awaiting the result,
    success if the action was succesful, and v.v..
    """

    def __init__(self, direction):

        #rospy.loginfo("Initialising move head behaviour.")

        # server
        mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')
        self.move_head_srv = rospy.ServiceProxy(mv_head_srv_nm, MoveHead)
        rospy.wait_for_service(mv_head_srv_nm, timeout=30)

        # head movement direction; "down" or "up"
        self.direction = direction

        # execution checker
        self.tried = False
        self.finished = False

        # become a behaviour
        super(movehead_r, self).__init__("Lower head!")

    def update(self):
        global Reset
        if Reset == True:
            self.tried = False
            self.finished = False
            return pt.common.Status.SUCCESS
        #rospy.loginfo("You reached movehead.")

        # success if finished
        if self.finished:
            return pt.common.Status.SUCCESS

        # try if not tried
        elif not self.tried:

            # command
            self.move_head_req = self.move_head_srv(self.direction)
            self.tried = True

            # tell the tree you're running
            return pt.common.Status.RUNNING

        # if succesful
        elif self.move_head_req.success:
            self.finished = True
            return pt.common.Status.SUCCESS

        # if failed
        elif not self.move_head_req.success:
            return pt.common.Status.FAILURE

        # if still trying
        else:
            return pt.common.Status.RUNNING

class pick(pt.behaviour.Behaviour):

    def __init__(self):

        # Initialize, call and wait for the pick service
        self.pick_srv_nm = rospy.get_param(rospy.get_name() + '/pick_srv') 
        rospy.wait_for_service(self.pick_srv_nm, timeout=30)
        self.pick_srv = rospy.ServiceProxy(self.pick_srv_nm, SetBool)
        
        # execution checker
        self.picking = False
        self.finished = False

        # become a behaviour
        super(pick, self).__init__("Pick!")

    def update(self):
        global Reset
        if Reset == True:
            self.finished = False
            return pt.common.Status.SUCCESS

        # already picked the cube
        if self.finished: 
            return pt.common.Status.SUCCESS
        
        # command to pick the cube if haven't already
        elif not self.picking:

            # send the goal
            self.picking = True
            self.pick_req = self.pick_srv(True)

            # tell the tree you're running
            return pt.common.Status.RUNNING

        # if I was succesful! :)))))))))
        elif self.pick_req.success == True:

            # than I'm finished!
            self.finished = True
            return pt.common.Status.SUCCESS

        # if failed
        elif not self.pick_req.success:
            return pt.common.Status.FAILURE

        # if I'm still trying :|
        else:
            return pt.common.Status.RUNNING

class respawnCube(pt.behaviour.Behaviour):

    def __init__(self):

        rospy.wait_for_service("/gazebo/set_model_state", timeout=30)

        self.respawnCube = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        self.cube_ModelState = ModelState()
        self.cube_ModelState.model_name = "aruco_cube"
        self.cube_ModelState.pose.position.x = -1.130530
        self.cube_ModelState.pose.position.y = -6.653650
        self.cube_ModelState.pose.position.z = 0.86250
        self.cube_ModelState.pose.orientation.x = 0
        self.cube_ModelState.pose.orientation.y = 0
        self.cube_ModelState.pose.orientation.z = 0
        self.cube_ModelState.pose.orientation.w = 1
        self.cube_ModelState.twist.linear.x = 0
        self.cube_ModelState.twist.linear.y = 0
        self.cube_ModelState.twist.linear.z = 0
        self.cube_ModelState.twist.angular.x = 0
        self.cube_ModelState.twist.angular.y = 0
        self.cube_ModelState.twist.angular.z = 0
        self.cube_ModelState.reference_frame = "map"

        self.mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')
        self.move_head_srv = rospy.ServiceProxy(self.mv_head_srv_nm, MoveHead)
        rospy.wait_for_service(self.mv_head_srv_nm, timeout=30)

        # head movement direction
        self.direction = "up"

        super(respawnCube, self).__init__("Respawn Cube")

    def update(self):
        global Reset
        rospy.loginfo("Respawn Cube and raise head")
        self.move_head_req = self.move_head_srv(self.direction)
        self.respawnCube_request = self.respawnCube(self.cube_ModelState)
        Reset = True
        return pt.common.Status.RUNNING

class verifyPlacement(pt.behaviour.Behaviour):

    def __init__(self, name):

        self.cube_wrong = True
        self.aruco_pose_top = rospy.get_param(rospy.get_name() + '/aruco_pose_topic')
        
        # become a behaviour
        super(verifyPlacement, self).__init__(name)


    def update(self):
        global Reset
        if Reset == True:
            Reset = False
            self.cube_wrong = False
            return pt.common.Status.RUNNING
        
        self.aruco_pose_subs = rospy.Subscriber(self.aruco_pose_top, PoseStamped, self.aruco_pose_cb)
        if self.cube_wrong == True:
            rospy.loginfo("It failed to place the cube correctly")
            return pt.common.Status.FAILURE
            
        else:
            rospy.loginfo("It succeeded to place the cube correctly")
            return pt.common.Status.SUCCESS
            

    def aruco_pose_cb(self, aruco_pose_msg):
        self.aruco_pose = aruco_pose_msg
        self.cube_wrong = False

class place(pt.behaviour.Behaviour):

    def __init__(self):

        # Initialize, call and wait for the place service
        self.place_srv_nm = rospy.get_param(rospy.get_name() + '/place_srv') 
        rospy.wait_for_service(self.place_srv_nm, timeout=30)
        self.place_srv = rospy.ServiceProxy(self.place_srv_nm, SetBool)

        # execution checker
        self.placing = False
        self.finished = False

        # become a behaviour
        super(place, self).__init__("Place!")

    def update(self):      
        global Reset
        if Reset == True:
            self.finished = False
            return pt.common.Status.SUCCESS

        # already placed the cube
        if self.finished: 
            return pt.common.Status.SUCCESS
        
        # command to place the cube if haven't already
        elif not self.placing:

            # send the goal
            self.placing = True
            self.place_req = self.place_srv(True)

            # tell the tree you're running
            return pt.common.Status.RUNNING

        # if I was succesful! :)))))))))
        elif self.place_req.success:

            # than I'm finished!
            self.finished = True
            return pt.common.Status.SUCCESS

        # if failed
        elif not self.place_req.success:
            return pt.common.Status.FAILURE

        # if I'm still trying :|
        else:
            
            return pt.common.Status.RUNNING

class navigatePick(pt.behaviour.Behaviour):

    def __init__(self):

        rospy.loginfo("Initialising navigation towards the goal")

        self.pickPose_topic = rospy.get_param(rospy.get_name() + '/pick_pose_topic')
        self.pose = rospy.wait_for_message(self.pickPose_topic, PoseStamped, 30)

        self.moveBaseAction = SimpleActionClient("/move_base", MoveBaseAction)
        if not self.moveBaseAction.wait_for_server(rospy.Duration(1000)):
            rospy.logerr("Could not connect to /move_base action server")
            exit()
        rospy.loginfo("Connected to move_base action server")

        self.finished = False

        super(navigatePick, self).__init__("Navigation to cube!")

    def update(self):
        global Reset
        if Reset == True:
            self.finished = False
            return pt.common.Status.SUCCESS
        # success if finished
        if self.finished:
            return pt.common.Status.SUCCESS
        else:
            self.moveGoal = MoveBaseGoal()
            self.moveGoal.target_pose = self.pose
            self.moveBaseAction.send_goal(self.moveGoal)
            self.navResult = self.moveBaseAction.wait_for_result(rospy.Duration(6.0))
            # if succesful
            if self.navResult:
                self.finished = True
                return pt.common.Status.SUCCESS

            # if failed
            else:
                self.moveBaseAction.cancel_goal()
                rospy.loginfo("Failed to navigate")
                return pt.common.Status.FAILURE

class navigatePlace(pt.behaviour.Behaviour):

    def __init__(self):

        rospy.loginfo("Initialising navigatePlace behaviour.")
        self.placePose_topic = rospy.get_param(rospy.get_name() + '/place_pose_topic')
        self.pose = rospy.wait_for_message(self.placePose_topic, PoseStamped, 30)

        self.moveBaseAction = SimpleActionClient("/move_base", MoveBaseAction)
        if not self.moveBaseAction.wait_for_server(rospy.Duration(1000)):
            rospy.logerr("Could not connect to /move_base action server")
            exit()
        rospy.loginfo("Connected to move_base action server")

        self.finished = False

        super(navigatePlace, self).__init__("Navigation to table!")

    def update(self):
        global Reset
        if Reset == True:
            self.finished = False
            return pt.common.Status.SUCCESS
        if self.finished:
            return pt.common.Status.SUCCESS

        else:
            self.moveGoal = MoveBaseGoal()
            self.moveGoal.target_pose = self.pose
            self.moveBaseAction.send_goal(self.moveGoal)
            self.navResult = self.moveBaseAction.wait_for_result(rospy.Duration(10.0))
            # if succesful
            if self.navResult:
                self.finished = True
                return pt.common.Status.SUCCESS

            # if failed
            else:
                self.moveBaseAction.cancel_goal()
                rospy.loginfo("Failed to navigate.")
                return pt.common.Status.FAILURE

class BehaviourTree(ptr.trees.BehaviourTree):

	def __init__(self):

		rospy.loginfo("Initialising behaviour tree")

		# tuck the arm
		tuckArm = tuckarm_reset()

		# lower head
		moveHeadDown = movehead_r("down")

		# # pick cube
		pickCube = pick()

		placeCube = place()

		localization = loc()

		moveHeadUp = movehead_r("up")

		navigationToPick = pt.composites.Selector(
			name="Navigation to cube",
			children=[navigatePick(),verifyPosition("cube")]
		)

		navigationToPlace = pt.composites.Selector(
			name="Navigation to table",
			children=[navigatePlace(), verifyPosition("table")]
		)

		verifyCube = pt.composites.Selector(
			name="Repeat process",
			children=[verifyPlacement('Place cube succeeded'), respawnCube()]
		)

		# become the tree
		tree = RSequence(name="Main sequence", children=[tuckArm,localization,navigationToPick,moveHeadDown,pickCube,moveHeadUp,navigationToPlace,placeCube,moveHeadDown,verifyCube,moveHeadUp])
		super(BehaviourTree, self).__init__(tree)

		# execute the behaviour tree
		rospy.sleep(5)
		self.setup(timeout=10000)
		while not rospy.is_shutdown(): self.tick_tock(1)	

if __name__ == "__main__":


	rospy.init_node('main_state_machine')
	try:
		BehaviourTree()
	except rospy.ROSInterruptException:
		pass

	rospy.spin()

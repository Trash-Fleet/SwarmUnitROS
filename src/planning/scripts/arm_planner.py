#!/usr/bin/env python
import copy

import sys
import rospy
import numpy as np
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf

from math import pi
from std_msgs.msg import Float64, Bool, String
from geometry_msgs.msg import Pose
from moveit_commander.conversions import pose_to_list
from scipy import interpolate 

from tf.transformations import euler_from_quaternion, quaternion_from_euler

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True


class MoveGroupPythonIntefaceTutorial(object):
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()

    ## BEGIN_SUB_TUTORIAL setup
    ##
    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    # rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

    ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
    ## kinematic model and the robot's current joint states
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
    ## for getting, setting, and updating the robot's internal understanding of the
    ## surrounding world:
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to a planning group (group of joints).  In this tutorial the group is the primary
    ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
    ## If you are using a different robot, change this value to the name of your robot
    ## arm planning group.
    ## This interface can be used to plan and execute motions:
    group_name = "arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
    ## trajectories in Rviz:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    ## END_SUB_TUTORIAL

    ## BEGIN_SUB_TUTORIAL basic_info
    ##
    ## Getting Basic Information
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    # We can get the name of the reference frame for this robot:
    planning_frame = move_group.get_planning_frame()
    print "============ Planning frame: %s" % planning_frame

    # We can also print the name of the end-effector link for this group:
    eef_link = move_group.get_end_effector_link()
    print "============ End effector link: %s" % eef_link

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print "============ Available Planning Groups:", robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print "============ Printing robot state"
    print robot.get_current_state()
    print ""
    ## END_SUB_TUTORIAL

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names


  # Debug purpose
  def go_to_joint_state(self):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    ## BEGIN_SUB_TUTORIAL plan_to_joint_state
    ##
    ## Planning to a Joint Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^^
    ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_ so the first
    ## thing we want to do is move it to a slightly better configuration.
    # We can get the joint values from the group and adjust some of the values:
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = pi/8
    joint_goal[2] = pi/8
    joint_goal[3] = pi/8
    joint_goal[4] = 0
    # joint_goal[5] = pi/3
    # joint_goal[6] = 0

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    move_group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    move_group.stop()

    ## END_SUB_TUTORIAL

    # For testing:
    current_joints = move_group.get_current_joint_values()

    print self.move_group.get_current_pose().pose

    return all_close(joint_goal, current_joints, 0.01)


  def go_to_pose_goal(self, x, y, z):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    ## BEGIN_SUB_TUTORIAL plan_to_pose
    ##
    ## Planning to a Pose Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:
    # pose_goal = geometry_msgs.msg.Pose()
    # pose_goal.orientation.x = -0.151
    # pose_goal.orientation.y = 0.378
    # pose_goal.orientation.z = 0.339
    # pose_goal.orientation.w = 0.848
    # pose_goal.position.x = 0.287
    # pose_goal.position.y = 0.208
    # pose_goal.position.z = 0.479

    # move_group.clear_path_constraints()
    move_group.clear_pose_targets()

    pose_goal = self.move_group.get_current_pose().pose
    pose_goal.position.x = x
    pose_goal.position.y = y
    pose_goal.position.z = z

    yaw = np.arctan2(pose_goal.position.y, pose_goal.position.x)
    q = tf.transformations.quaternion_from_euler(0, 0, yaw)
    pose_goal.orientation.x = q[0]
    pose_goal.orientation.y = q[1]
    pose_goal.orientation.z = q[2]
    pose_goal.orientation.w = q[3]

    # pose_goal = move_group.get_random_pose().pose
    # move_group.set_pose_target(pose_goal)
    # move_group.set_joint_value_target(pose_goal, "gripper_link")
    move_group.set_joint_value_target(pose_goal, True)
    move_group.set_goal_tolerance(0.01)
    plan = move_group.plan()

    ## Now, we call the planner to compute the plan and execute it.
    # plan = move_group.go(wait=True)
    move_group.execute(plan, wait=True)

    # display trajectory on rviz
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = self.robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    self.display_trajectory_publisher.publish(display_trajectory);

    # Calling `stop()` ensures that there is no residual movement
    move_group.stop()
    
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    move_group.clear_pose_targets()

    print self.move_group.get_current_pose().pose

    ## END_SUB_TUTORIAL

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_pose = self.move_group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)


class ArmPlanner(object):
    def __init__(self):
        ### Initialization##
        ####################

        # initialize attributes
        self.arm_state = None
        self.arm_task = ""
        self.move_up_height = 0.3
        self.move_forward_length = 0.05

        # initialize node
        rospy.init_node('arm_planner', anonymous=True)

        # initialize ROS publishers and subscribers
        # Initialize subscribers
        rospy.Subscriber('/arm/target_pose', Pose, self.pose_callback)
        rospy.Subscriber('/arm/task', String, self.task_callback)

        # Initialize publishers
        self.arm_done_pub = rospy.Publisher('/arm/done', Bool, queue_size=10)
        self.gripper_pub = rospy.Publisher('/gripper_joint/command', Float64, queue_size=10)

        # initialize tf listener
        self.listener = tf.TransformListener()

        # initializat the arm class
        self.arm = MoveGroupPythonIntefaceTutorial()
        print("initialized PythonInterface")

        # start main loop
        rospy.spin()  

    def pose_callback(self, msg):
        print("sending pose message")
        #wait for a task type
        if(self.arm_task != ""):
          pass
        
        # Get all the goal position parameters
        goal_position_x = msg.position.x
        goal_position_y = msg.position.y 
        goal_position_z = msg.position.z 
        goal_orientation_x = msg.orientation.x 
        goal_orientation_y = msg.orientation.y
        goal_orientation_z = msg.orientation.z
        goal_orientation_w = msg.orientation.w 
        goal_position = [goal_position_x,goal_position_y,goal_position_z]

        # Smooth the trajectory
        # current_position_x = self.arm.move_group.get_current_pose().pose.position.x
        # current_position_y = self.arm.move_group.get_current_pose().pose.position.y
        # current_position_z = self.arm.move_group.get_current_pose().pose.position.z
        # current_position = [current_position_x,current_position_y,current_position_z]
        # trajectory = self.create_trajectory(current_position, goal_position)
        
        # Execute the trajectory
        if(self.arm_task == "grab"):
          self.open_gripper()
          # for waypoints in trajectory:
          #   self.arm.go_to_pose_goal(waypoints[0], waypoints[1], waypoints[2])
          self.arm.go_to_pose_goal(goal_position_x, goal_position_y, goal_position_z)
          self.move_forward(goal_position_x, goal_position_y, goal_position_z)
          rospy.sleep(2)
          self.close_gripper()
          rospy.sleep(2)
          self.move_up(goal_position_x, goal_position_y, goal_position_z)
        elif(self.arm_task == "drop"):
          # for waypoints in trajectory:
          #   self.arm.go_to_pose_goal(waypoints[0], waypoints[1], waypoints[2])
          self.arm.go_to_pose_goal(goal_position_x, goal_position_y, goal_position_z)
          rospy.sleep(2)
          self.open_gripper()
          rospy.sleep(2)
          self.move_up(goal_position_x, goal_position_y, goal_position_z)
        else:
          self.arm.go_to_pose_goal(goal_position_x, goal_position_y, goal_position_z)

        # Specify the arm movement is done and reset the arm_task
        self.arm_task = ""
        self.arm_done_pub.publish(True)

    def task_callback(self, msg):
        self.arm_task = msg.data

    def create_trajectory(self, current_position, goal_position):
        x = np.array([current_position[0], goal_position[0]])
        y = np.array([current_position[1], goal_position[1]])
        z = np.array([current_position[2], goal_position[2]])
        current_position = [x, y, z]

        result = interpolate.splprep(current_position, s=0, k=1)
        x_i, y_i, z_i = interpolate.splev(np.linspace(0, 1, 10), result[0])
        trajectory = np.zeros((len(x_i), 3))
        for index in range(0, len(x_i)):
            trajectory[index] = [x_i[index], y_i[index], z_i[index]]
        print(trajectory)
        return trajectory

    def open_gripper(self):
        self.gripper_pub.publish(0)
        
        # move_group = self.arm.move_group
        # joint_goal = move_group.get_current_joint_values()
        # joint_goal[4] = 0
        
        # move_group.go(joint_goal, wait=True)
        # move_group.stop()

        # # For testing:
        # current_joints = move_group.get_current_joint_values()

        # return all_close(joint_goal, current_joints, 0.01)

    def close_gripper(self):
        self.gripper_pub.publish(-1.0)
        
        # move_group = self.arm.move_group
        # joint_goal = move_group.get_current_joint_values()
        # joint_goal[4] = -1.0
        
        # move_group.go(joint_goal, wait=True)
        # move_group.stop()

        # # For testing:
        # current_joints = move_group.get_current_joint_values()
        
        # return all_close(joint_goal, current_joints, 0.01)

    def move_forward(self, goal_position_x, goal_position_y, goal_position_z):
        # current_position_x = self.arm.move_group.get_current_pose().pose.position.x
        # current_position_y = self.arm.move_group.get_current_pose().pose.position.y
        # current_position_z = self.arm.move_group.get_current_pose().pose.position.z
        # current_position = [current_position_x,current_position_y,current_position_z]

        yaw = np.arctan2(goal_position_y, goal_position_x)
        diff_x = self.move_forward_length * np.cos(yaw)
        diff_y = self.move_forward_length * np.sin(yaw)

        goal_position = [goal_position_x + diff_x,goal_position_y + diff_y,goal_position_z]
        self.arm.go_to_pose_goal(goal_position[0], goal_position[1], goal_position[2])

    def move_up(self, goal_position_x, goal_position_y, goal_position_z):
        # current_position_x = self.arm.move_group.get_current_pose().pose.position.x
        # current_position_y = self.arm.move_group.get_current_pose().pose.position.y
        # current_position_z = self.arm.move_group.get_current_pose().pose.position.z
        # current_position = [current_position_x,current_position_y,current_position_z]

        goal_position = [goal_position_x,goal_position_y,goal_position_z+self.move_up_height]
        self.arm.go_to_pose_goal(goal_position[0], goal_position[1], goal_position[2])

        # trajectory = self.create_trajectory(current_position, goal_position)

        # for waypoints in trajectory:
        #     self.arm.go_to_pose_goal(waypoints[0], waypoints[1], waypoints[2])

if __name__ == '__main__':
    try:
        ArmPlanner()
    except rospy.ROSInterruptException:
        pass


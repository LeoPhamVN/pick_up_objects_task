#!/usr/bin/env python

import py_trees
import py_trees.decorators
import py_trees.display
from py_trees.blackboard import Blackboard

import rospy
import tf
import numpy as np
import operator 
import time

from std_srvs.srv import Trigger, TriggerRequest
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

points_list = [np.array([1.25, 0.5]),
               np.array([1.25,-1.25]),
               np.array([ 0.0,-1.25]),
               np.array([-0.5, 1.25]),
               np.array([-1.25, 0.5])]

destination_dict = {"beer": np.array([-1.5,-1.5]),
                    "coke": np.array([ 1.5, 1.5])}
                   
# Behavior for calling `check_object` task and if True, store object name to Blackboard
class CheckObject(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(CheckObject, self).__init__(name)
        self.blackboard = Blackboard()
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            "object_name", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(
            "goal", access=py_trees.common.Access.WRITE)

    def setup(self):
        self.logger.debug("  %s [CheckObject::setup()]" % self.name)
        rospy.wait_for_service('/manage_objects/check_object')
        try:
            self.server = rospy.ServiceProxy(
                '/manage_objects/check_object', Trigger)
            self.logger.debug(
                "  %s [CheckObject::setup() Server connected!]" % self.name)
        except rospy.ServiceException as e:
            self.logger.debug("  %s [CheckObject::setup() ERROR!]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [CheckObject::initialise()]" % self.name)

    def update(self):
        try:
            self.logger.debug(
                "  {}: call service /manage_objects/check_object".format(self.name))
            resp = self.server(TriggerRequest())
            if resp.success:
                self.blackboard.object_name = resp.message
                print("set to blackboard: ", resp.message)
                # Define goal position
                self.goal = PoseStamped()
                self.blackboard.goal = destination_dict[self.blackboard.object_name]   
                return py_trees.common.Status.SUCCESS
            else:
                return py_trees.common.Status.FAILURE
        except:
            self.logger.debug(
                "  {}: Error calling service /manage_objects/check_object".format(self.name))
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug("  %s [CheckObject::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))

# Behavior for calling `get_object`
class GetObject(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(GetObject, self).__init__(name)

    def setup(self):
        self.logger.debug("  %s [GetObject::setup()]" % self.name)
        rospy.wait_for_service('/manage_objects/get_object')
        try:
            self.server = rospy.ServiceProxy(
                '/manage_objects/get_object', Trigger)
            self.logger.debug(
                "  %s [GetObject::setup() Server connected!]" % self.name)
        except rospy.ServiceException as e:
            self.logger.debug("  %s [GetObject::setup() ERROR!]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [GetObject::initialise()]" % self.name)

    def update(self):
        try:
            self.logger.debug(
                "  {}: call service /manage_objects/get_object".format(self.name))
            resp = self.server(TriggerRequest())
            if resp.success:
                return py_trees.common.Status.SUCCESS
            else:
                return py_trees.common.Status.FAILURE
        except:
            self.logger.debug(
                "  {}: Error calling service /manage_objects/get_object".format(self.name))
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug("  %s [GetObject::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))

# Behavior for calling `let_object`
class LetObject(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(LetObject, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            "object_name", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(
            "n_collected_objects", access=py_trees.common.Access.WRITE)
        
    def setup(self):
        self.logger.debug("  %s [LetObject::setup()]" % self.name)

        self.blackboard.n_collected_objects = 0

        rospy.wait_for_service('/manage_objects/let_object')
        try:
            self.server = rospy.ServiceProxy(
                '/manage_objects/let_object', Trigger)
            self.logger.debug(
                "  %s [LetObject::setup() Server connected!]" % self.name)
        except rospy.ServiceException as e:
            self.logger.debug("  %s [LetObject::setup() ERROR!]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [LetObject::initialise()]" % self.name)
        self.blackboard.n_collected_objects += 1

    def update(self):
        try:
            self.logger.debug(
                "  {}: call service /manage_objects/let_object".format(self.name))
            resp = self.server(TriggerRequest())
            if resp.success:
                return py_trees.common.Status.SUCCESS
            else:
                return py_trees.common.Status.FAILURE
        except:
            self.logger.debug(
                "  {}: Error calling service /manage_objects/let_object".format(self.name))
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug("  %s [LetObject::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))

# Behavior for calling `move_robot`
class MoveRobot(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(MoveRobot, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            "n_points", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(
            "goal", access=py_trees.common.Access.READ)
        # Current robot SE2 pose [x, y, yaw], None if unknown            
        self.current_pose = None

    def setup(self):
        self.logger.debug("  %s [MoveRobot::setup()]" % self.name)
        # Initialise number of points robot already went to is 0
        self.blackboard.n_points = 0

        # PUBLISHERS
        # Publisher for sending goal position to the planning node
        self.goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

        # SUBSCRIBERS
        #subscriber to robot pose from odometry  
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.get_odom) 

        # Wait 0.2s to init pub and sub
        time.sleep(0.2)
        self.logger.debug("  %s [MoveRobot::setup() SUCCESS]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [MoveRobot::initialise()]" % self.name)

        # Define goal position
        self.goal = PoseStamped()
        self.goal.header.frame_id = "map"
        self.goal.pose.position.x = self.blackboard.goal[0]         
        self.goal.pose.position.y = self.blackboard.goal[1]        

        # Publish goal position
        self.goal_publisher.publish(self.goal)
        rospy.loginfo("Goal position published")

    def update(self):

        if np.linalg.norm(self.current_pose[0:2] - np.array([self.goal.pose.position.x, self.goal.pose.position.y])) < 0.35:
            self.logger.debug("  %s [MoveRobot::Update() SUCCESS]" % self.name)
            return py_trees.common.Status.SUCCESS
        else:
            self.logger.debug("  %s [MoveRobot::Update() RUNNING]" % self.name)
            return py_trees.common.Status.RUNNING


    def terminate(self, new_status):
        self.logger.debug("  %s [MoveRobot::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))
        
    # Odometry callback: Gets current robot pose and stores it into self.current_pose
    def get_odom(self, odom):
        _, _, yaw = tf.transformations.euler_from_quaternion([odom.pose.pose.orientation.x, 
                                                            odom.pose.pose.orientation.y,
                                                            odom.pose.pose.orientation.z,
                                                            odom.pose.pose.orientation.w])
        self.current_pose = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y, yaw])

# Behavior for calling `set_point`
class SetPoint(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(SetPoint, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            "n_points", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(
            "n_points", access=py_trees.common.Access.READ)
        self.blackboard.register_key(
            "goal", access=py_trees.common.Access.WRITE)

    def setup(self):
        self.logger.debug("  %s [SetPoint::setup() SUCCESS]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [SetPoint::initialise()]" % self.name)

    def update(self):
        # Define point position the robot need to go to
        self.blackboard.goal = points_list[self.blackboard.n_points]   
        self.blackboard.n_points += 1        
        self.logger.debug("  %s [SetPoint::Update() SUCCESS]" % self.name)
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        self.logger.debug("  %s [SetPoint::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))

#   Create Behavior trees function
def create_tree():
    # Create Behaviors
    set_point = SetPoint(name="set_point")

    move_to_point = MoveRobot(name="move_to_point")

    check_object = CheckObject(name="check_object")

    get_object = GetObject(name="get_object")

    move_to_destination = MoveRobot(name="move_to_destination")

    let_object = LetObject(name="let_object")

    # Special py_trees behavior
    # Check number of points in the list the robot already went to
    n_points_lt_5 = py_trees.behaviours.CheckBlackboardVariableValue(
        name="n_points_lt_5",
        check=py_trees.common.ComparisonExpression(
            variable="n_points",
            value=len(points_list),
            operator=operator.lt
        )
    )

    # Check number of objects the robot already collected
    n_collected_objects_lt_2 = py_trees.behaviours.CheckBlackboardVariableValue(
        name="n_collected_objects_lt_2",
        check=py_trees.common.ComparisonExpression(
            variable="n_collected_objects",
            value=2,
            operator=operator.lt
        )
    )
    # Behavior checking end condition of the behavior tree
    check_end = py_trees.composites.Sequence(name="check_end", memory=True)
    check_end.add_children([n_collected_objects_lt_2, n_points_lt_5])

    root = py_trees.composites.Sequence(name="Life", memory=True)    
    root.add_children([check_end, set_point, move_to_point, check_object, get_object, move_to_destination, let_object])
    # py_trees.display.render_dot_tree(root)
    return root

def run(it=200):
    root = create_tree()

    try:
        print("Call setup for all tree children")
        root.setup_with_descendants() 
        print("Setup done!\n\n")
        py_trees.display.ascii_tree(root)
        
        for _ in range(it):
            root.tick_once()
            time.sleep(2)
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    py_trees.logging.level = py_trees.logging.Level.DEBUG
   
    rospy.init_node('behavior_trees')

    # Create behavior tree
    root = create_tree()
    run()

 
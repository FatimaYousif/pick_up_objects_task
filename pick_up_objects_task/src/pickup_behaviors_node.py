#!/usr/bin/env python

# roslaunch pick_up_objects_task pick_up_objects_task.launch

import py_trees
import rospy
from std_srvs.srv import Trigger, TriggerRequest

#------------ others
import py_trees.decorators
import py_trees.display

from py_trees.blackboard import Blackboard

import rospy
import tf
import numpy as np
import operator 
import time

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

#-----------------------

place_points = {"beer": np.array([-1.5,-1.5]),
                "coke": np.array([ 1.5, 1.5])}


pick_points = [np.array([1.25, 0.5]),
               np.array([1.25,-1.25]),
               np.array([ 0.0,-1.25]),
               np.array([-0.5, 1.25]),
               np.array([-1.25, 0.5])]

#------------------
                   
# Behavior for calling `check_object` task and if True, store object name to Blackboard
class CheckObject(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(CheckObject, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            "object_name", access=py_trees.common.Access.WRITE)

    # once object checked - its respective goal too    
   #------------------     
        self.blackboard.register_key(
            "goal", access=py_trees.common.Access.WRITE)
    #-------------------
                
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

                #-------------------
                self.goal = PoseStamped()
                self.blackboard.goal = place_points[self.blackboard.object_name]   
                #-------------------
                
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

        #-------------------        
        self.blackboard= self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            "object_name", access=py_trees.common.Access.WRITE
        )

        #everytime it lets the object on its desired position it ++ the objects_collected
        self.blackboard.register_key(
            "objects_collected", access=py_trees.common.Access.WRITE
        )
        #-------------------
                

    def setup(self):
        self.logger.debug("  %s [LetObject::setup()]" % self.name)
        #-------------------
        self.blackboard.objects_collected = 0
        #-------------------

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
        #-------------------
        self.blackboard.objects_collected +=1
        #-------------------


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

# TODO: Create any other required behavior like those to move the robot to a point, 
#       add or check elements in the blackboard, ...
        
# ---------------------------------------
class MoveRobot(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(MoveRobot, self).__init__(name)

        self.blackboard= self.attach_blackboard_client(name=self.name)

        #visited
        self.blackboard.register_key(
            "points_visited", access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            "goal", access=py_trees.common.Access.WRITE
        )

        self.current_pose = None        

    def setup(self):
        self.logger.debug("  %s [MoveRobot::setup()]" % self.name)

        #visited places
        self.blackboard.points_visited = 0

        # PUBLISHERS
        # send goal to controller node
        self.goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

        # SUBSCRIBERS
        # for odom
        self.odom_pub = rospy.Subscriber('/odom', Odometry, self.get_odom)
        
        time.sleep(0.2)  
        self.logger.debug(
                "  %s [MoveRobot::setup() SUCCESS]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [MoveRobot::initialise()]" % self.name)

        #goal pos
        self.goal =PoseStamped()
        self.goal.header.frame_id="map"
        self.goal.pose.position.x=self.blackboard.goal[0]
        self.goal.pose.position.y=self.blackboard.goal[1]
        
        self.goal_publisher.publish(self.goal)
        rospy.loginfo("Goal position published")


    def update(self):
        #threshold = 0.35
        if np.linalg.norm(self.current_pose[0:2] - np.array([self.goal.pose.position.x, self.goal.pose.position.y])) < 0.35:
            self.logger.debug("  %s [MoveRobot::Update() SUCCESS]" % self.name)
            return py_trees.common.Status.SUCCESS
        else:
            self.logger.debug("  %s [MoveRobot::Update() RUNNING]" % self.name)
            return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        self.logger.debug("  %s [MoveRobot::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))
    

    def get_odom(self, odom):
        _, _, yaw = tf.transformations.euler_from_quaternion([odom.pose.pose.orientation.x, 
                                                            odom.pose.pose.orientation.y,
                                                            odom.pose.pose.orientation.z,
                                                            odom.pose.pose.orientation.w])
        self.current_pose = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y, yaw])



# Behavior for calling `setting point (Pick and place BOTH)`
class SetPoint(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(SetPoint, self).__init__(name)
        self.blackboard= self.attach_blackboard_client(name = self.name)
        
        # read/write
        self.blackboard.register_key(
            "points_visited", access= py_trees.common.Access.WRITE)
        
        self.blackboard.register_key(
            "points_visited", access= py_trees.common.Access.READ)
        
        self.blackboard.register_key(
            "goal", access= py_trees.common.Access.WRITE)
    
    def setup(self):
        self.logger.debug("  %s [SetPoint::setup() SUCCESS]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [SetPoint::initialise()]" % self.name)


    def update(self):
        self.blackboard.goal = pick_points[self.blackboard.points_visited]
        self.blackboard.points_visited += 1
        self.logger.debug("  %s [SetPoint::Update() SUCCESS]" % self.name)
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        self.logger.debug("  %s [SetPoint::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))

def run(iterations=300):
    #behaviours
    set_point = SetPoint(name="set_point")
    move_to_pick = MoveRobot(name= "move_to_pick")
    check_object = CheckObject(name= "check_object")
    get_object = GetObject(name= "get_object")
    move_to_place= MoveRobot(name= "move_to_place")
    let_object = LetObject(name= "let_object")

    points_below_5 = py_trees.behaviours.CheckBlackboardVariableValue(
        name= "points_below_5",
        check = py_trees.common.ComparisonExpression(
            variable= "points_visited",
            value=len(pick_points),
            operator = operator.lt
        )
    )

    objects_below_2 = py_trees.behaviours.CheckBlackboardVariableValue(
        name= "objects_below_2",
        check = py_trees.common.ComparisonExpression(
            variable= "objects_collected",
            value=len(place_points),   #use dict len instead of 2
            operator = operator.lt
        )
    )

    loop_terminator = py_trees.composites.Sequence( name = "loop_terminator", memory=True)
    loop_terminator.add_children([objects_below_2, points_below_5])

    root = py_trees.composites.Sequence(name = "Loop", memory=True)
    root.add_children([loop_terminator, set_point, move_to_pick,check_object, get_object, move_to_place, let_object])
    

    try:
        root.setup_with_descendants() 
        py_trees.display.ascii_tree(root)
        
        for _ in range(iterations):
            root.tick_once()
            time.sleep(2)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    py_trees.logging.level = py_trees.logging.Level.DEBUG
    rospy.init_node("behavior_trees")

    #behaviours
    set_point = SetPoint(name="set_point")
    move_to_pick = MoveRobot(name= "move_to_pick")
    check_object = CheckObject(name= "check_object")
    get_object = GetObject(name= "get_object")
    move_to_place= MoveRobot(name= "move_to_place")
    let_object = LetObject(name= "let_object")

    points_below_5 = py_trees.behaviours.CheckBlackboardVariableValue(
        name= "points_below_5",
        check = py_trees.common.ComparisonExpression(
            variable= "points_visited",
            value=len(pick_points),
            operator = operator.lt
        )
    )

    objects_below_2 = py_trees.behaviours.CheckBlackboardVariableValue(
        name= "objects_below_2",
        check = py_trees.common.ComparisonExpression(
            variable= "objects_collected",
            value=len(place_points),   #use dict len instead of 2
            operator = operator.lt
        )
    )

    loop_terminator = py_trees.composites.Sequence( name = "loop_terminator", memory=True)
    loop_terminator.add_children([objects_below_2, points_below_5])

    root = py_trees.composites.Sequence(name = "Loop", memory=True)
    root.add_children([loop_terminator, set_point, move_to_pick,check_object, get_object, move_to_place, let_object])

    # printing for report purpose
    # py_trees.display.render_dot_tree(root) # generates a figure for the 'root' tree.    
    
    run()

 

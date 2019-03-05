import rospy
from plan_execution.msg import ExecutePlanAction
from smach import State
from smach_ros import SimpleActionState, ServiceState
from bwi_msgs.srv import RobotTeleporterInterface, RobotTeleporterInterfaceRequest, DoorHandlerInterface, DoorHandlerInterfaceRequest
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Point
from std_srvs.srv import Empty, EmptyRequest
from move_base_msgs.msg import MoveBaseAction

from plan_execution.helpers import *

topics = {'plan_execution': "/plan_executor/execute_plan",
        'move_base': "/move_base"}


class Wait(State):
    def __init__(self, amount):
        State.__init__(self, outcomes=["succeeded"])
        self.amount = amount

    def execute(self, ud):
        rospy.sleep(self.amount)
        return "succeeded"


class NoOp(Wait):
    def __init__(self):
        Wait.__init__(self, 0)


class ExecuteGoal(SimpleActionState):
    def __init__(self):
        SimpleActionState.__init__(self, topics["plan_execution"],
                                   ExecutePlanAction,
                                   goal_cb=self.goal_cb,
                                   result_cb=self.result_cb,
                                   input_keys=['goal'],
                                   output_keys=['result'])

    def goal_cb(self, userdata, goal):
        goal.aspGoal = userdata.goal.aspGoal

    def result_cb(self, userdata, state, result):
        print (state, result)
        userdata['result'] = result

    """
    def _goal_feedback_cb(self, feedback):
        rospy.logerr("GOT FEEDBACK :" + str(feedback))
        ACTION_START_TYPE = 2
        speech = None
        try:
            speech = villa_audio.tts.TextToSpeech(timeout=5.0)
        except RuntimeError:
            return

        if feedback.event_type == ACTION_START_TYPE:
            if feedback.plan[0].name == "navigate_to":
                speech.say("I'm starting to move.", wait=False)
            if feedback.plan[0].name == "perceive_surface":
                speech.say("I'm starting to scan.", wait=False)
    """

class TeleportRobot(ServiceState):
    def __init__(self, position=Point(15,107,0)):
        ServiceState.__init__(self, 
                            "teleport_robot", 
                            RobotTeleporterInterface,
                            request_cb = self.request_cb,
                            response_cb = self.response_cb)
        self.pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
        self.position = position

    def request_cb(self, userdata, request):
        teleport_req = RobotTeleporterInterfaceRequest()
        
        teleport_req.pose.position = self.position
        teleport_req.pose.orientation.x = 0
        teleport_req.pose.orientation.y = 0
        teleport_req.pose.orientation.z = 0
        teleport_req.pose.orientation.w = 1

        return teleport_req

    def response_cb(self, userdata, response):

        pose = PoseWithCovarianceStamped()
        #pose.header.stamp = rospy.get_rostime()
        pose.header.frame_id = "/level_mux_map"
        pose.pose.pose.position = self.position
        pose.pose.pose.orientation.z = 0.0
        pose.pose.pose.orientation.w = 1.0
        pose.pose.covariance[0] = 0.1
        pose.pose.covariance[7] = 0.1
        pose.pose.covariance[35] = 0.25

        self.pub.publish(pose)
        rospy.sleep(1)
        self.pub.publish(pose)
        rospy.sleep(1)
        self.pub.publish(pose)
        rospy.sleep(1)
        self.pub.publish(pose)
        rospy.sleep(1)

class MoveToPosition(SimpleActionState):
    #def __init__(self, position=Point(15, 107, 0)):
    def __init__(self, position=Point(-13.8, -7.15, 0)):
        SimpleActionState.__init__(self, topics["move_base"],
                                   MoveBaseAction,
                                   goal_cb=self.goal_cb,
                                   result_cb=self.result_cb,
                                   input_keys=['goal'],
                                   output_keys=['result'])
        self.position = position

    def goal_cb(self, userdata, goal):
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = "/level_mux_map"
        goal.target_pose.pose.position = self.position
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = 0.0
        goal.target_pose.pose.orientation.w = 1.0


    def result_cb(self, userdata, state, result):
        print (state, result)

class ClearCostmap(ServiceState):
    def __init__(self):
        ServiceState.__init__(self, 
                            "/move_base/clear_costmaps", 
                            Empty,
                            request = EmptyRequest(),
                            response_cb = self.response_cb)

    def response_cb(self, userdata, response):
        rospy.sleep(1)

class CloseDoor(ServiceState):
    def __init__(self, door = None):
        ServiceState.__init__(self, 
                            "/update_doors", 
                            DoorHandlerInterface,
                            request_cb = self.request_cb,
                            response_cb = self.response_cb)
        self.door = door

    def request_cb(self, userdata, request):
        if not self.door:
            return DoorHandlerInterfaceRequest(door="", open=False, all_doors=True, open_timeout=0.0)
        else:
            return DoorHandlerInterfaceRequest(door=self.door, open=False, all_doors=False, open_timeout=0.0)

    def response_cb(self, userdata, response):
        rospy.sleep(1)

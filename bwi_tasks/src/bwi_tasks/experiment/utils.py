import roslaunch
import rospy
import sys
import rospkg

from smach import State

launch = roslaunch.parent.ROSLaunchParent(None, [None])

class LaunchPlanExecutor(State):
    def __init__(self, simulation, use_learning=True, use_motion_cost=True):
        State.__init__(self, outcomes=["succeeded"])
        self.simulation = "true" if simulation else "false"
        self.use_learning = "true" if use_learning else "false"
        self.use_motion_cost = "true" if use_motion_cost else "false"

    def execute(self, ud):
    	uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    	roslaunch.configure_logging(uuid)
    	rospack = rospkg.RosPack()
    	path = rospack.get_path('bwi_kr_execution') + "/launch/bwi_kr_execution.launch"
    	launch = roslaunch.parent.ROSLaunchParent(uuid, [path])

    	sys.argv.append("simulation:="+self.simulation)
    	sys.argv.append("use_learning:="+self.use_learning)
    	sys.argv.append("use_motion_cost:="+self.use_motion_cost)
    	sys.argv.append("--screen")

    	launch.start()

    	rospy.sleep(2)

    	return "succeeded"

class ShutdownPlanExecutor(State):
    def __init__(self):
        State.__init__(self, outcomes=["succeeded"])

    def execute(self, ud):
    	launch.shutdown()
    	return "succeeded"

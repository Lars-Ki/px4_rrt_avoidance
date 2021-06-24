import rospy
import mavros
from mavros_msgs.msg import OverrideRCIn, ParamValue, AttitudeTarget
from mavros_msgs.srv import SetMode, CommandBool, ParamPull, ParamPush, ParamGet, ParamSet

class ParameterViewer:
    def __init__(self):
        print("waiting for Service...")
        service_timeout = 180 #3min
        rospy.wait_for_service('mavros/param/get', service_timeout)
        print("finished!")

        get_param_topic  = 'mavros/param/get'
        self.get_param_srv  = rospy.ServiceProxy(get_param_topic, ParamGet)

    def getParam(self, param_name):
        answ = self.get_param_srv(param_name)
        if answ.success:
            if answ.value.integer != 0:
                value = answ.value.integer
            else:
                value = answ.value.real
        else:
            rospy.logwarn("Parameter "+param_name+" not read") 
            value = None
        return value

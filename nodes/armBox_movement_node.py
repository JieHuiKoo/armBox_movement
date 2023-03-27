#!/usr/bin/python3


import sys
print("Python version")
print (sys.version)
print("Version info.")
print (sys.version_info)


from gpiozero import AngularServo
import time

import rospy
import actionlib
import armBox_movement.msg

left_arm = AngularServo(17, min_angle=-180, max_angle=180)
right_arm = AngularServo(27, min_angle=-180, max_angle=180)
trapdoor = AngularServo(22, min_angle=-180, max_angle=180)

def openArm(duration):
    # Move Open
    left_arm.angle = int(180)
    right_arm.angle = int(180)
        
    time.sleep(duration)
    
    # Move Stop
    left_arm.angle = None
    right_arm.angle = None
    
def closeArm(duration):
    # Move Close
    left_arm.angle = int(-180)
    right_arm.angle = int(-180)
        
    time.sleep(duration)
    
    # Move Stop
    left_arm.angle = None
    right_arm.angle = None
    
def openTrapdoor(duration):
    # Move Open
    trapdoor.angle = int(180)
        
    time.sleep(duration)
    
    # Move Stop
    trapdoor.angle = None
    
def closeTrapdoor(duration):
    # Move Close
    trapdoor.angle = int(-180)
        
    time.sleep(duration)
    
    # Move Stop
    trapdoor.angle = None

class ArmBoxMovementAction(object):
    # create messages that are used to publish feedback/result
    _result = armBox_movement.msg.ArmBoxMovementResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, armBox_movement.msg.ArmBoxMovementAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
      
    def execute_cb(self, goal):
        # helper variables
        r = rospy.Rate(1)
        success = True
        
        # If Arm
        if goal.armbox_enum == 1:
            if goal.open == 1:
                rospy.loginfo('[%s]: Executing Arm Open', self._action_name)
                openArm(3)
                
            elif goal.open == 0:
                rospy.loginfo('[%s]: Executing Arm Close', self._action_name)
                closeArm(3)
                
        # If Trapdoor
        elif goal.armbox_enum == 0:
            if goal.open == 1:
                rospy.loginfo('[%s]: Executing Trapdoor Open', self._action_name)
                openTrapdoor(3)

            elif goal.open == 0:
                rospy.loginfo('[%s]: Executing Trapdoor Close', self._action_name)
                closeTrapdoor(3)

                  
        if success:
            rospy.loginfo('[%s]: Executed' % self._action_name)
            self._as.set_succeeded(self._result)
        
if __name__ == '__main__':
    rospy.init_node('ArmBoxMovement')
    server = ArmBoxMovementAction(rospy.get_name())
    rospy.spin()




































# 
# 
# 

# 
# def openArm(duration):
#     # Move Open
#     s0.angle = int(180)
#     s1.angle = int(360)
#     
#     print("S0: Open")
#     
#     time.sleep(duration)
#     
#     # Move Stop
#     s0.angle = int(-5)
#     s1.angle = int(0)
#     print("S0: Stop")
#     print("S1: Stop")
# 
#     
# def closeArm():
#     # Move Close
#     s0.angle = int(-360)
#     s1.angle = int(-360)
#     
#     # Amount to move the servo by
#     time.sleep(duration)
# 
#     # Move Stop
#     s0.angle = int(0)
#     s1.angle = int(0)
#     
# def openTrapdoor():
#     # Move Open
#     s2.angle = int(360)
#     
#     # Amount to move the servo by
#     time.sleep(duration)
#     
#     # Move Stop
#     s2.angle = int(0)
#     
# def closeTrapdoor():
#     # Move Close
#     s2.angle = int(-360)
#     
#     # Amount to move the servo by
#     time.sleep(duration)
#     
#     # Move Stop
#     s2.angle = int(0)
#     
# openArm(3)
#!/usr/bin/env python

import rospy
import smach
import smach_ros
import time
from pyrobot import Robot

ur5_arm = 'ur5'
vx300s_arm = 'vx300s'

# define state GoHome
class Init(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['init_all'])
        global vx300s_arm
        global ur5_arm
        vx300s_arm = Robot("vx300s", use_arm=True, use_base=False, use_camera=False, use_gripper=False)
        ur5_arm = Robot("ur5", use_arm=True, use_base=False, use_camera=False, use_gripper=False)

    def execute(self, userdata):
        rospy.loginfo('Executing state Init')

        return 'init_all'

class Perception_obj(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['have_objects','failure'])

    def execute(self, userdata):

        rospy.loginfo('Perception_obj')

        return 'have_objects'

# define state Move
class Move_vx300s_grasp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success','failure'])

    def execute(self, userdata):
        target_joint = [0, -1.7, -1.6, 0.0015, -1.110, -0.006]
        global vx300s_arm

        rospy.loginfo('Move_vx300s_grasp')
        vx300s_arm.arm.set_joint_positions(target_joint, plan=True)
        time.sleep(1)
        vx300s_arm.arm.go_home()

#        vx300s_arm.close_gripper()
        time.sleep(1)

        return 'success'
        return 'failure'

# define state Move
class Move_vx300s_hold(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['movement'])

    def execute(self, userdata):
        target_joint_1 = [0, 0, 0.6, -0.4, 0]

        rospy.loginfo('Move_vx300s_hold')
#        vx300s_arm.arm.set_joint_positions(target_joint_1, plan=True)
        time.sleep(1)

        return 'movement'

class Move_ur5_grasp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success','failure'])
        self.counter = 0

    def execute(self, userdata):
        joint = [-0.03938514391054326, -1.3492568174945276, 1.8922514915466309, -2.668490235005514, -1.6633527914630335, 0.23534096777439117]

        rospy.loginfo('Move_ur5_grasp')
        ur5_arm.arm.set_joint_positions(joint, plan=True)
        time.sleep(1)

#        vx300s_arm.open_gripper()
        time.sleep(1)

        return 'success'
        return 'failure'

class Move_ur5_hold(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['movement','back'])
        self.counter = 0

    def execute(self, userdata):
        joint = [-0.008826557789937794, -1.526635471974508, 1.9602179527282715, -2.5408323446856897, -1.6822107473956507, 0.25439926981925964]

        rospy.loginfo('Move_ur5_hold')
        ur5_arm.arm.set_joint_positions(joint, plan=True)
        time.sleep(1)

#        vx300s_arm.open_gripper()
        time.sleep(1)

        if self.counter < 2:
            self.counter += 1
            return 'back'
        else:
            return 'movement'

# main
def main():
#    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['END'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Init', Init(), transitions={'init_all':'Move_vx300s_grasp'})
        smach.StateMachine.add('Move_vx300s_grasp', Move_vx300s_grasp(), transitions={'success':'Move_vx300s_hold','failure':'Move_vx300s_grasp'})
        smach.StateMachine.add('Move_vx300s_hold', Move_vx300s_hold(), transitions={'movement':'Move_ur5_grasp'})
        smach.StateMachine.add('Move_ur5_grasp', Move_ur5_grasp(), transitions={'success':'Move_ur5_hold','failure':'Move_ur5_grasp'})
        smach.StateMachine.add('Move_ur5_hold', Move_ur5_hold(), transitions={'back':'Init','movement':'END'})

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('my_smach_introspection_server', sm, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()

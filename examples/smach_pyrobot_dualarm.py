#!/usr/bin/env python

import rospy
import smach
import smach_ros
import time
from pyrobot import Robot

ur5_arm = 'ur5'
vx300s_arm = 'vx300s'

# define state Init


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

# define state go_home
class Move_go_home(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['movement'])

    def execute(self, userdata):
        rospy.loginfo('Move_go_home')
        time.sleep(1)

        return 'movement'


class Perception_obj(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=['have_object_bn_0', 'have_object_bn_1', 'have_object', 'empty'])

    def execute(self, userdata):
        rospy.loginfo('Perception_obj')

        if have_object_bn :
            return 'have_object_bn_0'
        elif have_object_bn :
            return 'have_object_bn_1'
        elif have_object :
            return 'have_object'
        else:
            return 'empty'

# define state Move

class Move_vx300s_pick(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failure'])

    def execute(self, userdata):
        target_joint = [0, -1.7, -1.6, 0.0015, -1.110, -0.006]
        global vx300s_arm

        rospy.loginfo('Move_vx300s_pick')
        vx300s_arm.arm.set_joint_positions(target_joint, plan=True)
        time.sleep(1)

#        vx300s_arm.close_gripper()
        time.sleep(1)

        if if_grasp.success :
            return 'success'
        else:
            return 'failure'


class Move_ur5_pick(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.counter = True

    def execute(self, userdata):
        joint = [-0.03938514391054326, -1.3492568174945276, 1.8922514915466309, - \
            2.668490235005514, -1.6633527914630335, 0.23534096777439117]

        rospy.loginfo('Move_ur5_pick')
        ur5_arm.arm.set_joint_positions(joint, plan=True)
        time.sleep(1)

#        robotiq.close_gripper()
        time.sleep(1)

        if if_grasp.success :
            return 'success'
        else:
            return 'failure'


class Move_regrasping(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['movement'])

    def execute(self, userdata):
        rospy.loginfo('Move_regrasping')
        time.sleep(1)

        return 'movement'


class Move_placing(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['movement'])

    def execute(self, userdata):
        rospy.loginfo('Move_placing')
        time.sleep(1)

        return 'movement'

# main

def main():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['END'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Init', Init(), transitions={'init_all': 'Move_go_home'})
        smach.StateMachine.add('Move_go_home', Move_go_home(), transitions={'movement': 'Perception_obj'})
        smach.StateMachine.add('Perception_obj', Perception_obj(), transitions={'empty': 'END', 'have_object':'Move_regrasping', 'have_object_bn_0':'Move_ur5_pick', 'have_object_bn_1':'Move_vx300s_pick'})
        smach.StateMachine.add('Move_vx300s_pick', Move_vx300s_pick(), transitions={'success': 'Move_placing', 'failure':'Move_vx300s_pick'})
        smach.StateMachine.add('Move_ur5_pick', Move_ur5_pick(), transitions={'success': 'Move_placing', 'failure':'Move_ur5_pick'})
        smach.StateMachine.add('Move_regrasping', Move_regrasping(), transitions={'movement': 'Perception_obj'})
        smach.StateMachine.add('Move_placing', Move_placing(), transitions={'movement': 'Move_go_home'})

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer(
        'my_smach_introspection_server', sm, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()

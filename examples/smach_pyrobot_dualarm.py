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


class BN_detection(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=['have_bn', 'empty'])

    def execute(self, userdata):
        rospy.loginfo('BN_detection')

        if have_bn :
            return 'have_bn'
        else:
            return 'empty'

class Pose_estimate_BN(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=['have_object_pose_with_BN', 'empty'])

    def execute(self, userdata):
        rospy.loginfo('Pose_estimate_BN')

        if have_object_pose_with_BN:
            return 'have_object_pose_with_BN'
        else:
            return 'empty'

class Pose_estimate_object(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=['have_object_pose', 'empty'])

    def execute(self, userdata):
        rospy.loginfo('Pose_estimate_object')

        if have_object_pose:
            return 'have_object_pose'
        else:
            return 'empty'

# define state Move
class Move_vx300s_pick_to_tote(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failure'])

    def execute(self, userdata):
        target_joint = [0, -1.7, -1.6, 0.0015, -1.110, -0.006]
        global vx300s_arm

        rospy.loginfo('Move_vx300s_pick_to_tote')
        vx300s_arm.arm.set_joint_positions(target_joint, plan=True)
        time.sleep(1)

        #vx300s_arm.gripper.close()
        time.sleep(1)

        if success :
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

class OnHand_BN_detection(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=['have_bn', 'empty'])

    def execute(self, userdata):
        rospy.loginfo('OnHand_BN_detection')

        if have_bn :
            return 'have_bn'
        else:
            return 'empty'

class OnHand_Pose_estimate_BN(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=['have_object_pose_with_BN', 'empty'])

    def execute(self, userdata):
        rospy.loginfo('OnHand_Pose_estimate_BN')

        if have_object_pose_with_BN:
            return 'have_object_pose_with_BN'
        else:
            return 'empty'

class OnHand_Pose_estimate_object(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=['have_object_pose', 'empty'])

    def execute(self, userdata):
        rospy.loginfo('OnHand_Pose_estimate_object')

        if have_object_pose:
            return 'have_object_pose'
        else:
            return 'empty'

class Move_ur5_regrasping(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['movement'])

    def execute(self, userdata):
        rospy.loginfo('Move_ur5_regrasping')
        time.sleep(1)

        return 'movement'


class Move_ur5_placing(smach.State):
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
        smach.StateMachine.add('Init', Init(), transitions={'init_all':'Move_go_home'})
        smach.StateMachine.add('Move_go_home', Move_go_home(), transitions={'movement':'BN_detection'})

        smach.StateMachine.add('BN_detection', BN_detection(), transitions={'empty':'Pose_estimate_object', 'have_bn':'Pose_estimate_BN'})
        smach.StateMachine.add('Pose_estimate_BN', Pose_estimate_BN(), transitions={'empty':'END', 'have_object_pose_with_BN':'Move_vx300s_pick_to_tote'})
        smach.StateMachine.add('Pose_estimate_object', Pose_estimate_object(), transitions={'empty':'END', 'have_object_pose':'Move_vx300s_pick_to_tote'})

        smach.StateMachine.add('Move_vx300s_pick_to_tote', Move_vx300s_pick_to_tote(), transitions={'success':'OnHand_BN_detection', 'failure':'BN_detection'})

        smach.StateMachine.add('OnHand_BN_detection', OnHand_BN_detection(), transitions={'empty':'OnHand_Pose_estimate_object', 'have_bn':'OnHand_Pose_estimate_BN'})
        smach.StateMachine.add('OnHand_Pose_estimate_BN', OnHand_Pose_estimate_BN(), transitions={'empty':'END', 'have_object_pose_with_BN':'Move_ur5_pick'})
        smach.StateMachine.add('OnHand_Pose_estimate_object', OnHand_Pose_estimate_object(), transitions={'empty':'END', 'have_object_pose':'Move_ur5_regrasping'})

        smach.StateMachine.add('Move_ur5_pick', Move_ur5_pick(), transitions={'success':'Move_ur5_placing', 'failure':'OnHand_BN_detection'})
        smach.StateMachine.add('Move_ur5_regrasping', Move_ur5_regrasping(), transitions={'movement':'OnHand_BN_detection'})
        smach.StateMachine.add('Move_ur5_placing', Move_ur5_placing(), transitions={'movement':'Move_go_home'})

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

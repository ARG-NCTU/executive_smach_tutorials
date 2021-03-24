#!/usr/bin/env python

import rospy
import smach
import smach_ros
import time
from pyrobot import Robot

bot = Robot( "ur5", use_arm=True, use_base=False, use_camera=False, use_gripper=False)
loco_bot = Robot("locobot", use_base=False)

time.sleep(1)

# define state GoHome
class Init(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Init')
        time.sleep(1)
        return 'outcome1'

# define state Move
class Move_locobot_grasp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome2'])

    def execute(self, userdata):
        target_joint = [0, 0.3, 0, -0.4, 0]

        rospy.loginfo('Move_locobot_grasp')
        loco_bot.arm.set_joint_positions(target_joint, plan=False)
        time.sleep(1)

        loco_bot.gripper.close()
        time.sleep(1)

        return 'outcome2'

# define state Move
class Move_locobot_hold(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome3'])

    def execute(self, userdata):
        target_joint_1 = [0, 0, 0.6, -0.4, 0]

        rospy.loginfo('Move_locobot_hold')
        loco_bot.arm.set_joint_positions(target_joint_1, plan=False)
        time.sleep(1)

        return 'outcome3'

class Move_ur5_grasp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome4'])
        self.counter = 0

    def execute(self, userdata):
        joint = [-0.03938514391054326, -1.3492568174945276, 1.8922514915466309, -2.668490235005514, -1.6633527914630335, 0.23534096777439117]

        rospy.loginfo('Move_ur5_grasp')
        bot.arm.set_joint_positions(joint, plan=True)
        time.sleep(1)

        loco_bot.gripper.open()
        time.sleep(1)

        return 'outcome4'

class Move_ur5_hold(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome5','back'])
        self.counter = 0

    def execute(self, userdata):
        joint = [-0.008826557789937794, -1.526635471974508, 1.9602179527282715, -2.5408323446856897, -1.6822107473956507, 0.25439926981925964]

        rospy.loginfo('Move_ur5_hold')
        bot.arm.set_joint_positions(joint, plan=True)
        time.sleep(1)

        loco_bot.gripper.open()
        time.sleep(1)

        if self.counter < 2:
            self.counter += 1
            return 'back'
        else:
            return 'outcome5'

# main
def main():
    #rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['END'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Init', Init(), transitions={'outcome1':'Move_locobot_grasp'})
        smach.StateMachine.add('Move_locobot_grasp', Move_locobot_grasp(), transitions={'outcome2':'Move_locobot_hold'})
        smach.StateMachine.add('Move_locobot_hold', Move_locobot_hold(), transitions={'outcome3':'Move_ur5_grasp'})
        smach.StateMachine.add('Move_ur5_grasp', Move_ur5_grasp(), transitions={'outcome4':'Move_ur5_hold'})
        smach.StateMachine.add('Move_ur5_hold', Move_ur5_hold(), transitions={'back':'Init','outcome5':'END'})

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

#!/usr/bin/env python

import rospy
import smach
import smach_ros
import time
from pyrobot import Robot

bot = Robot( "ur5", use_arm=True, use_base=False, use_camera=False, use_gripper=False )
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
class Move_pick(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome2'])

    def execute(self, userdata):
        target_joints = [[0.0008875816711224616, -1.3548930327044886, 1.9611186981201172, -2.1576502958880823, -1.543307129536764, -0.019394699727193654],
        [0.011333332397043705, -1.501077953969137, 1.3318209648132324, -1.3776496092425745, -1.5517552534686487, -0.02223378816713506]]
        rospy.loginfo('Moving')
        for joint in target_joints:
                bot.arm.set_joint_positions(joint, plan=True)
                time.sleep(1)
        return 'outcome2'

# define state Move
class Move_place(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome3'])

    def execute(self, userdata):
        target_joints = [[-0.673718277608053, -1.3569310347186487, 1.1818432807922363, -1.3697541395770472, -1.5765941778766077, -0.7019279638873499],
        [-0.6554644743548792, -1.2213314215289515, 1.7868561744689941, -2.1037758032428187, -1.5695608297931116, -0.7016523520099085]]
        rospy.loginfo('Moving')
        for joint in target_joints:
                bot.arm.set_joint_positions(joint, plan=True)
                time.sleep(1)
        return 'outcome3'

class GoHome(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome4','back'])
        self.counter = 0

    def execute(self, userdata):
        target_joints = [[-0.673718277608053, -1.3569310347186487, 1.1818432807922363, -1.3697541395770472, -1.5765941778766077, -0.7019279638873499],
        [0.011333332397043705, -1.501077953969137, 1.3318209648132324, -1.3776496092425745, -1.5517552534686487, -0.02223378816713506]]
        rospy.loginfo('Moving')
        for joint in target_joints:
                bot.arm.set_joint_positions(joint, plan=True)
                time.sleep(1)
        if self.counter < 2:
            self.counter += 1
            return 'back'
        else:
            return 'outcome4'

# main
def main():
    #rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['END'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Init', Init(), transitions={'outcome1':'Move_pick'})
        smach.StateMachine.add('Move_pick', Move_pick(), transitions={'outcome2':'Move_place'})
        smach.StateMachine.add('Move_place', Move_place(), transitions={'outcome3':'GoHome'})
        smach.StateMachine.add('GoHome', GoHome(), transitions={'back':'Init','outcome4':'END'})

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

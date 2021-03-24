#!/usr/bin/env python

import rospy
import smach
import smach_ros
import time
from pyrobot import Robot

base_config_dict={'base_controller': 'ilqr'}
bot = Robot('locobot', base_config=base_config_dict)

# define state GoHome
class GoHome(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0

    def execute(self, userdata):
        target_position = [0,0,0]
        rospy.loginfo('Executing state GoHome')
        #bot.arm.go_home()
        bot.base.go_to_absolute(target_position)
        if self.counter < 3:
            self.counter += 1
            return 'outcome1'
        else:
            return 'outcome2'

# define state Move
class Move_straight(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome2'])

    def execute(self, userdata):
        target_position = [0.3,0,0]
        target_joints = [[0.408, 0.721, -0.471, -1.4, 0.920], [-0.675, 0, 0.23, 1, -0.70]]
        rospy.loginfo('Moving')
        for joint in target_joints:
                #bot.arm.set_joint_positions(joint, plan=False)
                bot.base.go_to_absolute(target_position)
                time.sleep(1)
        return 'outcome2'

# define state Move
class Move_right(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome5'])

    def execute(self, userdata):
        target_position = [0.3,-0.3,-1.57]
        target_joints = [[0.408, 0.721, -0.471, -1.4, 0.920], [-0.675, 0, 0.23, 1, -0.70]]
        rospy.loginfo('Moving')
        for joint in target_joints:
                #bot.arm.set_joint_positions(joint, plan=False)
                bot.base.go_to_absolute(target_position)
                time.sleep(1)
        return 'outcome5'

class Move_left(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome6'])

    def execute(self, userdata):
        target_position = [0.3,0,0]
        target_joints = [[0.408, 0.721, -0.471, -1.4, 0.920], [-0.675, 0, 0.23, 1, -0.70]]
        rospy.loginfo('Moving')
        for joint in target_joints:
                #bot.arm.set_joint_positions(joint, plan=False)
                bot.base.go_to_absolute(target_position)
                time.sleep(1)
        return 'outcome6'

# main
def main():
    #rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome3', 'outcome4'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('GoHome', GoHome(), transitions={'outcome1':'Move_straight', 'outcome2':'outcome3'})
        smach.StateMachine.add('Move_straight', Move_straight(), transitions={'outcome2':'Move_right'})
        smach.StateMachine.add('Move_right', Move_right(), transitions={'outcome5':'Move_left'})
        smach.StateMachine.add('Move_left', Move_left(), transitions={'outcome6':'GoHome'})

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

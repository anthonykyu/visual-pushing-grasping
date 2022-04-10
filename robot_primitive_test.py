from robot import Robot
import numpy as np
input = ("Which test:")
if input == 1:
    robot = Robot(False,False,False,False,False,False,False,False,False,False,False)
elif input == 2:
    robot = Robot(False,False,False,False,False,False,False,False,False,False,False)
    joints = robot.get_state['joints'] + [0.1,0.1,0.1,0.1,0.1,0.1,0.1]
    robot.move_joints(joints)
    robot.close_gripper()
    robot.open_gripper()
    robot.go_home()
elif input == 3:
    robot = Robot(False,False,False,False,False,False,False,False,False,False,False)
    state = robot.get_state()
    pose = robot.parse_state_data(state,'pose') + [0.05,0.05,0.05,0,0,0]
    robot.move_to(pose[0:3],pose[3:])
    robot.close_gripper()
    robot.open_gripper()
    robot.go_home()
    pass
elif input == 4:
    pass
elif input == 5:
    pass
elif input == 6:
    pass

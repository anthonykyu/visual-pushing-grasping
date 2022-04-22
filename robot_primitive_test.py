from franka import Franka
import numpy as np
workspace_limits = np.asarray([[0.14273662+0.05, 0.658929158-0.05], [-0.37338492+0.05, 0.37420559-0.05], [0.01125959, 0.75]]) # Cols: min max, Rows: x y z (define workspace limits in robot coordinates)
input = input("Which test:")
if input == "1":
    robot = Franka(False,workspace_limits)
elif input == "2":
    robot = Franka(False,workspace_limits)
    joints = robot.get_state()['joints']
    print(joints)
    # robot.move_joints(joints)
    # robot.close_gripper()
    # robot.open_gripper()
    # robot.go_home()
elif input == "3":
    robot = Franka(False,workspace_limits)
    state = robot.get_state()
    pose = robot.parse_state_data(state,'pose')
    robot.close_gripper()
    while True:
        pose = pose + [0.0,0.0,0.05,0,0,0]
        robot.move_to(pose[0:3],pose[3:])

        # robot.open_gripper()
        # robot.go_home()
elif input == "4":
    # [ 0.71329748 -0.34039687  0.01400886  3.11800001 -0.11070617  0.04244794]
    # [ 0.12142294  0.37248311  0.68205768  3.12858163 -0.05685351 -0.00963619]

    # x max = 6.58929158e-01
    # x min = 0.14273662
    # y max = 0.37420559
    # y min = -0.37338492
    # z min = 0.01125959
    workspace_limits = np.asarray([[0.14273662+0.05, 0.658929158-0.05], [-0.37338492+0.05, 0.37420559-0.05], [0.01125959, 0.75]]) # Cols: min max, Rows: x y z (define workspace limits in robot coordinates)

    robot = Franka(False,workspace_limits)
    state = robot.get_state()
    pose = robot.parse_state_data(state,'pose')
    print(pose)
elif input == "5":
    workspace_limits = np.asarray([[0.14273662+0.05, 0.658929158-0.05], [-0.37338492+0.05, 0.37420559-0.05], [0.01125959, 0.75]]) # Cols: min max, Rows: x y z (define workspace limits in robot coordinates)
    robot = Franka(False,workspace_limits)
    robot.grasp([0.5, 0.0, 0.05], -np.pi, workspace_limits)
elif input == "6":
    workspace_limits = np.asarray([[0.14273662+0.05, 0.658929158-0.05], [-0.37338492+0.05, 0.37420559-0.05], [0.01125959, 0.75]]) # Cols: min max, Rows: x y z (define workspace limits in robot coordinates)
    robot = Franka(False,workspace_limits)
    robot.push([0.5, 0, 0.2], np.pi, workspace_limits)
elif input == "7":
    workspace_limits = np.asarray([[0.14273662+0.05, 0.658929158-0.05], [-0.37338492+0.05, 0.37420559-0.05], [0.01125959, 0.75]]) # Cols: min max, Rows: x y z (define workspace limits in robot coordinates)
    robot = Franka(workspace_limits)
    robot.restart_real()


# azure_kinect_overhead
# world
# 0.724615 0.007335 0.863543
# 0.001284 0.999785 -0.020679
# 0.999944 -0.001480 -0.009480
# -0.009509 -0.020670 -0.999732
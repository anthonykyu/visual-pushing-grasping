import socket
import select
import struct
import time
import os
import numpy as np
from yaml import dump
import utils
from simulation import vrep
from frankapy import FrankaArm
from autolab_core import RigidTransform

class Franka(object):
    def __init__(self, workspace_limits, is_sim=False):

        self.fa = FrankaArm()

        self.workspace_limits = workspace_limits

        # Move robot to home pose
        # print("Closing Gripper")
        # self.close_gripper()
        print("Going Home")
        self.go_home()
        print("Opening Gripper")
        self.open_gripper()

        # Fetch RGB-D data from RealSense camera
        from real.camera import Camera
        self.camera = Camera()
        self.cam_intrinsics = self.camera.intrinsics
        print("Check")
        # Load camera pose (from running calibrate.py), intrinsics and depth scale
        self.cam_pose = np.loadtxt('real/camera_pose.txt', delimiter=' ')
        self.cam_depth_scale = np.loadtxt('real/camera_depth_scale.txt', delimiter=' ')


    def get_task_score(self):

        key_positions = np.asarray([[-0.625, 0.125, 0.0], # red
                                    [-0.625, -0.125, 0.0], # blue
                                    [-0.375, 0.125, 0.0], # green
                                    [-0.375, -0.125, 0.0]]) #yellow

        obj_positions = np.asarray(self.get_obj_positions())
        obj_positions.shape = (1, obj_positions.shape[0], obj_positions.shape[1])
        obj_positions = np.tile(obj_positions, (key_positions.shape[0], 1, 1))

        key_positions.shape = (key_positions.shape[0], 1, key_positions.shape[1])
        key_positions = np.tile(key_positions, (1 ,obj_positions.shape[1] ,1))

        key_dist = np.sqrt(np.sum(np.power(obj_positions - key_positions, 2), axis=2))
        key_nn_idx = np.argmin(key_dist, axis=0)

        return np.sum(key_nn_idx == np.asarray(range(self.num_obj)) % 4)


    def check_goal_reached(self):
        # Compute tool orientation from heightmap rotation angle
        # grasp_orientation = [1.0,0.0]
        # tool_rotation_angle = -np.pi/4
        # tool_orientation = np.asarray([grasp_orientation[0]*np.cos(tool_rotation_angle) - grasp_orientation[1]*np.sin(tool_rotation_angle), grasp_orientation[0]*np.sin(tool_rotation_angle) + grasp_orientation[1]*np.cos(tool_rotation_angle), 0.0])*np.pi
        # tool_orientation_angle = np.linalg.norm(tool_orientation)
        # tool_orientation_axis = tool_orientation/tool_orientation_angle
        # tool_orientation_rotm = utils.angle2rotm(tool_orientation_angle, tool_orientation_axis, point=None)[:3,:3]

        # tilt_rotm = utils.euler2rotm(np.asarray([-np.pi/4,0,0]))
        # tilted_tool_orientation_rotm = np.dot(tilt_rotm, tool_orientation_rotm)
        # tilted_tool_orientation_axis_angle = utils.rotm2angle(tilted_tool_orientation_rotm)
        # tilted_tool_orientation = tilted_tool_orientation_axis_angle[0]*np.asarray(tilted_tool_orientation_axis_angle[1:4])

        # Default tool position (rotated to grab box in back)
        # tool_orientation = np.asarray([np.pi, 0, np.pi/2])

        # Move to box grabbing position
        # box_grab_position = [0.5,-0.35,-0.12]
        # bin_position = [0.3,0.2,0.2]

        # go to cartesian end point -  need to implement

        # self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # self.tcp_socket.connect((self.tcp_host_ip, self.tcp_port))
        # tcp_command = "def process():\n"
        # tcp_command += " set_digital_out(8,False)\n"
        # tcp_command += " movej(p[%f,%f,%f,%f,%f,%f],a=%f,v=%f,t=0,r=0.09)\n" % (box_grab_position[0],box_grab_position[1],box_grab_position[2]+0.1,tilted_tool_orientation[0],tilted_tool_orientation[1],tilted_tool_orientation[2],self.joint_acc,self.joint_vel)
        # tcp_command += " movej(p[%f,%f,%f,%f,%f,%f],a=%f,v=%f,t=0,r=0.00)\n" % (box_grab_position[0],box_grab_position[1],box_grab_position[2],tool_orientation[0],tool_orientation[1],tool_orientation[2],self.joint_acc,self.joint_vel)
        # tcp_command += " set_digital_out(8,True)\n"
        # tcp_command += "end\n"
        # self.tcp_socket.send(str.encode(tcp_command))
        # self.tcp_socket.close()



        # Block until robot reaches box grabbing position and gripper fingers have stopped moving
        # state_data = self.get_state()
        # tool_analog_input2 = self.parse_tcp_state_data(state_data, 'tool_data')
        # while True:
        #     state_data = self.get_state()
        #     new_tool_analog_input2 = self.parse_tcp_state_data(state_data, 'tool_data')
        #     actual_tool_pose = self.parse_tcp_state_data(state_data, 'cartesian_info')
        #     if tool_analog_input2 < 3.7 and (abs(new_tool_analog_input2 - tool_analog_input2) < 0.01) and all([np.abs(actual_tool_pose[j] - box_grab_position[j]) < self.tool_pose_tolerance[j] for j in range(3)]):
        #         break
        #     tool_analog_input2 = new_tool_analog_input2

        # Move to box release position
        # box_release_position = [0.5,0.08,-0.12]
        # home_position = [0.49,0.11,0.03]

        # go to cartesian end point -  need to implement
        # self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # self.tcp_socket.connect((self.tcp_host_ip, self.tcp_port))
        # tcp_command = "def process():\n"
        # tcp_command += " movej(p[%f,%f,%f,%f,%f,%f],a=%f,v=%f,t=0,r=0.00)\n" % (box_release_position[0],box_release_position[1],box_release_position[2],tool_orientation[0],tool_orientation[1],tool_orientation[2],self.joint_acc*0.1,self.joint_vel*0.1)
        # tcp_command += " movej(p[%f,%f,%f,%f,%f,%f],a=%f,v=%f,t=0,r=0.00)\n" % (box_release_position[0],box_release_position[1],box_release_position[2]+0.3,tool_orientation[0],tool_orientation[1],tool_orientation[2],self.joint_acc*0.02,self.joint_vel*0.02)
        # tcp_command += " movej(p[%f,%f,%f,%f,%f,85264372%f],a=%f,v=%f,t=0,r=0.00)\n" % (box_grab_position[0],box_grab_position[1],box_grab_position[2],tool_orientation[0],tool_orientation[1],tool_orientation[2],self.joint_acc*0.1,self.joint_vel*0.1)
        # tcp_command += " movej(p[%f,%f,%f,%f,%f,%f],a=%f,v=%f,t=0,r=0.00)\n" % (box_grab_position[0]+0.05,box_grab_position[1],box_grab_position[2],tool_orientation[0],tool_orientation[1],tool_orientation[2],self.joint_acc*0.1,self.joint_vel*0.1)
        # tcp_command += " set_digital_out(8,False)\n"
        # tcp_command += " movej(p[%f,%f,%f,%f,%f,%f],a=%f,v=%f,t=0,r=0.09)\n" % (box_grab_position[0],box_grab_position[1],box_grab_position[2]+0.1,tilted_tool_orientation[0],tilted_tool_orientation[1],tilted_tool_orientation[2],self.joint_acc,self.joint_vel)
        # tcp_command += " movej(p[%f,%f,%f,%f,%f,%f],a=%f,v=%f,t=0,r=0.00)\n" % (home_position[0],home_position[1],home_position[2],tool_orientation[0],tool_orientation[1],tool_orientation[2],self.joint_acc,self.joint_vel)
        # tcp_command += "end\n"
        # self.tcp_socket.send(str.encode(tcp_command))
        # self.tcp_socket.close()

        # Block until robot reaches home position
        # state_data = self.get_state()
        # tool_analog_input2 = self.parse_tcp_state_data(state_data, 'tool_data')
        # while True:
        #     state_data = self.get_state()
        #     new_tool_analog_input2 = self.parse_tcp_state_data(state_data, 'tool_data')
        #     actual_tool_pose = self.parse_tcp_state_data(state_data, 'cartesian_info')
        #     if tool_analog_input2 > 3.0 and (abs(new_tool_analog_input2 - tool_analog_input2) < 0.01) and all([np.abs(actual_tool_pose[j] - home_position[j]) < self.tool_pose_tolerance[j] for j in range(3)]):
        #         break
        #     tool_analog_input2 = new_tool_analog85264372_input2


        goal_reached = self.get_task_score() == self.num_obj
        return goal_reached


    def get_camera_data(self):
        # Get color and depth image from ROS service
        color_img, depth_img = self.camera.get_data()
        # color_img = self.camera.color_data.copy()
        # depth_img = self.camera.depth_data.copy()

        return color_img, depth_img


    def parse_state_data(self, state_data, subpackage):
        substate = state_data[subpackage]

        def parse_joint_data(substate):
            return substate

        def parse_pose_info(substate):
            # Change matrix to vector
            t = substate.translation
            R = substate.rotation
            rpy = utils.rotm2euler(R)

            return np.array([t[0], t[1], t[2], rpy[0], rpy[1], rpy[2]])

        def parse_tool_data(substate):
            return substate

        parse_functions = {'joints' : parse_joint_data, 'pose' : parse_pose_info, 'gripper_width' : parse_tool_data}
        return parse_functions[subpackage](substate)


    def parse_rtc_state_data(self, state_data):
        return state_data['ee_force_torque']


    def close_gripper(self, asynch=False):
        self.fa.close_gripper()
        if asynch:
            gripper_fully_closed = True
        else:
            time.sleep(1.5)
            gripper_fully_closed = True

        return gripper_fully_closed

    def open_gripper(self, asynch=False):
        self.fa.open_gripper()
        if not asynch:
            time.sleep(1.5)


    def get_state(self):
        state_data = self.fa.get_robot_state()
        return state_data

    def move_to(self, tool_position, tool_orientation):
        if tool_orientation is None:
            R = self.fa.get_pose().rotation
        else:
            R = utils.euler2rotm(tool_orientation)
        pose = RigidTransform(rotation=R, translation=tool_position, from_frame='franka_tool', to_frame='world')
        self.fa.goto_pose(pose)

    def move_joints(self, joint_configuration):
        self.fa.goto_joints(joint_configuration)

    def go_home(self):
        self.fa.reset_joints()

    # Primitives ----------------------------------------------------------

    def grasp(self, position, heightmap_rotation_angle, workspace_limits):
        print('Executing: grasp at (%f, %f, %f)' % (position[0], position[1], position[2]))
        # Compute tool orientation from heightmap rotation angle
        # grasp_orientation = [1.0,0.0]
        heightmap_rotation_angle += np.pi/2
        if heightmap_rotation_angle > np.pi:
                heightmap_rotation_angle = heightmap_rotation_angle - np.pi

        # tool_rotation_angle = heightmap_rotation_angle/2
        tool_orientation = np.asarray([np.pi, 0, heightmap_rotation_angle])
        print(tool_orientation)
        # tool_orientation = np.asarray([grasp_orientation[0]*np.cos(tool_rotation_angle) - grasp_orientation[1]*np.sin(tool_rotation_angle), grasp_orientation[0]*np.sin(tool_rotation_angle) + grasp_orientation[1]*np.cos(tool_rotation_angle), 0.0])*np.pi
        tool_orientation_angle = np.linalg.norm(tool_orientation)
        tool_orientation_axis = tool_orientation/tool_orientation_angle
        tool_orientation_rotm = utils.angle2rotm(tool_orientation_angle, tool_orientation_axis, point=None)[:3,:3]

        # Compute tilted tool orientation during dropping into bin
        tilt_rotm = utils.euler2rotm(np.asarray([-np.pi/4,0,0]))
        tilted_tool_orientation_rotm = np.dot(tilt_rotm, tool_orientation_rotm)
        tilted_tool_orientation_axis_angle = utils.rotm2angle(tilted_tool_orientation_rotm)
        tilted_tool_orientation = tilted_tool_orientation_axis_angle[0]*np.asarray(tilted_tool_orientation_axis_angle[1:4])

        # Attempt grasp
        position = np.asarray(position).copy()
        position[2] = max(position[2] - 0.05, workspace_limits[2][0])
        self.open_gripper()
        self.move_to([position[0], position[1], position[2]+0.1], tool_orientation)
        self.move_to([position[0], position[1], position[2]], tool_orientation)
        self.close_gripper()

        # Check if gripper is open (grasp might be successful)
        width = self.fa.get_gripper_width()
        print(width)
        gripper_open = width > 0.0050

        # home_position = [0.49,0.11,0.03]
        bin_position = [0.53,-0.30,0.40]

        # If gripper is open, drop object in bin and check if grasp is successful
        grasp_success = False
        if gripper_open:
            self.move_to([position[0], position[1], position[2]+0.1], tool_orientation)
            self.move_to([position[0], position[1], bin_position[2]], tool_orientation)
            self.move_to([bin_position[0], bin_position[1], bin_position[2]], tool_orientation)
            self.open_gripper()
            self.go_home()
            grasp_success = True

        else:
            self.move_to([position[0], position[1], position[2]+0.1], tool_orientation)
            self.open_gripper()
            self.go_home()

        return grasp_success


    def push(self, position, heightmap_rotation_angle, workspace_limits,heightmap_resolution):
        print('Executing: push at (%f, %f, %f)' % (position[0], position[1], position[2]))

        # Determine locations before push
        color_img, depth_img = self.get_camera_data()
        depth_img = depth_img * self.cam_depth_scale
        color_heightmap, depth_heightmap = utils.get_heightmap(color_img,depth_img,self.cam_intrinsics,self.cam_pose,workspace_limits,heightmap_resolution)
        valid_depth_heightmap = depth_heightmap.copy()
        valid_depth_heightmap[np.isnan(valid_depth_heightmap)] = 0
        stuff_count_before = np.zeros(valid_depth_heightmap.shape)
        stuff_count_before[valid_depth_heightmap > 0.028] = 1

        # Compute tool orientation from heightmap rotation angle
        push_orientation = [1.0,0.0]

        heightmap_rotation_angle += np.pi/2
        if heightmap_rotation_angle > np.pi:
                heightmap_rotation_angle = heightmap_rotation_angle - np.pi
                push_orientation = [-1.0, 0.0]

        tool_orientation = np.asarray([np.pi, 0, heightmap_rotation_angle])

        # Compute push direction and endpoint (push to right of rotated heightmap)
        push_direction = np.asarray([push_orientation[0]*np.cos(heightmap_rotation_angle) - push_orientation[1]*np.sin(heightmap_rotation_angle), push_orientation[0]*np.sin(heightmap_rotation_angle) + push_orientation[1]*np.cos(heightmap_rotation_angle), 0.0])
        target_x = min(max(position[0] + push_direction[0]*0.1, workspace_limits[0][0]), workspace_limits[0][1])
        target_y = min(max(position[1] + push_direction[1]*0.1, workspace_limits[1][0]), workspace_limits[1][1])
        push_endpoint = np.asarray([target_x, target_y, position[2]])
        push_direction.shape = (3,1)

        # Push only within workspace limits
        position = np.asarray(position).copy()
        position[0] = min(max(position[0], workspace_limits[0][0]), workspace_limits[0][1])
        position[1] = min(max(position[1], workspace_limits[1][0]), workspace_limits[1][1])
        position[2] = max(position[2] + 0.005, workspace_limits[2][0] + 0.005) # Add buffer to surface

        # Attempt push
        self.close_gripper()
        self.move_to([position[0], position[1], position[2]+0.1], tool_orientation)
        self.move_to(position, tool_orientation)
        self.move_to(push_endpoint, tool_orientation)
        self.move_to([position[0], position[1], position[2]+0.1], tool_orientation)
        self.go_home()

        # Determine locations after push
        color_img, depth_img = self.get_camera_data()
        depth_img = depth_img *self.cam_depth_scale
        color_heightmap, depth_heightmap = utils.get_heightmap(color_img,depth_img,self.cam_intrinsics,self.cam_pose,workspace_limits,heightmap_resolution)
        valid_depth_heightmap = depth_heightmap.copy()
        valid_depth_heightmap[np.isnan(valid_depth_heightmap)] = 0
        stuff_count_after = np.zeros(valid_depth_heightmap.shape)
        stuff_count_after[valid_depth_heightmap > 0.028] = 1

        # Determine if push is a success
        differences = np.sum(np.abs(stuff_count_after - stuff_count_before))
        print("Pixel differences: ", differences)
        push_success = False
        if differences > 400:
            push_success = True

        # time.sleep(0.5)

        return push_success


    def restart_real(self):
        # Tool orientations
        default_orientation = np.asarray([np.pi, 0, 0])

        # Move to box grabbing position
        box_above_position = [0.612, -0.183, 0.390] # Determine this first
        box_grab_position = [0.612, -0.183, 0.290] # Determine this first
        box_drag_position = [0.5, 0.18, 0.290]
        box_dump_position = [0.5, 0.18, 0.7] #Determine this first
        box_reset_position = [0.612, -0.183, 0.55]

        # Idea: Move above the box_grab position, then move down to the grab position, close, up again, move to drop position, tilt, untilt, back to above, down, up, reset
        self.move_to(box_above_position,default_orientation)
        self.move_to(box_grab_position,default_orientation)
        self.close_gripper()
        # self.move_to(box_above_position,default_orientation)
        self.move_to(box_drag_position,default_orientation)
        self.move_to(box_dump_position,default_orientation)
        self.move_to(box_reset_position,default_orientation)
        self.move_to(box_above_position,default_orientation)
        self.move_to(box_grab_position,default_orientation)
        self.open_gripper()
        self.go_home()





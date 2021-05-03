#!/usr/bin/env python

import pyrealsense2 as rs
import rospy
import h5py
import numpy as np
import math
import time
from std_msgs.msg import String
from mservo_mani.msg import joint_vec

np.random.seed(910920)

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)

pipeline.start(config)
align_to = rs.stream.color
align = rs.align(align_to)

i = 0
des_joint_vec = np.array([0, 0, 0, 0, 0, 0, 0, 0])
next = True

def DHmatrix(a, alpha, d, theta):
    c_th = math.cos(theta)
    s_th = math.sin(theta)
    c_al = math.cos(alpha)
    s_al = math.sin(alpha)
    A = np.array([[c_th, -c_al*s_th, s_al*s_th, a*c_th],
                 [s_th, c_al*c_th, -s_al*c_th, a*s_th],
                 [0.0, s_al, c_al, d],
                 [0.0, 0.0, 0.0, 1.0]])
    return A

def random_generator():
    while True:
        joint_range = np.array([[math.pi/8.0, math.pi/2.0],
                               [math.pi/2.0, math.pi/2.0],
                               [math.pi/4.0, math.pi/4.0*3.],
                               [-math.pi/2.0, 0.0],
                               [-math.pi/3.0, math.pi/3.0],
                               [0.0, math.pi/2.0],
                               [-math.pi / 4.0, math.pi / 4.0],
                               [0.0, 0.1]])
        random_vec = np.random.rand(8)
        des_joint_vec = np.zeros((8,))
        for k in range(8):
            des_joint_vec[k] = random_vec[k]*(joint_range[k][1]-joint_range[k][0])+joint_range[k][0]

        A = np.zeros((7,4,4))
        alpha = np.array([-math.pi/2.0, math.pi/2.0, math.pi/2.0, -math.pi/2.0, -math.pi/2.0, math.pi/2.0, 0.0])
        d = np.array([0.167, 0., 0.330, 0., 0.314, 0., 0.180])
        l_z = np.array([0., 0., 0.17, 1.])
        l_x = np.array([0.1, 0., 0., 1.])
        T = np.eye(4)
        for k in range(7):
            A[k] = DHmatrix(0., alpha[k], d[k], des_joint_vec[k])
            T = np.matmul(T, A[k])

        end_pos = np.matmul(T, l_z)
        end_pos_x = np.matmul(T, l_x)
        if end_pos[0] > -0.2 and end_pos[0] < 0.2 and\
                end_pos[1] > 0.35 and end_pos[1] < 0.75 and\
                end_pos[2] > 0.1 and end_pos[2] < 0.3:
            break

    return des_joint_vec, end_pos, end_pos_x, random_vec


def callback(data):
    global i, des_joint_vec, next, random_vec, end_pos, end_pos_x
    rospy.loginfo("take image")
    while True:
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        aligned_depth_frame = aligned_frames.get_depth_frame()  # aligned_depth_frame is a 640x480 depth image
        color_frame = aligned_frames.get_color_frame()

        if not aligned_depth_frame or not color_frame:
            continue

        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        f = h5py.File('/home/mservo/Dataset/test_{0:04d}.hdf5'.format(i), 'w')
        g = f.create_group("data")
        g['image'] = color_image
        g['depth'] = depth_image
        g['joint_state'] = random_vec
        g['end_pos'] = end_pos[:-1]
        g['end_pos_x'] = end_pos_x[:-1]
        i += 1
        next = True
        break

def imgSaver():
    global des_joint_vec, random_vec, next, i, end_pos, end_pos_x
    pub = rospy.Publisher('des_joint_vec', joint_vec, queue_size=1)
    rospy.init_node('imgSaver', anonymous=True)
    rospy.Subscriber("take_img", String, callback)
    rate = rospy.Rate(2)
    count = 0

    des_joint_vec = np.zeros((8,))
    pub.publish(des_joint_vec)
    time.sleep(1)
    # for i in range(911):
    #     des_joint_vec, end_pos, random_vec = random_generator()
    #     count += 1
    #     i += 1

    while not rospy.is_shutdown():
        # check previous target is reached or not
        if not next:
            continue
        # generate random desired joint state and check constraint
        des_joint_vec, end_pos, end_pos_x, random_vec = random_generator()
        # des_joint_vec = np.zeros((8,))
        # des_joint_vec[0] = count

        rospy.loginfo(des_joint_vec[5])
        rospy.loginfo(end_pos)
        pub.publish(des_joint_vec)

        count += 1
        next = False
        if count == 1000:
            break
        rate.sleep()

if __name__ == '__main__':
    try:
        imgSaver()
    except rospy.ROSInterruptException:
        pass

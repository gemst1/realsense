#!/usr/bin/env python
import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import pyrealsense2 as rs
import numpy as np
import cv2
import time
import h5py

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

pipeline.start(config)

align_to = rs.stream.color
align = rs.align(align_to)

i = 0
time.sleep(1)
ep_num = 0
record = False
fourcc = cv2.VideoWriter_fourcc(*'XVID')

def print_end(ep_num):
    print('Episode %d is ended.' % ep_num)
    print('If you want to stop recording process, press q.')

try:
    command = input("Input start episode number and press enter to start to record demonstrations.")
    if command == 'q':
        pass
    else:
        try:
            ep_num = int(command)
            print("Episode starts from %d" % ep_num)
            print("Press s to start new episode, n to finish this episode %d." % ep_num)
            while True:

                frames = pipeline.wait_for_frames()
                aligned_frames = align.process(frames)
                aligned_depth_frame = aligned_frames.get_depth_frame()  # aligned_depth_frame is a 640x480 depth image
                color_frame = aligned_frames.get_color_frame()

                # depth_frame = frames.get_depth_frame()
                # color_frame = frames.get_color_frame()
                if not aligned_depth_frame or not color_frame:
                    continue

                depth_image = np.asanyarray(aligned_depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())

                # crop image
                depth_image = depth_image[:,80:560]
                color_image = color_image[:,80:560,:]

                depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

                images = np.vstack((color_image, depth_colormap))
                cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
                cv2.imshow('RealSense', images)
                time.sleep(0.1)

                if record:
                    out.write(color_image)
                    i += 1

                key = cv2.waitKey(1)
                if key == ord('s'):
                    print('Start episode %d.' % ep_num)
                    record = True
                    out = cv2.VideoWriter('./demonstrations/episode_%d.mp4' % ep_num, fourcc, 20.0, (480, 480))
                elif key == ord('n') or i == 50:
                    if record:
                        out.release()
                        record = False
                        print_end(ep_num)
                        i = 0
                        ep_num += 1
                elif key == ord('q'):
                    print_end(ep_num)
                    break
        except:
            print("Please put int value for starting episode number.")
    cv2.destroyAllWindows()

finally:
    pipeline.stop()
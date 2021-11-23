import argparse
import logging
import pyrealsense2 as rs

import torch.utils.data

from models.common import post_process_output
from utils.dataset_processing import evaluation, grasp
from utils.data import get_dataset
from utils.dataset_processing import grasp, image
from matplotlib import pyplot as plt
import numpy as np
import os
import json
import pdb
import cv2
import tifffile
import math
import time

import rospy
from std_msgs.msg import String, Float64MultiArray

logging.basicConfig(level=logging.INFO)


def parse_args():
    parser = argparse.ArgumentParser(description='Evaluate GG-CNN')

    # Network
    parser.add_argument('--network', type=str, default='ggcnn_weights_cornell/ggcnn_epoch_23_cornell', help='Path to saved network to evaluate')
    parser.add_argument('--num-workers', type=int, default=8, help='Dataset workers')
    parser.add_argument('--n-grasps', type=int, default=1, help='Number of grasps to consider per image')

    args = parser.parse_args()

    return args

def get_depth(depth_file, rot=0, zoom=1.0, output_size=300):
    depth_img = image.DepthImage.from_tiff(depth_file)
    depth_img.inpaint()
    depth_img.rotate(rot)
    depth_img.normalise()
    depth_img.zoom(zoom)
    depth_img.resize((output_size, output_size))
    return depth_img.img


def get_rgb(rgb_file, rot=0, zoom=1.0, output_size=300, normalise=True):
    rgb_img = image.Image.from_file(rgb_file)
    rgb_img.rotate(rot)
    rgb_img.zoom(zoom)
    rgb_img.resize((output_size, output_size))
    if normalise:
        rgb_img.normalise()
        rgb_img.img = rgb_img.img.transpose((2, 0, 1))
    return rgb_img.img

def get_rgb_from_array(rgb_array, rot=0, zoom=1.0, output_size=300, normalise=True):
    rgb_img = image.Image.from_array(rgb_array)
    rgb_img.rotate(rot)
    rgb_img.zoom(zoom)
    rgb_img.resize((output_size, output_size))
    if normalise:
        rgb_img.normalise()
        rgb_img.img = rgb_img.img.transpose((2, 0, 1))
    return rgb_img.img

def get_depth_from_array(depth_array, rot=0, zoom=1.0, output_size=300):
    depth_img = image.DepthImage.from_array(depth_array)
    depth_img.inpaint()
    depth_img.rotate(rot)
    depth_img.normalise()
    depth_img.zoom(zoom)
    depth_img.resize((output_size, output_size))
    return depth_img.img

def numpy_to_torch(s):
    if len(s.shape) == 2:
        return torch.from_numpy(np.expand_dims(s, 0).astype(np.float32))
    else:
        return torch.from_numpy(s.astype(np.float32))

def rotate_points(pt, center, th):
    """
    th -> -th
    """
    x, y = pt
    cx, cy = center

    x -= cx
    y -= cy

    xx = x
    x = x*math.cos(-th) - y*math.sin(-th)
    y = xx * math.sin(-th) + y*math.cos(-th)

    x += cx
    y += cy

    return [x,y]


if __name__ == '__main__':
    args = parse_args()

    net = torch.load(args.network)
    device = torch.device("cuda:0")

    # load depth input
    BasePath = '/home/tc/Github_repo/ggcnn/samples'
    # rgb_f = os.path.join(BasePath, '4.png')
    # rgb_im = get_rgb(rgb_f, normalise=False)
    # depth_f = rgb_f.replace('png', 'tiff')
    # depth_im = get_depth(depth_f)
    count = len(os.listdir(BasePath))
    img_index = count // 2

    # init rosnode and topic publisher
    rospy.init_node('ggcnn')
    grasp_pose_pub = rospy.Publisher('/ggcnn_grasp_pose', Float64MultiArray, queue_size=10)

    # RGB and dpeth strams
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    profile = pipeline.start(config)

    # Get depth camera's depth scale
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    print('Depth Scale is: ' + str(depth_scale))

    # Align depth frame to RGB frame
    align_to = rs.stream.color
    align = rs.align(align_to)

    sensor = pipeline.get_active_profile().get_device().query_sensors()[1]
    sensor.set_option(rs.option.exposure, 250)
    sensor.set_option(rs.option.auto_exposure_priority, True)

    # grasp drawing params
    radius = 1
    color = (0, 0, 255)
    thickness = 2
    isClosed = True


    while True:

        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        # Align the depth frame to color frame
        aligned_frames = align.process(frames)
        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame()  # aligned_depth_frame is a 640x480 depth image
        color_frame = aligned_frames.get_color_frame()

        # get instrinc params
        # color_profile = color_frame.get_profile()
        # cvsprofile = rs.video_stream_profile(color_profile)
        # color_intrin = cvsprofile.get_intrinsics()
        # color_intrin_part = [color_intrin.ppx, color_intrin.ppy, color_intrin.fx, color_intrin.fy]
        # print(color_intrin_part)

        # Validate that both frames are valid
        if not aligned_depth_frame or not color_frame:
            raise Exception("Invalid depth or rgb data.")

        depth_array = np.asanyarray(aligned_depth_frame.get_data())
        depth_array = (depth_array/1000.0).astype(np.float32)
        color_array = np.asanyarray(color_frame.get_data()).astype(np.uint8)

        rgb_im = get_rgb_from_array(color_array, normalise=False)
        depth_im = get_depth_from_array(depth_array)

        depth_tensor = numpy_to_torch(depth_im).unsqueeze(0).to(device)
        grasp_mask = os.path.join(BasePath, 'output/grasp_mask.npy')
        rel_file = os.path.join(BasePath, 'output/point_rel.json')

        # test on depth input
        TOG_FLAG = False
        with torch.no_grad():
            preds = net.compute_grasp(depth_tensor)
            q_img, ang_img, width_img = post_process_output(preds['pos'], preds['cos'], preds['sin'], preds['width'])
            if TOG_FLAG:
                out_grasps = evaluation.plot_output(rgb_im, depth_im, q_img,
                                            ang_img, no_grasps=args.n_grasps, grasp_width_img=width_img, grasp_mask=grasp_mask, rel_file=rel_file)
            else:
                # out_grasps = evaluation.plot_output(rgb_im, depth_im, q_img,
                #                     ang_img, no_grasps=args.n_grasps, grasp_width_img=width_img)
                out_grasps = evaluation.compute_grasps(rgb_im, depth_im, q_img,
                                    ang_img, no_grasps=args.n_grasps, grasp_width_img=width_img)
        

        # output predicted grasp parameters
        TOG = 'task-oriented' if TOG_FLAG else 'task-free'
        print("Predicting %s grasps:" %TOG)
        for grasp in out_grasps:
            print("center: " + str(grasp.center) + ", quality: %f, depth: %f, width: %f, length: %f, angle: %f" % (grasp.quality, depth_array[grasp.center[0], grasp.center[1]], grasp.width, grasp.length, grasp.angle))
        
        grasp_topic = Float64MultiArray(data=[np.float64(grasp.center[0]), np.float64(grasp.center[1]), depth_array[grasp.center[0], grasp.center[1]], grasp.angle, grasp.length, grasp.quality])
        grasp_pose_pub.publish(grasp_topic)

        # rotate grasp points
        pts = np.array([

        rotate_points((grasp.center[1]-grasp.length//2, grasp.center[0]-grasp.width//2,
        ), (grasp.center[1], grasp.center[0]), grasp.angle),

          rotate_points((grasp.center[1]+grasp.length//2, grasp.center[0]-grasp.width//2,
        ), (grasp.center[1], grasp.center[0]), grasp.angle),

        rotate_points((grasp.center[1]+grasp.length//2, grasp.center[0]+grasp.width//2,
        ), (grasp.center[1], grasp.center[0]), grasp.angle),

        rotate_points((grasp.center[1]-grasp.length//2, grasp.center[0]+grasp.width//2,
        ), (grasp.center[1], grasp.center[0]), grasp.angle)], np.int32)

        pts = pts.reshape((-1, 1, 2))


        # draw computed grasps in real time
        cv2.polylines(color_array, [pts], 
                      isClosed, color, thickness)
        cv2.circle(color_array, (grasp.center[1], grasp.center[0]), radius, color, thickness)
        cv2.imshow('detected grasps', color_array)


        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            cv2.destroyAllWindows()
            break
        elif key == ord('s'):
            depth_name = str(img_index)+'.tiff'
            color_name = str(img_index) + '.png'
            cv2.imwrite(os.path.join(BasePath, color_name), color_array)
            tifffile.imsave(os.path.join(BasePath, depth_name), depth_array)
            print("RGB and depth images %d saved" % img_index)
            img_index += 1
  

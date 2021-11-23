import numpy as np
import matplotlib.pyplot as plt
import cv2
import json
import os

from .grasp import GraspRectangles, detect_grasps


def plot_output(rgb_img, depth_img, grasp_q_img, grasp_angle_img, no_grasps=1, grasp_width_img=None, raw_size=(640, 480), grasp_mask=None, rel_file=None):
    """
    Plot the output of a GG-CNN
    :param rgb_img: RGB Image
    :param depth_img: Depth Image
    :param grasp_q_img: Q output of GG-CNN
    :param grasp_angle_img: Angle output of GG-CNN
    :param no_grasps: Maximum number of grasps to plot
    :param grasp_width_img: (optional) Width output of GG-CNN
    :return:
    """
    # resize
    INTER_TYPE = cv2.INTER_CUBIC
    width_factor = (raw_size[0]/grasp_q_img.shape[0] + raw_size[1]/grasp_q_img.shape[1]) / 2
    grasp_q_img = cv2.resize(grasp_q_img, dsize=raw_size, interpolation=INTER_TYPE)
    grasp_angle_img = cv2.resize(grasp_angle_img, dsize=raw_size, interpolation=INTER_TYPE)
    grasp_width_img = cv2.resize(grasp_width_img, dsize=raw_size, interpolation=INTER_TYPE) * width_factor
    rgb_img = cv2.resize(rgb_img, dsize=raw_size, interpolation=INTER_TYPE)
    depth_img = cv2.resize(depth_img, dsize=raw_size, interpolation=INTER_TYPE)

    # TODO: save grasps
    if grasp_mask is not None:
        mask = np.load(grasp_mask)
        grasp_q_img_mask = grasp_q_img * mask
        gs = detect_grasps(grasp_q_img_mask, grasp_angle_img, width_img=grasp_width_img, no_grasps=no_grasps)
    else:
        gs = detect_grasps(grasp_q_img, grasp_angle_img, width_img=grasp_width_img, no_grasps=no_grasps)

    id_rel = {1: 'scoop', 2: 'pour', 3: 'cut', 4: 'contain', 5: 'dump', 6: 'wipe'}

    if rel_file is not None:
        f = open(rel_file, 'r')
        c = f.read()
        rel = json.loads(c)
        rel_id, sub_ponit, obj_point = rel['rel_id'], rel['sub_point'], rel['obj_point']
        distance = (np.array(obj_point) - np.array(sub_ponit))/3
        start_point = distance + np.array(sub_ponit)
        f.close()

    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot(2, 2, 1)
    ax.imshow(rgb_img)
    for g in gs:
        g.plot(ax)
        # print("center: " + str(g.center) + ", width: %f, length: %f, angle: %f" %(g.width, g.length, g.angle))


    if rel_file is not None:
        ax.set_title('RGB, Action: ' + id_rel[rel_id])
        plt.arrow(start_point[0], start_point[1], distance[0], distance[1], head_width=10, color='g')
        plt.plot(obj_point[0], obj_point[1], 'o', color='b')
    else:
        ax.set_title('RGB')
    ax.axis('off')

    ax = fig.add_subplot(2, 2, 2)
    ax.imshow(depth_img, cmap='gray')
    for g in gs:
        g.plot(ax)
    if rel_file is not None:
        ax.set_title('Depth, Action: ' + id_rel[rel_id])
        plt.arrow(start_point[0], start_point[1], distance[0], distance[1], head_width=10, color='g')
        plt.plot(obj_point[0], obj_point[1], 'o', color='b')
    else:
        ax.set_title('Depth')
    ax.axis('off')

    ax = fig.add_subplot(2, 2, 3)
    plot = ax.imshow(grasp_q_img, cmap='jet', vmin=0, vmax=1)
    ax.set_title('Q')
    ax.axis('off')
    plt.colorbar(plot)

    ax = fig.add_subplot(2, 2, 4)
    plot = ax.imshow(grasp_angle_img, cmap='hsv', vmin=-np.pi / 2, vmax=np.pi / 2)
    ax.set_title('Angle')
    ax.axis('off')
    plt.colorbar(plot)
    plt.show()

    return gs

def compute_grasps(rgb_img, depth_img, grasp_q_img, grasp_angle_img, no_grasps=1, grasp_width_img=None, raw_size=(640, 480), grasp_mask=None, rel_file=None):
    """
    Plot the output of a GG-CNN
    :param rgb_img: RGB Image
    :param depth_img: Depth Image
    :param grasp_q_img: Q output of GG-CNN
    :param grasp_angle_img: Angle output of GG-CNN
    :param no_grasps: Maximum number of grasps to plot
    :param grasp_width_img: (optional) Width output of GG-CNN
    :return:
    """
    # # resize
    INTER_TYPE = cv2.INTER_CUBIC
    width_factor = (raw_size[0]/grasp_q_img.shape[0] + raw_size[1]/grasp_q_img.shape[1]) / 2
    grasp_q_img = cv2.resize(grasp_q_img, dsize=raw_size, interpolation=INTER_TYPE)
    grasp_angle_img = cv2.resize(grasp_angle_img, dsize=raw_size, interpolation=INTER_TYPE)
    grasp_width_img = cv2.resize(grasp_width_img, dsize=raw_size, interpolation=INTER_TYPE) * width_factor
    rgb_img = cv2.resize(rgb_img, dsize=raw_size, interpolation=INTER_TYPE)
    depth_img = cv2.resize(depth_img, dsize=raw_size, interpolation=INTER_TYPE)

    # TODO: save grasps
    if grasp_mask is not None:
        mask = np.load(grasp_mask)
        grasp_q_img_mask = grasp_q_img * mask
        gs = detect_grasps(grasp_q_img_mask, grasp_angle_img, width_img=grasp_width_img, no_grasps=no_grasps)
    else:
        gs = detect_grasps(grasp_q_img, grasp_angle_img, width_img=grasp_width_img, no_grasps=no_grasps)

    return gs



def calculate_iou_match(grasp_q, grasp_angle, ground_truth_bbs, no_grasps=1, grasp_width=None):
    """
    Calculate grasp success using the IoU (Jacquard) metric (e.g. in https://arxiv.org/abs/1301.3592)
    A success is counted if grasp rectangle has a 25% IoU with a ground truth, and is withing 30 degrees.
    :param grasp_q: Q outputs of GG-CNN (Nx300x300x3)
    :param grasp_angle: Angle outputs of GG-CNN
    :param ground_truth_bbs: Corresponding ground-truth BoundingBoxes
    :param no_grasps: Maximum number of grasps to consider per image.
    :param grasp_width: (optional) Width output from GG-CNN
    :return: success
    """

    if not isinstance(ground_truth_bbs, GraspRectangles):
        gt_bbs = GraspRectangles.load_from_array(ground_truth_bbs)
    else:
        gt_bbs = ground_truth_bbs
    gs = detect_grasps(grasp_q, grasp_angle, width_img=grasp_width, no_grasps=no_grasps)
    for g in gs:
        if g.max_iou(gt_bbs) > 0.25:
            return True
    else:
        return False
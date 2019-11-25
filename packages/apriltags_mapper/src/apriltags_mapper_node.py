#!/usr/bin/env python
from __future__ import print_function

import os
import time
import thread
import yaml
from collections import defaultdict
from pydataclasses import DataClass
from utils import Pool, Worker

# ROS libs
import tf
import rospy
import numpy as np
from duckietown_msgs.msg import AprilTagDetectionArray
from sensor_msgs.msg import CompressedImage, CameraInfo

# Duckietown libs
from duckietown import DTROS

NUM_THREADS = 20
NUM_REDUNDANT_DETECTIONS = 1
TOLERANCE_ROTATION_DEG = 5
# rotation conventions: (1) clockwise, (-1) counter-clockwise
ROTATION_CONVENTION = 1
TILE_SIZE_M = 0.61


class TF(DataClass):
    trans = None
    rot = None

class ApriltagsMapperNode(DTROS):

    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(ApriltagsMapperNode, self).__init__(node_name=node_name)

        # Add the node parameters to the parameters dictionary and load their default values
        self.parameters['~origin_tag'] = None
        self.parameters['~detections_topic'] = None
        self.parameters['~camera_frame'] = None
        self.parameters['~tags_range'] = None
        self.parameters['~image_topic'] = None
        self.parameters['~camera_topic'] = None
        self.parameters['~snap_position'] = None
        self.parameters['~snap_orientation'] = None
        self.parameters['~orientation_resolution_deg'] = None
        self.updateParameters()
        # parse tags_range
        self.tags_range = tuple(map(int, self.parameters['~tags_range'].split(':')))
        assert len(self.tags_range) == 2
        # parse origin_tag
        self.origin_tag = int(self.parameters['~origin_tag'])
        self.origin_frame = 'Tag%d' % int(self.origin_tag)
        # self.origin_frame = self.parameters['~camera_frame']

        # set of detected tags
        self.visible_tags = set()

        # subscribe to detections
        self.tags_discoverer = self.subscriber(
            self.parameters['~detections_topic'],
            AprilTagDetectionArray,
            self.detectionsCb
        )

    def detectionsCb(self, msg):
        for detection in msg.detections:
            if detection.id[0] < self.tags_range[0] or detection.id[0] > self.tags_range[1]:
                continue
            self.visible_tags.add(detection.id[0])

    def process(self):
        # sleep for wtime seconds
        wtime = 2
        # ptime = 1
        self.log('Allowing %d seconds for all the tags to be detected at least once' % wtime)
        time.sleep(wtime)
        #
        num_detected = len(self.visible_tags)
        self.log('Detected %d unique tags' % num_detected)
        self.tags_discoverer.active = False
        if num_detected == 0:
            msg = 'No tags found! Exiting...'
            self.log(msg, 'err')
            rospy.signal_shutdown(msg)
        # make sure that the origin tag can be detected
        if self.origin_tag not in self.visible_tags:
            msg = 'The origin ID #%s was not detected! Exiting...' % self.origin_tag
            self.log(msg, 'err')
            rospy.signal_shutdown(msg)
        # sort visible tags
        self.visible_tags = sorted(self.visible_tags)
        # get transformations from camera_frame to tag_frame
        tfs = self.get_transformations(
            self.parameters['~camera_frame'],
            self.visible_tags
        )
        # project tags onto the image plane
        tfs = project_transformations(tfs)
        # the origin of the map will be at the origin tag
        to_origin = tf.transformations.inverse_matrix(
            tf.transformations.compose_matrix(
                translate=tfs[self.origin_tag].trans,
                angles=np.deg2rad(tfs[self.origin_tag].rot)
            )
        )
        # tranform all the points wrt the origin tag
        tags = {}
        map_cfg = {}
        for tag, tag_t in tfs.items():
            tag_p = np.array(tag_t.trans.tolist() + [1])
            tag_p = to_origin.dot(tag_p)[:2]
            tags[tag] = tag_p
            map_cfg[tag] = (
                np.floor(tag_p[0] / TILE_SIZE_M),
                np.floor(tag_p[1] / TILE_SIZE_M),
                tag_p[0] % TILE_SIZE_M,
                tag_p[1] % TILE_SIZE_M,
                -1
            )



        # t301_p = [tfs[301].trans[0], tfs[301].trans[1], tfs[301].trans[2]]
        # t301_p = np.array(t301_p + [1])
        # print(to_origin.shape)
        # print(t301_p.shape)
        # print(to_origin.dot(t301_p))









        #
        self.log('-' * 40)

        # for f in self.visible_tags:
        #     origin = self.parameters['~camera_frame']
        #     dest = 'Tag%s' % f
        #     self.log('Position of [%s] wrt [%s] :: %r' % (dest, origin, tfs[f].trans))
        #     self.log('Rotation of [%s] wrt [%s] :: %r' % (dest, origin, tfs[f].rot))

        for tag in self.visible_tags:
            self.log('Position of [Tag%s] wrt [Tag%s] :: \n%r' % (tag, self.origin_tag, map_cfg[tag]))


        # shutdown the node
        rospy.signal_shutdown(0)



    def get_transformations(self, from_frame, to_tags, n=NUM_REDUNDANT_DETECTIONS):
        # define task
        def f(mapper, from_frame, to_tag, n):
            tfs = []
            to_frame = 'Tag%s' % str(to_tag)
            listener = tf.TransformListener()
            listener.waitForTransform(from_frame, to_frame, rospy.Time(), rospy.Duration(4.0))
            while (not mapper.is_shutdown) and (len(tfs) < n):
                try:
                    t = listener.getLatestCommonTime(from_frame, to_frame)
                    trans, rot = listener.lookupTransform(from_frame, to_frame, t)
                    if self.is_valid_transformation(trans, rot):
                        tfs.append(TF(
                            trans=np.array(trans),
                            rot=np.array(rot)
                        ))
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                    mapper.log(e, 'warn')
                time.sleep(0.1)
            return from_frame, to_tag, tfs
        # create a pool of threads
        p = Pool(NUM_THREADS)
        for to_tag in to_tags:
            self.log('Looking up transformations [%s] -> [%s]' % (from_frame, to_tag))
            p.enqueue(f, self, from_frame, to_tag, n)
        # spin workers
        p.run()
        # wait for results
        tfs = dict()
        for f0, f, ts in p.iterate_results():
            self.log('Looked up %d transformations [%s] -> [%s]' % (len(ts), f0, f))
            tfs[f] = extract_transformations(
                ts,
                self.parameters['~snap_position'],
                self.parameters['~snap_orientation'],
                self.parameters['~orientation_resolution_deg']
            )
        # ---
        return tfs

    def is_valid_transformation(self, trans, rot):
        rot_yaw_deg = quat2rpydeg(rot)[2]
        error = angle_error(rot_yaw_deg, self.parameters['~orientation_resolution_deg'])
        return error < TOLERANCE_ROTATION_DEG


def extract_transformations(tfs, snap_trans, snap_rot, rot_res_deg):
    trans = np.average([
       t.trans for t in tfs
    ], axis=0)
    rot = np.average([
        quat2rpydeg(t.rot) for t in tfs
    ], axis=0)
    if snap_rot:
        rot = np.array([int(round(a / rot_res_deg)) * rot_res_deg for a in rot])
    return TF(
        trans=trans,
        rot=rot
    )


def project_transformations(tfs):
    return {
        tag: TF(
            trans=np.array(t.trans[:2].tolist() + [0]),
            rot=t.rot
        ) for tag, t in tfs.items()
    }

def quat2rpydeg(quat):
    rpy = tf.transformations.euler_from_quaternion(quat)
    return [(np.rad2deg(rpy[i]) + 360) % 360 for i in range(3)]

def angle_error(angle, resolution):
    angle = np.abs(angle % resolution)
    error1 = int(angle)
    error2 = int(resolution - angle)
    error = min(error1, error2)
    return error

if __name__ == '__main__':
    # Initialize the node
    mapper_node = ApriltagsMapperNode(node_name='apriltag_mapper_node')
    # Start the node task
    thread.start_new_thread(mapper_node.process, ())
    # Keep it spinning to keep the node alive
    rospy.spin()

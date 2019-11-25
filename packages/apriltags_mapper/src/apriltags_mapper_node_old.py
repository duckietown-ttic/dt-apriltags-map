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

NUM_THREADS = 1
NUM_REDUNDANT_DETECTIONS = 10
ROTATION_OFFSET_DEG = 0
# rotation conventions: (1) clockwise, (-1) counter-clockwise
ROTATION_CONVENTION = -1


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
        self.parameters['~average_num_detections'] = None
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
        self.visible_tags = sorted(self.visible_tags)
        # retrieve transformations
        # tfs = self.get_transformations(
        #     ['Tag%s' % t for t in self.visible_tags],
        #     self.origin_frame
        # )
        tfs = self.get_transformations_2(
            self.origin_frame,
            ['Tag%s' % t for t in self.visible_tags]
        )
        #
        self.log('-' * 40)

        for f in self.visible_tags:
            origin = self.origin_frame
            dest = 'Tag%s' % f
            trans, rot = tfs[dest]
            self.log('Position of [%s] wrt [%s] :: %r' % (dest, origin, trans))
            self.log('Rotation of [%s] wrt [%s] :: %r' % (dest, origin, rot))




        # shutdown the node
        rospy.signal_shutdown(0)



    def get_transformations(self, from_frames, to_frame, n=NUM_REDUNDANT_DETECTIONS):
        # define task
        def f(mapper, from_frame, to_frame, n):
            tfs = []
            listener = tf.TransformListener()
            while (not mapper.is_shutdown) and (len(tfs) < n):
                try:
                    t = listener.getLatestCommonTime(from_frame, to_frame)
                    trans, rot = listener.lookupTransform(from_frame, to_frame, t)
                    tfs.append(TF(
                        trans=np.array(trans),
                        rot=np.array(rot)
                    ))
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue
                time.sleep(0.2)
            return from_frame, to_frame, tfs
        # create a pool of threads
        p = Pool(NUM_THREADS)
        for from_frame in from_frames:
            self.log('Looking up transformations [%s] -> [%s]' % (from_frame, to_frame))
            p.enqueue(f, self, from_frame, to_frame, n)
        # spin workers
        p.run()
        # wait for results
        tfs = dict()
        for f, f0, ts in p.iterate_results():
            self.log('Looked up %d transformations [%s] -> [%s]' % (len(ts), f, f0))
            tfs[f] = extract_transformations(
                ts,
                self.parameters['~snap_position'],
                self.parameters['~snap_orientation'],
                self.parameters['~orientation_resolution_deg']
            )
        # ---
        return tfs


    def get_transformations_2(self, from_frame, to_frames, n=NUM_REDUNDANT_DETECTIONS):
        # define task
        def f(mapper, from_frame, to_frame, n):
            tfs = []
            listener = tf.TransformListener()
            while (not mapper.is_shutdown) and (len(tfs) < n):
                try:
                    t = listener.getLatestCommonTime(from_frame, to_frame)
                    trans, rot = listener.lookupTransform(from_frame, to_frame, t)
                    tfs.append(TF(
                        trans=np.array(trans),
                        rot=np.array(rot)
                    ))
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue
                time.sleep(0.2)
            return from_frame, to_frame, tfs
        # create a pool of threads
        p = Pool(NUM_THREADS)
        for to_frame in to_frames:
            self.log('Looking up transformations [%s] -> [%s]' % (from_frame, to_frame))
            p.enqueue(f, self, from_frame, to_frame, n)
        # spin workers
        p.run()
        # wait for results
        tfs = dict()
        for f0, f, ts in p.iterate_results():
            self.log('Looked up %d transformations [%s] -> [%s]' % (len(ts), f, f0))
            tfs[f] = extract_transformations(
                ts,
                self.parameters['~snap_position'],
                self.parameters['~snap_orientation'],
                self.parameters['~orientation_resolution_deg']
            )
        # ---
        return tfs


def extract_transformations(tfs, snap_trans, snap_rot, rot_res_deg):
    print([t.trans for t in tfs])
    trans = np.average([
        t.trans[:2] for t in tfs]
    , axis=0)
    rot = ROTATION_CONVENTION * np.average([
        (np.rad2deg(tf.transformations.euler_from_quaternion(t.rot)[2]) + ROTATION_OFFSET_DEG + 360) % 360 for t in tfs
    ])
    if snap_rot:
        rot = int(round(rot / rot_res_deg)) * rot_res_deg

    return trans, rot



if __name__ == '__main__':
    # Initialize the node
    mapper_node = ApriltagsMapperNode(node_name='apriltag_mapper_node')
    # Start the node task
    thread.start_new_thread(mapper_node.process, ())
    # Keep it spinning to keep the node alive
    rospy.spin()

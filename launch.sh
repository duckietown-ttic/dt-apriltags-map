#!/bin/bash

set -e

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------
roslaunch apriltags_mapper apriltags_mapper.launch \
  tags_range:='300:314' \
  origin_tag:=300 \
  detections_topic:='/apriltag_detector/tag_detections' \
  camera_frame:='/foscam_r2/camera_optical_frame' \
  image_topic:='/foscam_r2/camera_node/crop/rect/image' \
  camera_topic:='/foscam_r2/camera_node/crop/rect/camera_info'

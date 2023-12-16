# Mono Color Segmentation Package

This is a ROS2 package designed for generating segmentation masks from pixels having a specific color in images. It's useful for identifying and segmenting areas with uniform color in visual data: its primary design purpose is to generate masks from the gazebo ground plane, which typically has a uniform color.

![ezgif com-speed](https://github.com/kiwicampus/navigation2/assets/71234974/9144519f-49d2-4d7e-8ce0-55c2994dc27d)


## Dependencies

- [OpenCV](https://opencv.org/)
- [cv_bridge](https://index.ros.org/p/cv_bridge/github-ros-perception-vision_opencv/)
- [vision_msgs](https://index.ros.org/p/vision_msgs/github-ros-perception-vision_msgs)

Install the dependencies by running:

```.sh 
sudo apt-get install libopencv-dev ros-$ROS_DISTRO-cv-bridge ros-$ROS_DISTRO-vision-msgs

```

## Functionality

The Mono Color Segmentation node subscribers to an image topic, processing each received image, and then publishing a binary mask as a `sensor_msgs/msg/Image` with `mono8` encoding and label information as a `vision_msgs/msg/LabelInfo`. It exposes the `target_color` parameter which takes in a vector of 3 `uint8` specifying the RGB color that is to be segmented. The mask is generated using OpenCV's `cv::inRange` function, which sets to 255 all pixels that have the same color as `target_color` and to 0 all other pixels; the `LabelInfo` message thus contains the following class map:

```
- class_id: 0
  class_name: others
- class_id: 255
  class_name: ground_plane
```

## Interfaces

This node uses generic topic names for its subscribers and publishers, however users are encouraged to use namespaces or remappings to integrate it with their own system. The package provides a dummy launch with the `/intel_realsense_r200_depth` namespace to be used with [Nav2's turtlebot simulation](https://navigation.ros.org/getting_started/index.html#running-the-example):

```.sh
ros2 launch mono_color_segmentation turtlebot_world_segmentation.launch.py
```


### Subscribers

- **`/image_raw`** (sensor_msgs/msg/Image): Input image topic, should be a 3 channel color image. 

### Publishers

- **`/mask`**` (sensor_msgs/msg/Image): Binary mask with 255 (white) for pixels matching the target color and 0 (black) for all other pixels.
- **`/label_info`** (vision_msgs/msg/LabelInfo): Class map that assigns human readable names to class ids.

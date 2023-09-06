# l3cam_fusion

This is a ROS2 package for the visualization of the fusion of the L3Cam device sensors. This package relies on [Beamagine](https://beamagine.com/)'s matrix files. It is intended to be used with the [l3cam_ros2](https://github.com/beamaginelidar/l3cam_ros2) package topics.

This package has only been tested with ROS2 foxy on an Ubuntu 20.04 system.

## Installation

Clone this repository in your ROS2 workspace (e.g. ros2_ws) and build:

```
cd ~/ros2_ws/src && git clone https://github.com/beamaginelidar/l3cam_fusion
colcon build --packages-select l3cam_fusion
```

## Nodes

### fusion_lidar_img

The fusion_lidar_img node fuses the LiDAR sensor with an image sensor (such as RGB or IR) in real time by projecting the points of the LiDAR point cloud to the image sensor.

The node first gets the fusion matrix from the matrix file, then gets the projection for each arriving point cloud, paints the projection on the image with the nearest timestamp to the point cloud timestamp, and publishes each image with the projection.

The name of the topic at which the fusion data gets published is `fusion` by default, but it can be named as desired by changing the [`fusion_topic` parameter](#fusion_lidar_img-parameters). This is because it is possible to show the fusion for different sensors by launching the same node with different parameters regarding different sensors of the same L3Cam.

To run this node you must first set the [node parameters](#fusion_lidar_img-parameters). You can set the parameters and run the node in different ways:

- By editing the `fusion_lidar_img_launch.xml` file arguments and launching it:

```
ros2 launch l3cam_fusion fusion_lidar_img_launch.xml
```

- By launching the `fusion_lidar_img_launch.xml` and passing the arguments via command line:

```
ros2 launch l3cam_fusion fusion_lidar_img_launch.xml "<PARAMETER_1>:=<VALUE_1>" "<PARAMETER_2>:=<VALUE_2>" ... "<PARAMETER_N>:=<VALUE_N>"
```

- By editing the `fusion_lidar_img.yaml` file and running the node:

```
ros2 run l3cam_fusion fusion_lidar_img --ros-args --params-file src/l3cam_fusion/fusion_lidar_img.yaml
```

## Parameters

### fusion_lidar_img parameters

| Parameter                | Type   | Description                                                                                                                                              |
| ------------------------ | ------ | -------------------------------------------------------------------------------------------------------------------------------------------------------- |
| mat_file                 | string | Route to the matrix file.                                                                                                                                |
| sensor_name              | string | Name of the image sensor. Check the sensor names in the matrix file.                                                                                     |
| print_matrix             | bool   | Bool to choose if you want to see the fusion matrix printed in the terminal.                                                                             |
| lidar_topic              | string | The name of the topic to subscribe from where to get the point clouds.                                                                                   |
| img_topic                | string | The name of the topic to subscribe from where to get the images.                                                                                         |
| fusion_topic             | string | The name of the topic to publish the fusion.                                                                                                             |
| pointcloud_color         | int    | Point cloud color visualization type. See the [visualization options](#pointcloud_color-options).                                                        |
| color_value_min          | int    | The minimum value of distance (in mm) at which Rainbow colors repeat and of values at which Intensity color shows, depending on the pointcloud_color value. |
| color_value_max          | int    | The maximum value of distance (in mm) at which Rainbow colors repeat and of values at which Intensity color shows, depending on the pointcloud_color value. |
| projection_points_radius | int    | Radius (in pixels) of the points projected                                                                                                                           |

#### Real time changeable parameters 

The following parameters can be changed while the node is running in real time:

| Parameter                | Type | Default | Range       |
| ------------------------ | ---- | ------- | ----------- |
| pointcloud_color         | int  | 3       | [0, 4]      |
| color_value_min          | int  | 0       | [0, 200000] |
| color_value_max          | int  | 10000   | [0, 200000] |
| projection_points_radius | int  | 3       | [0, 15]     |

They can be changed with dynamic reconfigure or with command line:

```
ros2 param set /fusion_lidar_img <PARAMETER> <VALUE>
```

#### pointcloud_color options

(See `visualizationTypes`)

- WHITE = 0
- INTENSITY = 1
- RGB = 2
- RAINBOW = 3
- RAINBOW_Z = 4

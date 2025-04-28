# ROS2 workspace for localization packages Test

## Production Setup Instructions

docker build -t my_image .devcontainer

1. First clone the repo into a suitable working directory

1. Build the image:

    ```bash
    docker build -t systems_ws .devcontainer
	```

1. Run the image:

    ```bash
    docker run -it \
    -u rosdev \
    --network=host \
    --ipc=host \
    -v $PWD:/home/rosdev/ros2_ws \
    -v /dev:/dev \
    --device-cgroup-rule='b *:* rwm' \
    --device-cgroup-rule='c *:* rwm'  \
    systems_ws
	```

1. Running individual node

    ```bash
    // Launch both the camera, imu, kinematics and object tracker
    ros2 launch mpu9250driver mpu9250driver_launch.py 

    ros2 run hardware_interfaces ultrasonic
    ros2 run hardware_interfaces manipulator
    ros2 run hardware_interfaces speaker

    ros2 run controller controller
    
    
    // Single executables
    ros2 run kinematics kinematics_node
    ros2 run object_tracker object_tracker_node
    ros2 run usb_cam usb_cam_node_exe
	```


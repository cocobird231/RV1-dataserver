*`Established: 2024/04/29`* *`Updated: 2024/04/29`*

## About The Project
The ROS2 topic message collection for the robot vehicle ver. 1 project.

## Getting Started

### Prerequisites
- ROS2 `Foxy` or later (`Humble` recommended)
    Install ROS2 from official website: [ROS2 official website](https://docs.ros.org/en/humble/Installation.html) or simply run the following command to automatically install ROS2:
    ```bash
    curl -fsSL ftp://61.220.23.239/scripts/install-ros2.sh | bash
    ```
    **NOTE:** The script only supports `Foxy` and `Humble` versions depending on the Ubuntu version.
    **NOTE:** The script will create a new workspace at `~/ros2_ws`.
    **NOTE:** The script will create an alias `humble` or `foxy` for global ROS2 environment setup (e.g. `source /opt/ros/<$ROS_DISTRO>/setup.bash`) depending on the ROS2 version.
- [vehicle_interfaces](https://github.com/cocobird231/RV1-vehicle_interfaces.git)
- libopencv-dev
- python3-opencv
- nlohmann-json3-dev

The required packages are listed in the `requirements_apt.txt` file. Install the required packages by running the following command:
```bash
xargs sudo apt install -y < requirements_apt.txt
```
**NOTE:** The required packages will be installed automatically while installing the package using the (`vcu-installer`)[https://github.com/cocobird231/RV1-vcu-install.git].


### Installation
There are two ways to install the package: manually or using the `vcu-installer`. 

#### Install Manually
1. Check if `vehicle_interfaces` package is installed. If not, install the package by following the instructions in the [vehicle_interfaces](https://github.com/cocobird231/RV1-vehicle_interfaces.git).
2. Clone the repository under `~/ros2_ws/src` and rename it to `cpp_dataserver3`:
    ```bash
    git clone https://github.com/cocobird231/RV1-dataserver.git cpp_dataserver3
    ```
3. Change the directory to the `~/ros2_ws` workspace and build the package:
    ```bash
    # Change directory to workspace.
    cd ~/ros2_ws

    # Source the local environment.
    . install/setup.bash

    # Build the package.
    colcon build --symlink-install --packages-select cpp_dataserver3
    ```
    **NOTE:** The package is installed in the local workspace.


#### Install Using `vcu-installer`
1. Run the installer and press `Scan` button under Package Management window. If the installer not installed, install the installer by following the instructions in the [`vcu-installer`](https://github.com/cocobird231/RV1-vcu-install.git).

2. Checked the `Data Server 3` checkbox under package list, right-click to modify the internet setting, then press the `Install` button to install the package.

3. The installer will create the start-up script for the package under `/etc/xdg/autostart` directory. The package will be started automatically after the system boot-up.


## Usage
The package contains two executables: `sub` and `control`. The `sub` executable is used to run the main service, while the `control` executable is used to control the service.

### Run the Main Service
1. Source the local workspace:
    ```bash
    source ~/ros2_ws/install/setup.bash
    ```

2. Run the main service:
    - Using the `launch`:
        ```bash
        ros2 launch cpp_dataserver3 launch.py
        ```
        **NOTE:** The launch file parsed the `common.yaml` file to set the parameters. The `common.yaml` file is located in the `cpp_dataserver3/launch` directory.
        **NOTE:** The `common.yaml` file default the namespace to `V0`.

    - Using the `run`:
        ```bash
        ros2 run cpp_dataserver3 sub
        ```

### Control the Service
1. Source the local workspace:
    ```bash
    source ~/ros2_ws/install/setup.bash
    ```

2. Run the control service:
    ```bash
    ros2 run cpp_dataserver3 control
    ```
    **NOTE:** If the main service is using the namespace (e.g. `V0`), the control service should use the same namespace to control the main service:
    ```bash
    ros2 run cpp_dataserver3 control --ros-args -r __ns:=/V0
    ```

## Description

### Control Service
The control executable demonstrates the control of the data server using `DataServer.srv` under vehicle_interfaces package. The `DataServer.srv` contains the following items:
```.srv
# Request field
DataServerStatus request # Send DataServerStatus to data server.

# Response field
bool response # Whether the service is successfully executed.

string reason # # If response flase, describes the reason.

DataServerStatus status # Response current data server status.
```

The `DataServerStatus` message can be used to describe the control signal of data server, or used to describe the current status of data server. The `DataServerStatus` message contains the following items:
```.msg
# The action of the data server. E.g. set timer, period or do nothing. See DataServerStatus.msg for more details.
uint8 server_action

# The status of scanning timer. Set to TIMER_STATUS_XXX.
uint8 server_scan_timer_status

# The period of scanning in _ms. Server will ignore this if value <= 0.
float64 server_scan_period_ms

# The status of sampling timer. Set to TIMER_STATUS_XXX.
uint8 server_sample_timer_status

# The period of sampling in _ms. Server will ignore this if value <= 0.
float64 server_sample_period_ms

# The status of dumping timer. Set to TIMER_STATUS_XXX.
uint8 server_dump_timer_status

# The period of dumping in _ms. Server will ignore this if value <= 0.
float64 server_dump_period_ms

# The status of countdown timer. Set to TIMER_STATUS_XXX.
uint8 server_countdown_timer_status

# The period of countdown in _ms. Server will ignore this if value <= 0.
float64 server_countdown_period_ms
```

### `common.yaml` File
The `common.yaml` file is used to set the parameters for the main service. The parameters are listed below:
- `topic_monitor`:
    - `msg_filter`: (vector<string>) The message type filter list.
    - `scan_period_ms`: (double) Scan the topic list in every period (ms).
        Timer call function to scan the whole topic list, and create new subscription nodes for the new topic.
- `msg_record`:
    - `sample_period_ms`: (double) The period of sample timer (ms).
        Timer call function to grab the latest message from each subscription node and store it in the buffer.
    - `dump_period_s`: (double) The period of dump timer (s).
        Timer call function to dump the message buffer to the JSON file.
    - `countdown_duration_s`: (double) The duration of countdown timer (s).
        Timer call function to stop recording the message and dump the rest of the message buffer to the JSON file.
    - `img_threads`: (int) Number of threads for image SaveQueue.
        The SaveQueue will create the threads for image saving process.
    - `gnd_threads` (not used): (int) Number of threads for ground truth SaveQueue.
        The SaveQueue will create the threads for ground truth saving process.
    - `dump_path`: (string) The path to store the record file.
        The JSON file will be stored under `<dump_path>/json`, and the specific `SaveQueueSubNode` will store data under `<dump_path>/<sub_node_name>`. E.g. The subscription node `_V0_zed_rgb_0_subnode` will store the image data under `<dump_path>/_V0_zed_rgb_0_subnode/`.
    - `enable_control`: (bool) Enable the control service.
        If true, the service will be idle until the control service sends the command to start the recording. Otherwise, the service will start recording immediately after the service is started.
- `service_prop`:
    - `serviceName`: (string) The service name.
        The service name revealed on the ROS2 network.
- `generic_prop`:
    - `namespace`: (string) The namespace of the node.
        The namespace is used to separate the services.
    - `nodeName`: (string) The node name.
        The node name revealed on the ROS2 network.
    - `id`: (int) The node id.
        The id of the node.

### Current Support Message Type
- `vehicle_interfaces/msg/Chassis`: chassis motor information could be published by `ControlServer` or other controller node.
- `vehicle_interfaces/msg/Distance`: range sensor information published by `UltraSoundPublisher` under `py_ultrasound` package.
- `vehicle_interfaces/msg/Environment`: environment information published by `SensePublisher` under `py_sense` package.
- `vehicle_interfaces/msg/GPS`: GPS information published by `GPSPublisher` under `py_gps` package.
- `vehicle_interfaces/msg/IDTable`: not used.
- `vehicle_interfaces/msg/Image`: image information could be published by `RGBImagePublisher` under `cpp_webcam` package or `ZEDPublisher` under `cpp_zedcam` package.
- `vehicle_interfaces/msg/IMU`: IMU information published by `SensePublisher` under `py_sense` package.
- `vehicle_interfaces/msg/MillitBrakeMotor`: not used.
- `vehicle_interfaces/msg/MillitPowerMotor`: not used.
- `vehicle_interfaces/msg/MotorAxle`: axle motor information published by `ChassisMotorPublisher` under `py_chassis` package.
- `vehicle_interfaces/msg/MotorSteering`: steering motor information by `ChassisSteeringPublisher` under `py_chassis` package.
- `vehicle_interfaces/msg/QosUpdate`: QoS update information published by `QoSServer`.
- `vehicle_interfaces/msg/SteeringWheel`: steering wheel information could be published by `ControlServer` or other controller node.
- `vehicle_interfaces/msg/UPS`: not used.
- `vehicle_interfaces/msg/WheelState`: not used.

### Custom Message Type Support
The data server aims to record the ROS2 topic message into JSON file. For those large scale data structure like image, point cloud, etc., the data server supports the custom save queue method, allow the user to seperate the data into JSON record file and the specific folder. For instance, the image data will be stored in the specific folder while JSON record file will store the image path and other information.

The custom message type could be easily added by modifying several steps:
1. Add the message type to the `msg_filter` list in the `common.yaml` file.

2. In `record_msg.h`, specialize the make function for the custom message type, there are three scenarios could be considered:

    1. The subscribe ROS2 message type can be directly stored in the record buffer. E.g. simple data structure like imu, gps, distance, etc..
        In this case, user does not need to modify the make function, the data server will automatically store the message in the record buffer.

    2. The subscribe ROS2 message type need to be converted to the custom record structure then stored in the buffer. E.g. modify the subscribed message then store the modified message in the buffer.
        In this case, user need to specialized the `make_record_msg_2` function. The tparam `msgT` is the subscribed ROS2 message type, the tparam `rmsgT` is the custom record structure to be recored into JSON file. The function return the shared pointer of the `RecordMsg<rmsgT>`.

    3. The subscribe ROS2 message data need to be stored in the specific folder using `SaveQueue`. E.g. the image message, we convert the image message to the image record structure, then push the image data to the `SaveQueue` for saving the image data in the specific folder.
        In this case, user need to specialized the `make_record_msg_3` function. The tparam `msgT` is the subscribed ROS2 message type, the tparam `rmsgT` is the custom record structure to be recored into JSON file, the tparam `smsgT` is the custom save queue structure. The function return the shared pointer of the `RecordMsg<rmsgT>`.

3. For the `scenario 3`, user need to specialized the `_saveCbFunc` function in `save_queue.h` file to implement the saving method. The `_saveCbFunc` function arguments takes the input `PairQue<smsgT>`. The `PairQue` contains the file name and data pairs. The file name should be the data saving path.

4. In `dump_json.h`, user need to specialized the `RecordMsgToJSON` function to dump the custom record structure into JSON file. The `RecordMsgToJSON` function arguments takes the reference of the JSON object `json`, and a shared pointer of `RecordMsg<rmsgT>`. Append the custom record structure to `json`.

5. In `scannode.h`, append the new custom message name under `line 235` for `ScanNode` to support the message. At `line 278`, for `scenario 1`, add `SubNode<msgT>` shared pointer to `this->subNodes_`; for `scenario 2`, add `SubNode2<msgT, rmsgT>` shared pointer to `this->subNodes_`; for `scenario 3`, add `SaveQueueSubNode<msgT, rmsgT, smsgT>` shared pointer to `this->subNodes_`.
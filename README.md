# wrpps_ros

ROS driver package for Wide-range Precise Proximity Sensor (WrPPS) board.  
Details of WrPPS are described in our paper: https://ieeexplore.ieee.org/document/9144379

Japanese papers:
- https://www.jstage.jst.go.jp/article/jsmermd/2019/0/2019_2P1-H08/_article/-char/ja/
- https://www.jstage.jst.go.jp/article/jsmermd/2020/0/2020_2A2-K15/_article/-char/ja/

## Installation

Build this package with [FA-I-sensor](https://github.com/RoboticMaterials/FA-I-sensor.git) in your catkin workspace.

### Using Arduino to read WrPPS Single Board

1. [Setup Arduino IDE with rosserial](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup)
   - Before running `make_libraries.py`, do not forget to source the workspace including `wrpps_ros`
2. Search and install Adafruit_VL53L0X on library manager of Arduino IDE
3. Prepare an Arduino (e.g., Arduino Nano Every)
4. Connect WrPPS Single Board to your Arduino (Vcc (3.3V), GND, SCL, and SDA)
5. Upload [wrpps_single_board_driver.ino](arduino/wrpps_single_board_driver/wrpps_single_board_driver.ino) to your Arduino

## Usage

### Using Arduino to read WrPPS Single Board

```bash
roslaunch wrpps_ros wrpps_single_board.launch port:=/dev/ttyACM0 sensor_name:=wrpps_single_board  # port depends on your environment
```

#### Publishing topics

- `$(arg sensor_name)/driver/output/proximity_intensity` (`force_proximity_ros/ProximityStamped`)

  Intensity sensor value.

- `$(arg sensor_name)/driver/output/range_tof` (`sensor_msgs/Range`)

  ToF sensor value.

- `$(arg sensor_name)/intensity_model_acquisition/output/range_intensity` (`sensor_msgs/Range`)

  Range value converted from the intensity sensor. To get this value, call `set_init_intensity` service first.

- `$(arg sensor_name)/intensity_model_acquisition/output/range_combined` (`sensor_msgs/Range`)

  Range value generated with the combination of the intensity-based range value and the ToF range value (the close-range value comes from the intensity value and the long-range value comes from the ToF value).

#### Services

- `$(arg sensor_name)/intensity_model_acquisition/set_init_intensity` (`std_srvs/Empty`)

  Get initial value of the intensity sensor. Call this service when there is no object in front of the board. After calling this service, calibration of the intensity sensor with the ToF sensor starts. Once the calibration is completed, the range value converted from the intensity sensor becomes meaningful.

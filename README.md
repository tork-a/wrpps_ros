# wrpps_ros

ROS driver package for Wide-range Precise Proximity Sensor (WrPPS) board.  

The feature of `wrpps_ros` is that by combining a ToF range sensor and a proximity sensor, it is possible to estimate the proximity distance in a nearby region outside the range of the ToF sensor.

Details of WrPPS are described in a paper: https://ieeexplore.ieee.org/document/9144379

Japanese papers:
- https://www.jstage.jst.go.jp/article/jsmermd/2019/0/2019_2P1-H08/_article/-char/ja/
- https://www.jstage.jst.go.jp/article/jsmermd/2020/0/2020_2A2-K15/_article/-char/ja/


## WrPPS Single Board

### Hardware

WrPPS Single Board has 2 type of proximity sensors, a ToF range sensor and a proximity and amibent light sensor.

![WrPPS Single Board v1.1 Dimensions](doc/images/wrpps-single-board_dimensions_small.png)

- Dimensions
  - Width: 18 [mm]
  - Length: 18 [mm]
  - Height: 4.7 [mm]
- Weight: 1.5 [g]
- Sensors
  - Time-of-Flight (ToF) Ranging Sensor
    - VL53L0X - STMicroelectronics
      - https://www.st.com/ja/imaging-and-photonics-solutions/vl53l0x.html
      - Range: ≦ 2.0 [m]
  - Proximity and Ambient Light Sensor
    - VCNL4040 - VISHAY
      - https://www.vishay.com/ja/product/84274/
      - Range: ≦ 0.2 [m]
- Connector Socket
  - SH Connector
    - 4 pin
    - SM04B-SRSS-TB(LF)(SN) - JST
      - https://www.jst-mfg.com/product/index.php?series=231


### Connector Pin Asignment

The pin assignment follows the order 1-2-3-4 from the right side, as viewed in the image above, using either ● mark as a reference.

SH Connector<br> Pin No. | Arduino<br>UNO R4 MINIMA | Arduino<br>nano Every
:---: | :---: | :---:
1 | GND | GND
2 | 3.3V | 3.3V
3 | SDA | A4 (SDA)
4 | SCL | A5 (SCL)


### Arduino Boards

The following models of the Arduino series have been confirmed to work.

- Arduino UNO R4 MINIMA
- Arduino nano Every

<br>


## System Requirement

- Ubuntu 20.04
- ROS Noetic

<br>


## Installation

The installation outlines are shown as followings.

1. [Set up Arduino IDE](#set-up-arduino-ide)
2. [Run Arduino Post Install Script](#run-arduino-post-install-script)
3. [Clone and Build wrpps_ros and FA-I-sensor packages](#clone-and-build-wrpps_ros-and-fa-i-sensor-packages)
4. [Make rosserial Arduino Libraries](#make-rosserial-arduino-libraries)
5. [Upload WrPPS Sketch to Arduino](#upload-wrpps-sketch-to-arduino)


### Set up Arduino IDE

Download Arduino IDE software.

- Arduino - Software - Downloads
  - https://www.arduino.cc/en/software/
    - [Linux AppImage 64 bits (X86-64)](https://downloads.arduino.cc/arduino-ide/arduino-ide_2.3.6_Linux_64bit.AppImage)

``` bash
cd ~/Download
chmod a+x arduino-ide_2.3.6_Linux_64bit.AppImage
./arduino-ide_2.3.6_Linux_64bit.AppImage
```

Search and install **Adafruit_VL53L0X** in **Library Manager** of Arduino IDE.

If you use a R4 type Arduino board, install Board Package.

- [Getting Started with Arduino UNO R4 Minima - ARDUINO DOCS](https://docs.arduino.cc/tutorials/uno-r4-minima/minima-getting-started/)
  - https://docs.arduino.cc/tutorials/uno-r4-minima/minima-getting-started/

Close Arduino IDE window or type Ctrl-C on the terminal to quit for now.


### Run Arduino Post Install Script

Download a script and execute it.

- [ArduinoCore-mbed / post_install.sh - GitHub](https://github.com/arduino/ArduinoCore-mbed/blob/main/post_install.sh)
  - https://github.com/arduino/ArduinoCore-mbed/blob/main/post_install.sh

``` bash
cd ~/Download
chmod a+x post_install.sh
sudo ./post_install.sh 
```


### Clone and Build wrpps_ros and FA-I-sensor packages

``` bash
source /opt/ros/noetic/setup.bash
mkdir -p ~/wrpps_ws/src
cd ~/wrpps_ws/src
git clone https://github.com/tork-a/wrpps_ros.git
git clone https://github.com/RoboticMaterials/FA-I-sensor.git
cd ~/wrpps_ws
rosdep install -r -y --from-paths src --ignore-src
catkin build
source ~/wrpps_ws/devel/setup.bash
```

#### ROS Environment Settings (Optional)

If you are constantly using `wrpps_ros`, it is useful to add preferences to the `.bashrc` file so that the path to the workspace containing `wrpps_ros` is set when the terminal is started.

The following is an example of opening and editing `.bashrc` file with the nano editor. But you can use any text editor.

``` bash
nano ~/.bashrc
```

Add the following lines to the end of the `.bashrc` file, save it with Ctrl-S, then use Ctrl-X to exit the nano editor.

``` bash
# ROS Environment
source /opt/ros/noetic/setup.bash
source ~/wrpps_ros/devel/setup.bash
env | grep ROS
```

This modified `.bashrc` file will set up environments for using ROS and the workspace that includes `wrpps_ros` every time you start the terminal from the next time.


### Make rosserial Arduino Libraries

``` bash
sudo apt update
sudo apt install ros-noetic-rosserial-arduino
sudo apt install ros-noetic-rosserial
source ~/wrpps_ws/devel/setup.bash
cd ~/Arduino/libraries
rm -rf ros_lib
rosrun rosserial_arduino make_libraries.py .
```


### Upload WrPPS Sketch to Arduino

Upload WrPPS Sketch [wrpps_single_board_driver.ino](arduino/wrpps_single_board_driver/wrpps_single_board_driver.ino) to your Arduino.
The Sketch is included in `arduino / wrpps_sigle_board_driver` folder of this wrpps_ros package.

``` bash
source ~/wrpps_ws/devel/setup.bash
cd ~/Download
./arduino-ide_2.3.6_Linux_64bit.AppImage
```

1. Connect WrPPS Single Board to your Arduino (Vcc (3.3V), GND, SCL, and SDA)
2. Connect the Arduino to your PC
3. Choose your **Arduino** in the menu **Tools > Board:**
4. Choose a **Port** that your Arduino is connected in the menu **Tools > Port:**
5. Upload **wrpps_single_board_driver.ino** to your Arduino

Quit Arduino IDE.

<br>


## Usage

### Using Arduino to read WrPPS Single Board

To start sensing and publishing ROS topics, run the following command.

```bash
source ~/wrpps_ws/devel/setup.bash
roslaunch wrpps_ros wrpps_single_board.launch
```

If the serial port for your Arduino is not the default value: `/dev/ttyACM0`, run it with `port` launch option.

```bash
roslaunch wrpps_ros wrpps_single_board.launch port:=/dev/ttyUSB0
```

#### Launch Options of `wrpps_single_board.launch`

- sensor_name
  - default="wrpps_single_board
    - Sensor name of 
- rosserial_respawn
  - default="false"
    - rosserial respawn enable (true) or disable (false)
- port" 
  - default="/dev/ttyACM0"
    - Serial port for Arduino
- baud
  - default="115200"
    - Baud rate for the serial communication
- intensity_frame_id
  - default="$(arg sensor_name)_intensity_frame"
    - Frame ID for the intensity sensor frame
- tof_frame_id
  - default="$(arg sensor_name)_tof_frame"
    - Frame ID for the ToF sensor frame
- rate
  - default="40"
    - Rate of sensing [Hz]
<!--
- tof_field_of_view
  - default="0.44"
- tof_min_range
  - default="0.03"
- tof_max_range
  - default="2.0"
-->

#### Publishing Topics

- `$(arg sensor_name)/driver/output/proximity_intensity` (`force_proximity_ros/ProximityStamped`)
  - Intensity sensor value.
- `$(arg sensor_name)/driver/output/range_tof` (`sensor_msgs/Range`)
  - ToF sensor value.
- `$(arg sensor_name)/intensity_model_acquisition/output/range_intensity` (`sensor_msgs/Range`)
  - Range value converted from the intensity sensor. To get this value, call `set_init_intensity` service first.
- `$(arg sensor_name)/intensity_model_acquisition/output/range_combined` (`sensor_msgs/Range`)
  - Range value generated with the combination of the intensity-based range value and the ToF range value (the close-range value comes from the intensity value and the long-range value comes from the ToF value).

#### ROS Topic Examples

**Terminal-1**
``` bash
source ~/wrpps_ws/devel/setup.bash
roslaunch wrpps_ros wrpps_single_board.launch
```

**Terminal-2**
``` bash
source ~/wrpps_ws/devel/setup.bash
rostopic echo /wrpps_single_board/driver/output/range_tof
```

``` bash
$ rostopic echo /wrpps_single_board/driver/output/range_tof
header: 
  seq: 1015
  stamp: 
    secs: 1754966728
    nsecs: 463486433
  frame_id: "wrpps_single_board_tof_frame"
radiation_type: 1
field_of_view: 0.4399999976158142
min_range: 0.029999999329447746
max_range: 2.0
range: 8.190999984741211
---

:
:
:

---
header: 
  seq: 1223
  stamp: 
    secs: 1754966733
    nsecs: 501786708
  frame_id: "wrpps_single_board_tof_frame"
radiation_type: 1
field_of_view: 0.4399999976158142
min_range: 0.029999999329447746
max_range: 2.0
range: 0.07100000232458115
---
header: 
  seq: 1224
  stamp: 
    secs: 1754966733
    nsecs: 526211977
  frame_id: "wrpps_single_board_tof_frame"
radiation_type: 1
field_of_view: 0.4399999976158142
min_range: 0.029999999329447746
max_range: 2.0
range: 0.07000000029802322
---
header: 
  seq: 1225
  stamp: 
    secs: 1754966733
    nsecs: 550466775
  frame_id: "wrpps_single_board_tof_frame"
radiation_type: 1
field_of_view: 0.4399999976158142
min_range: 0.029999999329447746
max_range: 2.0
range: 0.0689999982714653
---

:
:
:
```

Type Ctrl-C to quit.


**Terminal-2**
``` bash
rostopic echo /wrpps_single_board/driver/output/proximity_intetof
```

``` bash
$ rostopic echo /wrpps_single_board/driver/output/proximity_intensity
header: 
  seq: 266
  stamp: 
    secs: 1754967229
    nsecs: 459060668
  frame_id: "wrpps_single_board_intensity_frame"
proximity: 
  proximity: 5
  average: 4
  fa2: -1
  fa2derivative: 0
  mode: "0"
---

:
:
:

---
header: 
  seq: 440
  stamp: 
    secs: 1754967233
    nsecs: 678566694
  frame_id: "wrpps_single_board_intensity_frame"
proximity: 
  proximity: 361
  average: 397
  fa2: 36
  fa2derivative: 32
  mode: "0"
---
header: 
  seq: 441
  stamp: 
    secs: 1754967233
    nsecs: 702883005
  frame_id: "wrpps_single_board_intensity_frame"
proximity: 
  proximity: 316
  average: 386
  fa2: 70
  fa2derivative: 34
  mode: "0"
---
header: 
  seq: 442
  stamp: 
    secs: 1754967233
    nsecs: 727200746
  frame_id: "wrpps_single_board_intensity_frame"
proximity: 
  proximity: 266
  average: 365
  fa2: 99
  fa2derivative: 29
  mode: "0"
---
header: 
  seq: 443
  stamp: 
    secs: 1754967233
    nsecs: 750994205
  frame_id: "wrpps_single_board_intensity_frame"
proximity: 
  proximity: 220
  average: 335
  fa2: 115
  fa2derivative: 16
  mode: "0"
---

:
:
:
```


### ROS Services of `wrpps_ros`

#### Set the Initial Intensity Value for `Range` Conversion

- `$(arg sensor_name)/intensity_model_acquisition/set_init_intensity` (`std_srvs/Empty`)
  - Get initial value of the intensity sensor. Call this service when there is no object in front of the board. After calling this service, calibration of the intensity sensor with the ToF sensor starts. Once the calibration is completed, the range value converted from the intensity sensor becomes meaningful.

An Example is shown below to set the initial Intensity value for `Range` conversion from the intensicty sensor data.

**Terminal-1**
``` bash
source ~/wrpps_ws/devel/setup.bash
roslaunch wrpps_ros wrpps_single_board.launch
```

**Terminal-2**
``` bash
source ~/wrpps_ws/devel/setup.bash
$ rosservice call /wrpps_single_board/intensity_model_acquisition/set_init_intensity 
success: True
message: ''
$ rostopic echo /wrpps_single_board/intensity_model_acquisition/output/range_combined 
header: 
  seq: 1
  stamp: 
    secs: 1754978543
    nsecs: 564466476
  frame_id: "wrpps_single_board_intensity_frame"
radiation_type: 1
field_of_view: 0.4399999976158142
min_range: 0.0
max_range: 2.0
range: nan
---

:
:
:

---
header: 
  seq: 336
  stamp: 
    secs: 1754978551
    nsecs: 683211565
  frame_id: "wrpps_single_board_intensity_frame"
radiation_type: 1
field_of_view: 0.4399999976158142
min_range: 0.0
max_range: 2.0
range: 0.010047607123851776
---
header: 
  seq: 337
  stamp: 
    secs: 1754978551
    nsecs: 707510232
  frame_id: "wrpps_single_board_intensity_frame"
radiation_type: 1
field_of_view: 0.4399999976158142
min_range: 0.0
max_range: 2.0
range: 0.009967952966690063
---
header: 
  seq: 338
  stamp: 
    secs: 1754978551
    nsecs: 732238531
  frame_id: "wrpps_single_board_intensity_frame"
radiation_type: 1
field_of_view: 0.4399999976158142
min_range: 0.0
max_range: 2.0
range: 0.009987088851630688
---
header: 
  seq: 339
  stamp: 
    secs: 1754978551
    nsecs: 756275892
  frame_id: "wrpps_single_board_intensity_frame"
radiation_type: 1
field_of_view: 0.4399999976158142
min_range: 0.0
max_range: 2.0
range: 0.009863249026238918
---

:
:
:
```

#### Enable Sensors

Wrap sensor enabling topic interfaces with service interfaces.
Users should use service interfaces for their application.

Message published to the topic interface is sometimes lost in rosserial communication.
The service interface can detect this and resend the message.
Ideally, native service server of rosserial_arduino should work,
but this causes "Lost sync with device, restarting..." when it is called repeatedly

- `$(arg sensor_name)/driver/enable_tof` (`std_srvs/SetBool`)
  - Enable the ToF sensor
- `$(arg sensor_name)/driver/enable_intensity` (`std_srvs/SetBool`)
  - Enable the intensity sensor

Examples are shown below to enable the ToF sensor or the intensity sensor.

**Terminal-1**
``` bash
source ~/wrpps_ws/devel/setup.bash
roslaunch wrpps_ros wrpps_single_board.launch
```

**Terminal-2** : Enable the ToF sensor
``` bash
$ source ~/wrpps_ws/devel/setup.bash
$ rosservice call /wrpps_single_board/driver/enable_tof true
success: True
message: ''
$
```

**Terminal-2** : Enable the intensity sensor
``` bash
$ source ~/wrpps_ws/devel/setup.bash
$ rosservice call /wrpps_single_board/driver/enable_intensity true
success: True
message: ''
$
```

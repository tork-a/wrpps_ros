#include <math.h>
#include <stdio.h>
#include <Wire.h>
#include <ros.h>
#include <force_proximity_ros/Proximity.h>
#include <force_proximity_ros/ProximityStamped.h>
#include <sensor_msgs/Range.h>
#include "Adafruit_VL53L0X.h"  // Search and install Adafruit_VL53L0X on library manager of Arduino IDE
// https://github.com/adafruit/Adafruit_VL53L0X

// TODO: Use https://github.com/sparkfun/SparkFun_VCNL4040_Arduino_Library
//       Naive usage of that library seems to make starting/stopping sensor slow
class VCNL4040
{
public:
  // Constants
  enum
  {
    // 7-bit unshifted I2C address of VCNL4040
    VCNL4040_ADDR = 0x60,
    PS_CONF1 = 0x03,
    PS_CONF3 = 0x04,
    PS_DATA_L = 0x08,
  };
  // Sensitivity of touch/release detection, values closer to zero increase sensitivity
  signed int sensitivity_;
  // exponential average weight parameter / cut-off frequency for high-pass filter
  float ea_;
  // low-pass filtered intensity reading
  unsigned int average_value_;
  bool is_average_init_;
  // FA-II value;
  signed int fa2_;

  VCNL4040()
    : sensitivity_(1000)
    , ea_(0.3)
    , average_value_(0)
    , is_average_init_(false)
    , fa2_(0)
  {
  }

  ~VCNL4040()
  {
  }

  void init()
  {
    //Set the options for PS_CONF3 and PS_MS bytes
    // uint8_t ms = 0b00000000;  // IR LED current to 50mA
    // uint8_t ms = 0b00000001;  // IR LED current to 75mA
    // uint8_t ms = 0b00000010;  // IR LED current to 100mA
    uint8_t ms = 0b00000110;  // IR LED current to 180mA
    // uint8_t ms = 0b00000111;  // IR LED current to 200mA
    writeToCommandRegister(PS_CONF3, 0x00, ms);
  }

  void startSensing()
  {
    //Clear PS_SD to turn on proximity sensing
    //Integrate 8T, Clear PS_SD bit to begin reading
    //Set PS to 16-bit
    writeToCommandRegister(PS_CONF1, 0b00001110, 0b00001000); //Command register, low byte, high byte
  }

  void stopSensing()
  {
    //Set PS_SD to turn off proximity sensing
    //Set PS_SD bit to stop reading
    writeToCommandRegister(PS_CONF1, 0b00000001, 0b00000000); //Command register, low byte, high byte
  }

  //Reads a two byte value from a command register
  unsigned int readFromCommandRegister(byte commandCode)
  {
    Wire.beginTransmission(VCNL4040_ADDR);
    Wire.write(commandCode);
    Wire.endTransmission(false); //Send a restart command. Do not release bus.

    Wire.requestFrom(VCNL4040_ADDR, 2); //Command codes have two bytes stored in them

    unsigned int data = Wire.read();
    data |= Wire.read() << 8;

    return (data);
  }

  //Write a two byte value to a Command Register
  void writeToCommandRegister(byte commandCode, byte lowVal, byte highVal)
  {
    Wire.beginTransmission(VCNL4040_ADDR);
    Wire.write(commandCode);
    Wire.write(lowVal); //Low byte of command
    Wire.write(highVal); //High byte of command
    Wire.endTransmission(); //Release bus
  }

  unsigned int getRawProximity()
  {
    return (readFromCommandRegister(PS_DATA_L));
  }

  void getProximity(force_proximity_ros::Proximity* prox)
  {
    unsigned int raw = getRawProximity();
    prox->proximity = raw;
    if (!is_average_init_)
    {
      average_value_ = raw;
      is_average_init_ = true;
    }
    prox->average = average_value_;
    signed int fa2derivative = (signed int) average_value_ - raw - fa2_;
    prox->fa2derivative = fa2derivative;
    fa2_ = (signed int) average_value_ - raw;
    prox->fa2 = fa2_;
    if (fa2_ < -sensitivity_)
    {
      prox->mode = "T";
    }
    else if (fa2_ > sensitivity_)
    {
      prox->mode = "R";
    }
    else
    {
      prox->mode = "0";
    }
    average_value_ = ea_ * raw + (1 - ea_) * average_value_;
  }
};

/***** ROS *****/
ros::NodeHandle  nh;
force_proximity_ros::ProximityStamped intensity_msg;
sensor_msgs::Range tof_msg;
ros::Publisher intensity_pub("~output/proximity_intensity", &intensity_msg);
ros::Publisher tof_pub("~output/range_tof", &tof_msg);

/***** USER PARAMETERS *****/
unsigned long time;
unsigned long loop_duration;  // loop duration in ms

/***** GLOBAL VARIABLES *****/
VCNL4040 intensity_sensor;
Adafruit_VL53L0X tof_sensor;

void setup()
{
  nh.getHardware()->setBaud(115200);
  // cf. https://arduino.stackexchange.com/questions/296/how-high-of-a-baud-rate-can-i-go-without-errors
  //     http://wormfood.net/avrbaudcalc.php
  // I'm not sure why, but 500000/250000 sometimes causes "Mismatched protocol version" error on Arduino Nano Every
  nh.initNode();
  nh.advertise(intensity_pub);
  nh.advertise(tof_pub);
  while (!nh.connected())
  {
    nh.spinOnce();
  }

  char intensity_frame_id[128] = "intensity_frame";
  char* pp_intensity_frame_id[1];
  pp_intensity_frame_id[0] = intensity_frame_id;
  nh.getParam("~intensity_frame_id", pp_intensity_frame_id, 1);
  intensity_msg.header.frame_id = intensity_frame_id;

  char tof_frame_id[128] = "tof_frame";
  char* pp_tof_frame_id[1];
  pp_tof_frame_id[0] = tof_frame_id;
  nh.getParam("~tof_frame_id", pp_tof_frame_id, 1);
  tof_msg.header.frame_id =  tof_frame_id;

  tof_msg.field_of_view = 0.44;  // 25 degrees
  if (!nh.getParam("~tof_field_of_view", &(tof_msg.field_of_view), 1))
  {
    nh.logwarn("Perhaps you do not pass float to tof_field_of_view");
  }

  tof_msg.min_range = 0.03;
  if (!nh.getParam("~tof_min_range", &(tof_msg.min_range), 1))
  {
    nh.logwarn("Perhaps you do not pass float to tof_min_range");
  }

  tof_msg.max_range = 2.0;
  if (!nh.getParam("~tof_max_range", &(tof_msg.max_range), 1))
  {
    nh.logwarn("Perhaps you do not pass float to tof_max_range");
  }

  tof_msg.radiation_type = sensor_msgs::Range::INFRARED;

  int rate = -1;
  if (!nh.getParam("~rate", &rate, 1))
  {
    nh.logwarn("Perhaps you do not pass integer to rate");
  }
  if (rate <= 0)
  {
    loop_duration = 0;
  }
  else
  {
    loop_duration = 1000.0f / (float) rate;
  }

  Wire.begin();
  // Wire.setClock(400000);
  // Increasing I2C bus speed to 400kHz makes communication unstable on Arduino Nano Every

  intensity_sensor.init();
  intensity_sensor.startSensing();

  if (!tof_sensor.begin())
  {
    nh.logerror("Failed to boot VL53L0X");
    while (1);
  }
  if (!tof_sensor.setMeasurementTimingBudgetMicroSeconds(20000))
  {
    nh.logerror("Failed to set MeasurementTimingBudget in VL53L0X");
    while (1);
  }
  if (tof_sensor.setDeviceMode(VL53L0X_DEVICEMODE_CONTINUOUS_RANGING) != VL53L0X_ERROR_NONE)
  {
    nh.logerror("Failed to set device mode to continuous ranging in VL53L0X");
    while (1);
  }
  if (tof_sensor.startMeasurement() != VL53L0X_ERROR_NONE)
  {
    nh.logerror("Failed to start measurement in VL53L0X");
    while (1);
  }
}

void loop()
{
  time = millis();

  intensity_sensor.getProximity(&(intensity_msg.proximity));
  intensity_msg.header.stamp = nh.now();
  intensity_pub.publish(&intensity_msg);

  if (!tof_sensor.waitRangeComplete())
  {
    nh.logerror("Failed to wait completion of Range operation of VL53L0X");
    // TODO: Try to reset VL53L0X for recovering from momentary I2C error
  }
  else
  {
    VL53L0X_RangingMeasurementData_t tof_data;
    if (tof_sensor.getRangingMeasurement(&tof_data) != VL53L0X_ERROR_NONE)
    {
      nh.logerror("Failed to get measurement data of VL53L0X");
      // TODO: Try to reset VL53L0X for recovering from momentary I2C error
    }
    else
    {
      uint8_t status = tof_data.RangeStatus;
      if (status > 5)
      {
        char buf[64];
        snprintf(buf, 64, "RangeStatus of VL53L0X was %d, it is strange", status);
        nh.logerror(buf);
        // TODO: Try to reset VL53L0X for recovering from momentary I2C error
      }
      else
      {
        uint16_t range_mm = tof_data.RangeMilliMeter;
        if ((status != 4) && (range_mm != 0) && (range_mm != 8190))
        {
          tof_msg.range = (float)range_mm / 1000.0f;
          // We do not limit this value between tof_min_range and tof_max_range because
          // https://www.ros.org/reps/rep-0117.html#reference-implementation
          // accepts the case where the value is not between these limits
          // and we assume someone wants to know the value even in that case
        }
        else
        {
          // In our experience, measurement is erroneous when:
          // - RangeStatus is 4 (Phase Fail)
          // - RangeMilliMeter is 0 or 8190
          // Out of range detections are included in those cases, but cannot be distinguished.
          // So we publish NaN in all of those cases.
          // cf. https://www.ros.org/reps/rep-0117.html
          tof_msg.range = NAN;
        }
        tof_msg.header.stamp = nh.now();
        tof_pub.publish(&tof_msg);
      }
    }
  }

  while (millis() < time + loop_duration); // enforce constant loop time
  nh.spinOnce();
}

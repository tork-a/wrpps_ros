/*
 * Software License Agreement (BSD 3-Clause License)
 *
 * Copyright (c) 2024- JSK Robotics Laboratory, Shun Hasegawa
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <math.h>
#include <stdio.h>
#include <Wire.h>
#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt32.h>
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
  /*
  // Sensitivity of touch/release detection, values closer to zero increase sensitivity
  signed int sensitivity_;
  // exponential average weight parameter / cut-off frequency for high-pass filter
  float ea_;
  // low-pass filtered intensity reading
  unsigned int average_value_;
  bool is_average_init_;
  // FA-II value;
  signed int fa2_;
  */

  VCNL4040()
  /*
    : sensitivity_(1000)
    , ea_(0.3)
    , average_value_(0)
    , is_average_init_(false)
    , fa2_(0)
  */
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

  /*
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
  */
};

#define TOF_PERIOD_MIN 25  // rate: 40
#define TOF_PERIOD_MAX 200  // rate: 5

ros::NodeHandle nh;
std_msgs::UInt32 intensity_msg;
std_msgs::Float32 tof_msg;
ros::Publisher intensity_pub("~output/proximity_intensity/raw", &intensity_msg);
ros::Publisher tof_pub("~output/range_tof/raw", &tof_msg);

std_msgs::Empty got_en_msg;
ros::Publisher got_en_pub("~got_enabling_command", &got_en_msg);

bool enable_int_command;
void enableIntCb(const std_msgs::Bool& msg)
{
  enable_int_command = msg.data;
  got_en_pub.publish(&got_en_msg);
}
ros::Subscriber<std_msgs::Bool> enable_int_sub("~enable_intensity", &enableIntCb);
// Native srv server of rosserial_arduino causes "Lost sync with device, restarting..." when it is called repeatedly,
// so we instead use sub to get request and pub to return response.
// Multi request subs share single response pub to avoid "Lost sync with device, restarting..." by reducing pub number,
// so you should not publish to different request sub before getting response from sub you previously published to

bool enable_tof_command;
bool enable_tof_state;
void enableTofCb(const std_msgs::Bool& msg)
{
  enable_tof_command = msg.data;
  got_en_pub.publish(&got_en_msg);
}
ros::Subscriber<std_msgs::Bool> enable_tof_sub("~enable_tof", &enableTofCb);
// Native srv server of rosserial_arduino causes "Lost sync with device, restarting..." when it is called repeatedly,
// so we instead use sub to get request and pub to return response.
// Multi request subs share single response pub to avoid "Lost sync with device, restarting..." by reducing pub number,
// so you should not publish to different request sub before getting response from sub you previously published to

unsigned long start_tm;
VCNL4040 intensity_sensor;
Adafruit_VL53L0X tof_sensor;
uint16_t tof_period;  // inter-measurement period of ToF in ms

void setup()
{
  nh.getHardware()->setBaud(115200);
  // cf. https://arduino.stackexchange.com/questions/296/how-high-of-a-baud-rate-can-i-go-without-errors
  //     http://wormfood.net/avrbaudcalc.php
  // I'm not sure why, but 500000/250000 sometimes causes "Mismatched protocol version" error on Arduino Nano Every
  nh.initNode();
  nh.advertise(intensity_pub);
  nh.advertise(tof_pub);
  nh.advertise(got_en_pub);
  enable_int_command = true;
  nh.subscribe(enable_int_sub);
  enable_tof_command = true;
  nh.subscribe(enable_tof_sub);
  while (!nh.connected())
  {
    nh.spinOnce();
  }

  int rate = -1;
  if (!nh.getParam("~rate", &rate, 1))
  {
    nh.logwarn("Perhaps you do not pass integer to rate");
  }
  if (rate <= 0)
  {
    tof_period = 0;
  }
  else
  {
    tof_period = 1000.0f / (float) rate;
  }
  if (tof_period < TOF_PERIOD_MIN)
  {
    tof_period = TOF_PERIOD_MIN;
  }
  else if (tof_period > TOF_PERIOD_MAX)
  {
    tof_period = TOF_PERIOD_MAX;
  }

  Wire.begin();
  // Wire.setClock(400000);
  // Increasing I2C bus speed to 400kHz makes communication unstable on Arduino Nano Every

  intensity_sensor.init();
  intensity_sensor.stopSensing();

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
  tof_sensor.stopRangeContinuous();
  // Do not startRangeContinuous here to ensure intensity sensing starts just after ToF sensing
  // TODO: Fix stopRangeContinuous to notify error
  // TODO: Try to reset VL53L0X for recovering from momentary I2C error
  enable_tof_state = false;
  start_tm = millis();
}

void loop()
{
  if (enable_tof_state != enable_tof_command)
  {
    enable_tof_state = enable_tof_command;
    if (enable_tof_command)
    {
      if (!tof_sensor.startRangeContinuous(tof_period))
      {
        nh.logerror("Failed to start measurement in VL53L0X");
        enable_tof_state = false;
        // TODO: Try to reset VL53L0X for recovering from momentary I2C error
      }
      // startRangeContinuous starts continuous timed ranging, not single ranging nor continuous ranging.
      // Single ranging takes much time because it calls VL53L0X_StartMeasurement every time.
      // By using continuous timed ranging, we skip that function in each loop.
      // Also, we can do stable intensity sensing in inter-measurement period of VL53L0X.
      // On low-power device, intensity value becomes unstable when VL53L0X measurement is conducted at the same time.
      // Continuous ranging cannot control inter-measurement period, so we selected continuous timed ranging.
    }
    else
    {
      tof_sensor.stopRangeContinuous();
      // TODO: Fix stopRangeContinuous to notify error
      // TODO: Try to reset VL53L0X for recovering from momentary I2C error
    }
  }

  if (enable_tof_state)
  {
    if (!tof_sensor.waitRangeComplete())
    {
      nh.logerror("Failed to wait completion of Range operation of VL53L0X");
      // TODO: Try to reset VL53L0X for recovering from momentary I2C error
    }
    else
    {
      VL53L0X_RangingMeasurementData_t tof_data;
      if (tof_sensor.getRangingMeasurement(&tof_data) != VL53L0X_ERROR_NONE ||
          tof_sensor.clearInterruptMask() != VL53L0X_ERROR_NONE)
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
            tof_msg.data = (float)range_mm / 1000.0f;
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
            tof_msg.data = NAN;
          }
        }
      }
    }
  }
  else
  {
    while (millis() < start_tm + tof_period);  // Emulate sleep in waitRangeComplete
    tof_msg.data = NAN;
  }
  tof_pub.publish(&tof_msg);
  start_tm = millis();

  if (enable_int_command)
  {
    delayMicroseconds(500);
    // Interval between ToF and intensity sensing. Without this, intensity value becomes unstable
    intensity_sensor.startSensing();
    delay(5);
    // Interval between startSensing and getProximity. Without this, intensity value becomes 0
  }
  intensity_msg.data = intensity_sensor.getRawProximity();
  intensity_sensor.stopSensing();
  intensity_pub.publish(&intensity_msg);

  nh.spinOnce();
}

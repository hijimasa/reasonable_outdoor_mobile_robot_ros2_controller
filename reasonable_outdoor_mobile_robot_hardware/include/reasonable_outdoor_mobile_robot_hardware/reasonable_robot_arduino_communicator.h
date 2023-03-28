/**
* @file reasonable_robot_arduino_communicator.h
* @brief header file for omni_robot_arduino_communicator
* @author Masaaki Hijikata <hijikata@ir.utsunomiya-u.ac.jp>, Utsunomiya Univ.
* @date 20210218
* @details 
*/

#include <string>
#include <vector>

#ifndef __REASONABLE_ROBOT_ARDUINO_COMUNICATOR_H__
#define __REASONABLE_ROBOT_ARDUINO_COMUNICATOR_H__

#define REASONABLE_ROBOT_ARDUINO_DEFAULT_DEVICE "/dev/ttyUSB0"
#define REASONABLE_ROBOT_DRIVER_DEFAULT_FREQUENCY 10

class ReasonableRobotArduinoComunicator
{
public:
  static constexpr int32_t PULSE_PER_REV = 6400;
  
  ReasonableRobotArduinoComunicator(std::string device_name, int motor_num);
  ~ReasonableRobotArduinoComunicator();

  bool isConnected();
  bool writeRadps(std::vector<float>& radps);
  bool writeBrake();
  bool readRad(std::vector<float>& rad, std::vector<float>& radps, std::vector<float>& current);

private:
  int device_fd_;
  bool is_connected_;

  int motor_num_;
  std::vector<float> current_angle_rad_;
  std::vector<float> current_speed_radps_;
  std::vector<float> current_current_;
};

#endif /* __REASONABLE_ROBOT_ARDUINO_COMUNICATOR_H__ */

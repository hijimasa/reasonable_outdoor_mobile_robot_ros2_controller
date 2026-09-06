/**
* @file Reasonable_robot_arduino_communicator.c
* @brief source file for Reasonable_robot_arduino_communicator
* @author Masaaki Hijikata <hijikata@ir.utsunomiya-u.ac.jp>, Utsunomiya Univ.
* @date 20210218
* @details 
* 
*/

#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include <cmath>
#include <cfloat>

#include "reasonable_outdoor_mobile_robot_hardware/reasonable_robot_arduino_communicator.h"

#define BAUD_RATE    B9600
using SERIAL_WRITE = decltype(&write);
SERIAL_WRITE serial_write = write;
using SERIAL_READ = decltype(&read);
SERIAL_READ serial_read = read;

ReasonableRobotArduinoComunicator::ReasonableRobotArduinoComunicator(std::string device_name, int motor_num)
: is_connected_(true)
, motor_num_(motor_num)
, current_angle_rad_(motor_num)
, current_speed_radps_(motor_num)
, current_current_(motor_num)
{
  // serial device config
  struct termios oldtio, newtio;

  device_fd_ = open(device_name.c_str(), O_RDWR);
  if (device_fd_< 0) { // デバイスオープンに失敗
    is_connected_ = false;
  }

  ioctl(device_fd_, TCGETS, &oldtio);
  newtio = oldtio;
  // system default c i o l = 0x0cdb 0x0500 0x0005 0x8a3b
  newtio.c_cflag =  BAUD_RATE | CS8 | CLOCAL | CREAD;
  newtio.c_cflag &= ~(PARENB | PARODD);
  newtio.c_iflag =  IGNPAR;
  // Raw output. ONLCR rewrote every 0x0A in a command frame as 0x0D 0x0A, so
  // any speed whose low byte was 10 rpm reached the board one byte long with
  // a wrong checksum and was ignored.
  newtio.c_oflag =  0;
  newtio.c_lflag = 0x0000;
  ioctl(device_fd_, TCSETS, &newtio);

  sleep(1);

}

ReasonableRobotArduinoComunicator::~ReasonableRobotArduinoComunicator()
{
  if (device_fd_ >= 0)
  {
    close(device_fd_);
  }
}

bool
ReasonableRobotArduinoComunicator::isConnected()
{
  return is_connected_;
}

bool
ReasonableRobotArduinoComunicator::writeRadps(std::vector<float>& command_radps)
{
  if (command_radps.size() != static_cast<size_t>(motor_num_))
  {
    return false;
  }

  uint8_t *send_command;
  send_command = new uint8_t[motor_num_ * 2 + 2];
  send_command[0] = 85;
  send_command[motor_num_ * 2 + 1] = send_command[0];
  for (int i = 0; i < motor_num_; i++)
  {
    int32_t speed_rpm = static_cast<int32_t>(command_radps[i] / (M_PI * 2.0f) * 60.0);
    send_command[2*i + 1] = static_cast<uint8_t>((speed_rpm >> 8) & 0x000000ff);
    send_command[2*i + 2] = static_cast<uint8_t>(speed_rpm & 0x000000ff);
    send_command[motor_num_ * 2 + 1] += send_command[2*i + 1];
    send_command[motor_num_ * 2 + 1] += send_command[2*i + 2];
  }

  serial_write(device_fd_, send_command, motor_num_ * 2 + 2);

  delete[] send_command;

  return true;
}

bool
ReasonableRobotArduinoComunicator::writeBrake()
{
  uint8_t *send_command;
  send_command = new uint8_t[motor_num_ * 2 + 2];
  send_command[0] = 85;
  send_command[motor_num_ * 2 + 1] = send_command[0];
  for (int i = 1; i <= motor_num_ * 2; i++)
  {
    send_command[i] = 0;
    send_command[motor_num_ * 2 + 1] += send_command[i];
  }

  serial_write(device_fd_, send_command, motor_num_ * 2 + 2);

  delete[] send_command;

  return true;
}

bool
ReasonableRobotArduinoComunicator::readRad(std::vector<float>& response_rad, std::vector<float>& response_radps, std::vector<float>& response_current)
{
  if (response_rad.size() != static_cast<size_t>(motor_num_))
  {
    return false;
  }
  if (response_radps.size() != static_cast<size_t>(motor_num_))
  {
    return false;
  }
  if (response_current.size() != static_cast<size_t>(motor_num_))
  {
    return false;
  }

  int available_size = 0;
  int32_t retry_count = 0;
  while (available_size < static_cast<int32_t>(motor_num_) * 6 + 2)
  {
    ioctl(device_fd_, FIONREAD, &available_size);
    usleep(10000);

    retry_count++;
    if (retry_count > 30)
    {
      tcflush(device_fd_, TCIFLUSH);
      return false;
    }
  }

  // Take everything the board has sent and use the newest complete frame in
  // it: header 0x55, then 6 bytes per motor, then a checksum that is the sum
  // of all preceding bytes. Scanning for the header instead of assuming the
  // buffer starts on one means a lost byte or a late reply costs one cycle,
  // not every cycle until the buffer happens to empty.
  const int frame_len = motor_num_ * 6 + 2;
  uint8_t recv_buffer[256];
  int recv_size = serial_read(device_fd_, recv_buffer, sizeof(recv_buffer));
  if (recv_size < frame_len)
  {
    return false;
  }

  const uint8_t * frame = nullptr;
  for (int start = recv_size - frame_len; start >= 0; start--)
  {
    if (recv_buffer[start] != 85)
    {
      continue;
    }
    uint8_t checksum = 0;
    for (int i = 0; i < frame_len - 1; i++)
    {
      checksum += recv_buffer[start + i];
    }
    if (checksum == recv_buffer[start + frame_len - 1])
    {
      frame = &recv_buffer[start];
      break;
    }
  }
  if (frame == nullptr)
  {
    return false;
  }

  for (int i = 0; i < motor_num_; i++)
  {
    // The DDT motor reports its angle as 0..32767 for one revolution in the
    // low 15 bits; bit 15 is not part of the angle (on the robot it flickers
    // on a wheel that is standing still). This used to be read as a signed
    // 16-bit value over 65535, which halved every revolution and turned each
    // flicker into a pi-sized jump in the odometry.
    uint16_t raw_angle = (static_cast<uint16_t>(frame[i*6+1]) << 8) | frame[i*6+2];
    response_rad[i] = static_cast<float>(raw_angle & 0x7FFF) / 32768.0f * (M_PI * 2.0f);

    int16_t recv_data;
    recv_data = (static_cast<int16_t>(frame[i*6+3]) << 8) + frame[i*6+4];
    response_radps[i] = static_cast<float>(recv_data) * (M_PI * 2.0f) / 60.0f;

    recv_data = (static_cast<int16_t>(frame[i*6+5]) << 8) + frame[i*6+6];
    response_current[i] = static_cast<float>(recv_data);
  }

  return true;
}


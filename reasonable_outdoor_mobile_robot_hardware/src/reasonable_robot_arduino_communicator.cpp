/**
* @file Reasonable_robot_arduino_communicator.c
* @brief source file for Reasonable_robot_arduino_communicator
* @author Masaaki Hijikata <hijikata@ir.utsunomiya-u.ac.jp>, Utsunomiya Univ.
* @date 20210218
* @details 
* 
*/

#include <stdio.h>
#include <poll.h>
#include <unistd.h>
#include <time.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include <chrono>
#include <cmath>
#include <cfloat>

#include "reasonable_outdoor_mobile_robot_hardware/reasonable_robot_arduino_communicator.h"

// Must match Serial.begin() in the DDTMotorController sketch. 500000 divides
// the board's 16 MHz exactly (UBRR 3 with U2X, 0.00% error) and is a standard
// Linux constant, so neither end is approximating; 115200 would be +2.12% on
// that AVR. It takes the 6 byte command and 14 byte reply from 20.8 ms on the
// wire to 0.40 ms.
//
// Both ends have to change together. Flashing one without rebuilding the other
// leaves the link silent, which reads as "No valid status frame from the motor
// board" here and as a command timeout on the board.
#define BAUD_RATE    B500000
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
    // The board takes whole rpm, so round to nearest rather than truncating:
    // that alone halves the deadband, to half an rpm, 0.005 m/s at the tyre.
    //
    // Do NOT floor this at +-1 rpm for a non-zero command. That was tried on
    // 2026-09-07 and had to come out on 2026-09-12: the board's feedback is
    // also whole rpm, and its PID is an accumulator with no anti-windup --
    //   current_command += k_p*(e[k]-e[k-1]) + k_d*(...) + k_i*e[k]
    // with k_i 40 and a 30 ms loop. A 1 rpm target the motor reports back as
    // 0 rpm is an error of 1 that never clears, so the integral term adds 40
    // every loop until the command saturates at 32767 and the wheel breaks
    // away at full power. Nav2 asks for velocities under half an rpm all the
    // time -- trimming a heading, easing into a goal -- so this turned every
    // one of them into a lurch. Below the deadband the robot should sit
    // still; the goal tolerances are what stop a goal hanging on it.
    const float rpm = command_radps[i] / (M_PI * 2.0f) * 60.0f;
    const int32_t speed_rpm = static_cast<int32_t>(std::lround(rpm));
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

  // Wait for a whole frame, waking when the kernel has something rather than on
  // a 10 ms tick.
  //
  // This loop used to probe FIONREAD and then usleep(10000) unconditionally, so
  // the wait was quantised to 10 ms and never shorter than 10 ms even when the
  // frame was already sitting in the buffer. At 9600 baud a 14 byte reply takes
  // 14.6 ms on the wire, which lands the wait on 20 ms about as often as 30, and
  // the cycle time moved in 10 ms steps with it. A constant command does not
  // care when it is repeated, which is why teleop looks smooth; a command that
  // changes every cycle gets applied at those uneven moments, and that is the
  // stutter. poll() returns as soon as a byte arrives, so the wait becomes the
  // frame's actual arrival time.
  //
  // The overall budget is unchanged at 300 ms: this is about when the wait ends,
  // not about how long a broken link is tolerated.
  const int32_t want = static_cast<int32_t>(motor_num_) * 6 + 2;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(300);
  int available_size = 0;
  for (;;)
  {
    ioctl(device_fd_, FIONREAD, &available_size);
    if (available_size >= want)
    {
      break;
    }
    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline)
    {
      tcflush(device_fd_, TCIFLUSH);
      return false;
    }
    const auto left =
      std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count();
    struct pollfd pfd = {device_fd_, POLLIN, 0};
    // At least 1 ms so a deadline less than a millisecond away still yields.
    poll(&pfd, 1, static_cast<int>(left > 0 ? left : 1));
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


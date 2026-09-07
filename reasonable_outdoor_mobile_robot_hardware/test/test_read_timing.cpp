// How long readRad() waits for a reply that is already on its way.
//
// The robot stuttered under Nav2 and not under teleop. The control loop runs at
// 20 Hz and blocks inside readRad() waiting for the motor board, and that wait
// used to be a FIONREAD probe followed by an unconditional usleep(10000): never
// shorter than 10 ms, and always rounded up to the next 10 ms. A command that is
// repeated unchanged does not care when it lands, which is teleop; a command
// that changes every cycle is applied at those uneven moments, which is Nav2.
//
// A pty stands in for the serial port. The test writes a frame after a chosen
// delay and measures how long readRad() takes to come back. What is being
// checked is that the wait tracks the delay instead of climbing to the next
// multiple of 10 ms.

#include <fcntl.h>
#include <pty.h>
#include <termios.h>
#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "reasonable_outdoor_mobile_robot_hardware/reasonable_robot_arduino_communicator.h"

namespace
{
constexpr int kMotors = 2;
constexpr int kFrameLen = kMotors * 6 + 2;   // header + 6 bytes a motor + checksum

// One status frame the parser will accept: 0x55, six bytes per motor, then the
// sum of everything before it.
std::vector<uint8_t> MakeFrame()
{
  std::vector<uint8_t> f(kFrameLen, 0);
  f[0] = 0x55;
  for (int i = 0; i < kMotors; i++)
  {
    f[i * 6 + 1] = 0x00; f[i * 6 + 2] = 0x10;   // angle
    f[i * 6 + 3] = 0x00; f[i * 6 + 4] = 0x05;   // speed
    f[i * 6 + 5] = 0x00; f[i * 6 + 6] = 0x01;   // current
  }
  uint8_t sum = 0;
  for (int i = 0; i < kFrameLen - 1; i++)
  {
    sum += f[i];
  }
  f[kFrameLen - 1] = sum;
  return f;
}

class PtyLink
{
public:
  PtyLink()
  {
    char name[256];
    if (openpty(&master_, &slave_, name, nullptr, nullptr) != 0)
    {
      master_ = slave_ = -1;
      return;
    }
    slave_name_ = name;
    // The communicator opens the slave by name itself; this end is only held
    // open so the pty does not collapse in between.
  }
  ~PtyLink()
  {
    if (master_ >= 0) { close(master_); }
    if (slave_ >= 0) { close(slave_); }
  }
  bool ok() const { return master_ >= 0; }
  const std::string & slave_name() const { return slave_name_; }
  void Send(const std::vector<uint8_t> & bytes) const
  {
    ssize_t n = write(master_, bytes.data(), bytes.size());
    (void)n;
  }

private:
  int master_{-1};
  int slave_{-1};
  std::string slave_name_;
};

// Time readRad() while the frame is put on the wire after `delay`.
std::chrono::milliseconds TimeRead(std::chrono::milliseconds delay)
{
  PtyLink link;
  EXPECT_TRUE(link.ok()) << "could not open a pty";
  ReasonableRobotArduinoComunicator comm(link.slave_name(), kMotors);

  const auto frame = MakeFrame();
  std::thread sender([&link, &frame, delay] {
      std::this_thread::sleep_for(delay);
      link.Send(frame);
    });

  std::vector<float> rad(kMotors), radps(kMotors), current(kMotors);
  const auto t0 = std::chrono::steady_clock::now();
  const bool got = comm.readRad(rad, radps, current);
  const auto elapsed = std::chrono::steady_clock::now() - t0;
  sender.join();

  EXPECT_TRUE(got) << "readRad rejected a well-formed frame";
  return std::chrono::duration_cast<std::chrono::milliseconds>(elapsed);
}
}  // namespace

// A reply that is already buffered should come back at once. The old loop slept
// 10 ms first, every time.
TEST(ReadTiming, AlreadyBufferedReturnsImmediately)
{
  const auto waited = TimeRead(std::chrono::milliseconds(0));
  EXPECT_LT(waited.count(), 5) << "waited " << waited.count() << " ms for a frame already sent";
}

// The wait should follow the arrival time rather than round up to the next
// 10 ms. 14 ms is the interesting case: it is roughly what a 14 byte frame
// costs at 9600 baud, and the old loop turned it into 20 ms.
TEST(ReadTiming, TracksArrivalRatherThanTenMillisecondTicks)
{
  for (int delay_ms : {4, 14, 23})
  {
    const auto waited = TimeRead(std::chrono::milliseconds(delay_ms));
    EXPECT_GE(waited.count(), delay_ms - 2)
      << "returned before the frame was sent (" << delay_ms << " ms)";
    EXPECT_LT(waited.count(), delay_ms + 5)
      << "waited " << waited.count() << " ms for a frame sent after " << delay_ms << " ms";
  }
}

// A silent board still has to give up, and inside the budget.
TEST(ReadTiming, SilenceTimesOutAndReportsFailure)
{
  PtyLink link;
  ASSERT_TRUE(link.ok());
  ReasonableRobotArduinoComunicator comm(link.slave_name(), kMotors);

  std::vector<float> rad(kMotors), radps(kMotors), current(kMotors);
  const auto t0 = std::chrono::steady_clock::now();
  const bool got = comm.readRad(rad, radps, current);
  const auto waited = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - t0);

  EXPECT_FALSE(got);
  EXPECT_GE(waited.count(), 290) << "gave up early";
  EXPECT_LT(waited.count(), 400) << "overran the 300 ms budget";
}

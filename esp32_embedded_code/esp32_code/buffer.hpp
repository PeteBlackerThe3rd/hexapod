// Tiny file, but it's the only way to cleanly use buffers in the packets and the packetiser
#ifndef __BUFFER_HPP__
#define __BUFFER_HPP__
#include <vector>
#include <memory>

typedef std::vector<uint8_t> Buffer;
typedef std::shared_ptr<Buffer> BufferPtr;

class JointState {
public:
  uint16_t rawFeedback;
  int16_t feedbackAngle_mrads;
  uint16_t current_mA;
  uint16_t driveDutyCycle;
  int16_t goalAngle_mrads;
};

#endif // __BUFFER_HPP__

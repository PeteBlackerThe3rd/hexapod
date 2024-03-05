// Tiny file, but it's the only way to cleanly use buffers in the packets and the packetiser
#ifndef __BUFFER_HPP__
#define __BUFFER_HPP__
#include <vector>
#include <memory>

typedef std::vector<uint8_t> Buffer;
typedef std::shared_ptr<Buffer> BufferPtr;

#endif // __BUFFER_HPP__

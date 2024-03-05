/*

Packet defintions used by Hexapod_V1 (Need to think of a better name at some point)

*/
#ifndef __PACKET_DEFS_HPP__
#define __PACKET_DEFS_HPP__
#include <memory>
#include <string>
#include "buffer.hpp"

class BasePacket;
typedef std::shared_ptr<BasePacket> BasePacketPtr;

class BaseException
{
public:
  BaseException(std::string msg) : msg(msg) {};
  std::string& what() {return msg; };
protected:
  std::string msg;
};

class InvalidPacketLength : public BaseException { public: InvalidPacketLength(const std::string msg) : BaseException(msg) {}; };
class InvalidProtocolId : public BaseException { public: InvalidProtocolId(const std::string msg) : BaseException(msg) {}; };
class InvalidDirectionId : public BaseException { public: InvalidDirectionId(const std::string msg) : BaseException(msg) {}; };
class InvalidPacketId : public BaseException { public: InvalidPacketId(const std::string msg) : BaseException(msg) {}; };

class BasePacket
{
public:

  static BasePacketPtr deserialise(Buffer& buffer);
  virtual std::string toString() = 0;

private:

  static constexpr uint8_t protocolId = 0xF2;
  static constexpr uint8_t TCId = 0x01;
  static constexpr uint8_t TMId = 0x02;
};

class TCAreYouAlivePacket : public BasePacket
{
public:
  static size_t expectedSize(Buffer& buffer) { return 5; };
  std::string toString() { return "TCAreYouAlivePacket"; };
  static constexpr uint8_t packetId = 0x10;
};

BasePacketPtr BasePacket::deserialise(Buffer& buffer)
{
  if (buffer.size() < 5)
    throw(InvalidPacketLength("Buffer size (" + std::to_string(buffer.size()) + ") is less than minimum packet size"));

  if (buffer[0] != protocolId)
    throw(InvalidProtocolId("Initial byte (" + std::to_string(buffer[0]) + ") is not protocol Id (" + std::to_string(protocolId) + ")"));
  
  if (buffer[1] != TCId && buffer[1] != TMId)
    throw(InvalidDirectionId("Invalid direction Id (" + std::to_string(buffer[1]) + ")"));

  // TODO check length

  uint8_t packetId = buffer[2];
  switch(packetId) {
    case TCAreYouAlivePacket::packetId: 
      if (buffer.size() == TCAreYouAlivePacket::expectedSize(buffer)) 
        return BasePacketPtr(new TCAreYouAlivePacket());
      throw InvalidPacketLength("Buffer length invalid for TCAreYouAlivePacket");
  }
};


#endif // __PACKET_DEFS_HPP__

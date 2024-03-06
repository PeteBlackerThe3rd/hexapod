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
  virtual BufferPtr serialise() = 0;

protected:

  void addHeader(BufferPtr packetBuffer, uint8_t direction, uint8_t packetId) {
    packetBuffer->insert(packetBuffer->begin(), 5, 0);
    (*packetBuffer.get())[0] = protocolId;
    (*packetBuffer.get())[1] = direction;
    (*packetBuffer.get())[2] = packetId;
    uint16_t length = packetBuffer->size(); 
    (*packetBuffer.get())[3] = length & 0xff;
    (*packetBuffer.get())[4] = length >> 8;
  };

  static constexpr uint8_t TCId = 0x01;
  static constexpr uint8_t TMId = 0x02;

private:

  static constexpr uint8_t protocolId = 0xF2;
};

class TCAreYouAlivePacket : public BasePacket
{
public:
  static size_t expectedSize(Buffer& buffer) { return 5; };
  std::string toString() { return "TCAreYouAlivePacket"; };
  BufferPtr serialise() {
    BufferPtr packetBuffer(new Buffer);
    addHeader(packetBuffer, TMId, packetId);
    return packetBuffer;
  };
  static constexpr uint8_t packetId = 0x10;
};

class TMIAmAlivePacket : public BasePacket
{
public:
  static size_t expectedSize(Buffer& buffer) { return 5; };
  std::string toString() { return "TMIAmAlivePacket"; };
  BufferPtr serialise() {
    BufferPtr packetBuffer(new Buffer);
    addHeader(packetBuffer, TMId, packetId);
    return packetBuffer;
  };
  static constexpr uint8_t packetId = 0x11;
};

class TMHouseKeepingPacket : public BasePacket
{
public:
  TMHouseKeepingPacket(uint16_t sequenceId) {
    this->sequenceId = sequenceId;
    socketConnectionCount = 0;
    motorsOn = 0;
    batteryVoltage100thsVolt = 0;
  };
  static size_t expectedSize(Buffer& buffer) { return 11; };
  std::string toString() { return "TMHouseKeepingPacket"; };
  BufferPtr serialise() {
    BufferPtr packetBuffer(new Buffer);
    
    packetBuffer->push_back(sequenceId & 0xff);
    packetBuffer->push_back(sequenceId >> 8);
    packetBuffer->push_back(socketConnectionCount);
    packetBuffer->push_back(motorsOn);
    packetBuffer->push_back(batteryVoltage100thsVolt & 0xff);
    packetBuffer->push_back(batteryVoltage100thsVolt >> 8);

    addHeader(packetBuffer, TMId, 0x12);//packetId);
    return packetBuffer;
  };
  static constexpr uint8_t packetId = 0x12;

  uint16_t sequenceId;
  uint8_t socketConnectionCount;
  uint8_t motorsOn;
  uint16_t batteryVoltage100thsVolt;
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

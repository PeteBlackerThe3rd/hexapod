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
  virtual uint8_t getId() = 0;

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
  uint8_t getId() { return packetId; };
  static size_t expectedSize(Buffer& buffer) { return 5; };
  std::string toString() { return "TCAreYouAlivePacket"; };
  BufferPtr serialise() {
    BufferPtr packetBuffer(new Buffer);
    addHeader(packetBuffer, TMId, packetId);
    return packetBuffer;
  };
  static constexpr uint8_t packetId = 0x10;
};

class TCManualJointPositionPacket : public BasePacket
{
public:
  TCManualJointPositionPacket(Buffer& buffer) {
    jointIndex = buffer[5];
    driveType = buffer[6];
    drivePosition = buffer[7] | ((uint16_t)buffer[8]) << 8;
  };
  uint8_t getId() { return packetId; };
  static size_t expectedSize(Buffer& buffer) { return 9; };
  std::string toString() {
    std::string str = "TCManualJointPositionPacket: Joint:" + std::to_string(jointIndex) + " Drive Type:";
    switch(driveType) {
      case driveTypeFeedback: str += "Feedback goal "; break;
      case driveTypeServoDutyCycle: str += "Servo duty cycle "; break;
      case driveTypeAngle: str += "Angle goal "; break;
    }
    str += "position:" + std::to_string(drivePosition);
    return str;
  };
  BufferPtr serialise() {
    BufferPtr packetBuffer(new Buffer);
    // TODO add packet contents (not really needed on embedded side)
    addHeader(packetBuffer, TMId, packetId);
    return packetBuffer;
  };

  uint8_t jointIndex;
  uint8_t driveType;
  uint16_t drivePosition;

  static constexpr uint8_t driveTypeFeedback = 0;
  static constexpr uint8_t driveTypeServoDutyCycle = 1;
  static constexpr uint8_t driveTypeAngle = 2;
  
  static constexpr uint8_t packetId = 0xA0;
};

class TMIAmAlivePacket : public BasePacket
{
public:
  uint8_t getId() { return packetId; };
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
  uint8_t getId() { return packetId; };
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

class TMRobotStatePacket : public BasePacket
{
public:
  uint8_t getId() { return packetId; };
  TMRobotStatePacket(JointState* jointStates) {
    this->jointStates = jointStates;
    ownsMemory = false;
  };
  ~TMRobotStatePacket() {
    if (ownsMemory)
      delete[] jointStates;
  }
  static size_t expectedSize(Buffer& buffer) { return (10 * 18) + 5; };
  std::string toString() { return "TMHouseKeepingPacket"; };
  BufferPtr serialise() {
    BufferPtr packetBuffer(new Buffer);
    packetBuffer->reserve((10*18)+5);
    for (int j=0; j<18; ++j) {
      packetBuffer->push_back(jointStates[j].rawFeedback & 0xff);
      packetBuffer->push_back(jointStates[j].rawFeedback >> 8);
      packetBuffer->push_back(jointStates[j].feedbackAngle_mrads & 0xff);
      packetBuffer->push_back(jointStates[j].feedbackAngle_mrads >> 8);
      packetBuffer->push_back(jointStates[j].current_mA & 0xff);
      packetBuffer->push_back(jointStates[j].current_mA >> 8);
      packetBuffer->push_back(jointStates[j].driveDutyCycle & 0xff);
      packetBuffer->push_back(jointStates[j].driveDutyCycle >> 8);
      packetBuffer->push_back(jointStates[j].goalAngle_mrads & 0xff);
      packetBuffer->push_back(jointStates[j].goalAngle_mrads >> 8);
    }

    addHeader(packetBuffer, TMId, 0x13);//packetId);
    return packetBuffer;
  };
  static constexpr uint8_t packetId = 0x13;

  JointState* jointStates;
  bool ownsMemory;
};

#endif // __PACKET_DEFS_HPP__

#include "packet_defs.hpp"

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

    case TCManualJointPositionPacket::packetId:
      if (buffer.size() == TCManualJointPositionPacket::expectedSize(buffer))
        return BasePacketPtr(new TCManualJointPositionPacket(buffer));
      throw InvalidPacketLength("Buffer length invalid for TCManualJointPositionPacket");
  }
};
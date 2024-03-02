import struct


class BasePacket:

  PROTOCOL_ID = 0xF2
  TC_ID = 0x01
  TM_ID = 0x02

  def __init__(self):
    pass

  def serialise_header(self, direction, packet_id, payload):
    header = struct.pack(">BBBH", self.PROTOCOL_ID, direction, packet_id, len(payload) + 5)
    # print("Serialising header with dir[%d] id[%0x02X] payload_len(%d)" % (direction, packet_id, len(payload)))
    packet = header + bytes(payload)
    # print("packet length %d bytes" % len(packet))
    return packet

  @classmethod
  def matches(cls, direction, packet_id):
    return False

  @classmethod
  def deserialise(cls, buffer):

    if len(buffer) < 5:
      print("Failed to deserialise, buffer shorter than 5 bytes")
      return None

    prot_id, direction, pack_id, length = struct.unpack(">BBBH", buffer[:6])
    if prot_id != cls.PROTOCOL_ID:
      print("Failed to deserialise, protocol ID [0x%02X] is invalid" % prot_id)
      return None

    if direction not in [cls.TC_ID, cls.TM_ID]:
      print("Failed to deserialise, direction ID [0x%02X] is invalid" % direction)
      return None

    if length != len(buffer):
      print("Failed to deserialise, encoded length %d does not match buffer length %d" % (length, len(buffer)))
      return None

    # find a matching concrete packet class to this buffer
    for PacketClass in cls.__subclasses__():
      if PacketClass.matches(direction, pack_id):
        return PacketClass.deserialise(buffer[6:])

    print("Failed to find a packet class matching packet_id:0x%02F" % pack_id)
    return None


class TCAreYouAlivePacket(BasePacket):

  PACKET_ID = 0x10

  def __init__(self):
    pass

  @classmethod
  def matches(cls, direction, packet_id):
    return direction == cls.TC_ID and packet_id == cls.PACKET_ID

  def serialise(self):
    return self.serialise_header(self.TC_ID, self.PACKET_ID, [])

  @classmethod
  def deserialise(cls, buffer):
    if len(buffer) == 0:
      return TCAreYouAlivePacket()
    else:
      print("Failed to deserialise are you alive TC, payload not zero length")
      return None


class TMIAmAlivePacket(BasePacket):

  PACKET_ID = 0x11

  def __init__(self):
    pass

  @classmethod
  def matches(cls, direction, packet_id):
    return direction == cls.TM_ID and packet_id == cls.PACKET_ID

  def serialise(self):
    return self.serialise_header(self.TC_ID, self.PACKET_ID, [])

  @staticmethod
  def deserialise(cls, buffer):
    if len(buffer) == 0:
      return TMIAmAlivePacket()
    else:
      print("Failed to deserialise I am alive TM, payload not zero length")
      return None

import struct


class BasePacket:

  PROTOCOL_ID = 0xF2
  TC_ID = 0x01
  TM_ID = 0x02

  def __init__(self):
    pass

  def serialise_header(self, direction, packet_id, payload):
    header = struct.pack(">BBBI", self.PROTOCOL_ID, direction, packet_id, len(payload))
    return header + bytes(payload)

  @staticmethod
  def matches(direction, packet_id):
    return False

  @staticmethod
  def deserialise(cls, buffer):

    if len(buffer) < 7:
      print("Failed to deserialise, buffer shorter than 7 bytes")
      return None

    prot_id, direction, pack_id, length = struct.unpack(">BBBI", buffer[:6])
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

  @staticmethod
  def matches(self, direction, packet_id):
    return direction == self.TC_ID and packet_id == self.PACKET_ID

  def serialise(self):
    return self.serialise_header(self.TC_ID, self.PACKET_ID, [])

  @staticmethod
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

  @staticmethod
  def matches(self, direction, packet_id):
    return direction == self.TM_ID and packet_id == self.PACKET_ID

  def serialise(self):
    return self.serialise_header(self.TC_ID, self.PACKET_ID, [])

  @staticmethod
  def deserialise(cls, buffer):
    if len(buffer) == 0:
      return TMIAmAlivePacket()
    else:
      print("Failed to deserialise I am alive TM, payload not zero length")
      return None

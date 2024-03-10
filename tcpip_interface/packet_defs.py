import struct


class BasePacket:

  PROTOCOL_ID = 0xF2
  TC_ID = 0x01
  TM_ID = 0x02

  def __init__(self):
    pass

  def serialise_header(self, direction, packet_id, payload):
    header = struct.pack("<BBBH", self.PROTOCOL_ID, direction, packet_id, len(payload) + 5)
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

    prot_id, direction, pack_id, length = struct.unpack("<BBBH", buffer[:5])
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
        return PacketClass.deserialise(buffer[5:])

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


class TCManualJointPositionPacket(BasePacket):

  PACKET_ID = 0xA0

  def __init__(self, jointIdx, driveType, position):
    self.joint_idx = jointIdx
    self.drive_type = driveType
    self.position = position

  @classmethod
  def matches(cls, direction, packet_id):
    return direction == cls.TC_ID and packet_id == cls.PACKET_ID

  def serialise(self):
    buffer = struct.pack("<BBH", self.joint_idx, self.drive_type, self.position)
    return self.serialise_header(self.TC_ID, self.PACKET_ID, buffer)

  @classmethod
  def deserialise(cls, buffer):
    if len(buffer) == 4:
      joint_idx, drive_type, position = struct.unpack("<BBH", buffer)
      return TCManualJointPositionPacket(joint_idx, drive_type, position)
    else:
      print("Failed to deserialise TCManualJointPositionPacket, payload length not 4 bytes")
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

  @classmethod
  def deserialise(cls, buffer):
    if len(buffer) == 0:
      return TMIAmAlivePacket()
    else:
      print("Failed to deserialise I am alive TM, payload not zero length")
      return None


class TMHouseKeepingPacket(BasePacket):

  PACKET_ID = 0x12

  def __init__(self):
    self.sequence_id = 0
    self.socket_connection_conunt = 0
    self.motors_on = 0
    self.voltage100thVolt = 0

  def __str__(self):
    desc = "House Keeping Packet: "
    desc += "SeqId: %d " % self.sequence_id
    desc += "Connections: %d " % self.socket_connection_conunt
    desc += "Motors On " if self.motors_on == 1 else "Motors Off "
    desc += "Battery Voltage %.2f v" % (self.voltage100thVolt * 0.01)
    return desc

  @classmethod
  def matches(cls, direction, packet_id):
    return direction == cls.TM_ID and packet_id == cls.PACKET_ID

  def serialise(self):
    return self.serialise_header(self.TC_ID, self.PACKET_ID,
                                 struct.pack("<HBBH",
                                             self.sequence_id,
                                             self.socket_connection_conunt,
                                             self.motors_on,
                                             self.voltage100thVolt))

  @classmethod
  def deserialise(cls, buffer):
    if len(buffer) == 6:
      hk_packet = TMHouseKeepingPacket()
      hk_packet.sequence_id, hk_packet.socket_connection_conunt, hk_packet.motors_on, hk_packet.voltage100thVolt = \
        struct.unpack("<HBBH", buffer)
      return hk_packet
    else:
      print("Failed to deserialise House Keeping TM, payload not 6 bytes in length")
      return None


class JointState:

  def __init__(self):
    self.raw_feedback = 0
    self.feedback_angle_mrads = 0
    self.current_mA = 0
    self.drive_duty_cycle = 0
    self.goal_angle_mrads = 0


class TMRobotStatePacket(BasePacket):

  PACKET_ID = 0x13

  def __init__(self):
    self.joint_states = []
    for _ in range(18):
      self.joint_states.append(JointState())

  def __str__(self):
    desc = "RobotState Packet: "
    desc += "J0 angle: %f rads" % (self.joint_states[2].feedback_angle_mrads / 1000.0)
    return desc

  @classmethod
  def matches(cls, direction, packet_id):
    return direction == cls.TM_ID and packet_id == cls.PACKET_ID

  def serialise(self):
    buffer = bytearray()
    for joint_state in self.joint_states:
      buffer.extend(struct.pack("<HhHHh",
                                joint_state.raw_feedback,
                                joint_state.feedback_angle_mrads,
                                joint_state.current_mA,
                                joint_state.drive_duty_cycle,
                                joint_state.goal_angle_mrads))
    return self.serialise_header(self.TC_ID, self.PACKET_ID, buffer)

  @classmethod
  def deserialise(cls, buffer):
    if len(buffer) == 180:
      rs_packet = TMRobotStatePacket()
      for j in range(18):
        (rs_packet.joint_states[j].raw_feedback,
         rs_packet.joint_states[j].feedback_angle_mrads,
         rs_packet.joint_states[j].current_mA,
         rs_packet.joint_states[j].drive_duty_cycle,
         rs_packet.joint_states[j].goal_angle_mrads) = struct.unpack("<HhHHh", buffer[j*10:j*10+10])

      return rs_packet
    else:
      print("Failed to deserialise House Keeping TM, payload not 6 bytes in length")
      return None

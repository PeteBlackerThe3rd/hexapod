import socket
from .stream_packetiser import StreamPacketiser
from .packet_defs import *


class RobotTCPIPInterface:

  HOST = "192.168.4.1"  # The server's hostname or IP address
  PORT = 16523  # The port used by the server

  def __init__(self):
    # echo-client.py

    self.packetiser = StreamPacketiser()
    self.connected = False
    self.socket = None

  def connect(self):
    self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    self.socket.connect((self.HOST, self.PORT))

    are_you_alive_packet = TCAreYouAlivePacket()
    data = self.packetiser.encapsulate_packet(are_you_alive_packet.serialise())
    self.socket.sendall(data)
    data = self.socket.recv(1024)

    print(f"Received {data!r}")
    self.connected = True

  def is_connected(self):
    return self.connected

  def disconnect(self):
    if not self.connected:
      print("Warning called disconnect on an already disconnected TCPIP Interface")
    self.socket.close()

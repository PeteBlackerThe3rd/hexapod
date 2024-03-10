import socket
import threading
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

    self.decoder_thread = threading.Thread(target=self.decoder_thread_entry)
    self.decoder_shutdown = threading.Event()

    self.robot_state = None
    self.robot_state_mutex = threading.Lock()

    self.i_am_alive_received = threading.Event()

  def connect(self):
    self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    self.socket.connect((self.HOST, self.PORT))
    self.socket.settimeout(0.1)
    self.decoder_thread.start()

    self.send_packet(TCAreYouAlivePacket())
    # data = self.socket.recv(1024)

    # print(f"Received {data!r}")
    self.connected = True

  def send_packet(self, packet):
    serialised_packet = packet.serialise()
    print("Sending packet: " + ''.join("{:02x} ".format(x) for x in serialised_packet))
    encapsulated_data = self.packetiser.encapsulate_packet(serialised_packet)
    self.socket.sendall(encapsulated_data)

  def decoder_thread_entry(self):

    while not self.decoder_shutdown.is_set():
      # read socket
      try:
        new_data = self.socket.recv(1024)

        if len(new_data) == 0:
          continue

        # if some data was received then packetise it and process packets
        new_packets = self.packetiser.process_input(new_data, return_garbage=True)
        for i, packet in enumerate(new_packets):
          if packet[0] == 'Packet':
            hex_str = ' '.join("{:02x}".format(x) for x in packet[1])
            # print("Received packet [%s]" % hex_str)
            decoded_packet = BasePacket.deserialise(packet[1])

            if isinstance(decoded_packet, TMIAmAlivePacket):
              print("I am alive! packet received")
              self.i_am_alive_received.set()

            if isinstance(decoded_packet, TCAreYouAlivePacket):
              print("Are you alive! TC packet received")
              self.i_am_alive_received.set()

            if isinstance(decoded_packet, TMHouseKeepingPacket):
              print(decoded_packet)

            if isinstance(decoded_packet, TMRobotStatePacket):
              print(decoded_packet)
              self.robot_state_mutex.acquire()
              self.robot_state = decoded_packet
              self.robot_state_mutex.release()
              # print(decoded_packet)

            # add further packet types as they're developed

          elif packet[0] == 'Garbage':
            hex_str = ''.join("{:02x}".format(x) for x in packet[1])
            print("Received garbase [%s]" % hex_str)

      except socket.timeout:
        pass

  def is_connected(self):
    return self.connected

  def disconnect(self):
    if not self.connected:
      print("Warning called disconnect on an already disconnected TCPIP Interface")
    self.socket.close()

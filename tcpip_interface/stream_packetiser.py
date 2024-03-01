

class StreamPacketiser:

  # Define Encapsulation codes
  START_MARKER = 0xa1
  STOP_MARKER = 0xa2
  ESC_SEQUENCES = [{'orig': 0xa1, 'esc': bytearray([0xa3, 0xa4])},
                   {'orig': 0xa2, 'esc': bytearray([0xa3, 0xa5])},
                   {'orig': 0xa3, 'esc': bytearray([0xa3, 0xa6])}]

  def __init__(self):
    self.input_buffer = bytearray([])

  @classmethod
  def match_esc(cls, seq):
    if len(seq) != 2:
      return False
    for esc_seq in cls.ESC_SEQUENCES:
      if seq == esc_seq['esc']:
        return esc_seq['orig']
    return False

  def encapsulate_packet(self, data):
    output = bytearray([self.START_MARKER])

    # add packet while escaping special bytes
    for byte in data:
      escaped = False
      for esc in self.ESC_SEQUENCES:
        if byte == esc['orig']:
          output.extend(esc['esc'])
          escaped = True
          break
      if not escaped:
        output.append(byte)

    output.append(self.STOP_MARKER)
    return output

  def process_input(self, data, return_garbage=False, debug=False):
    outputs = []
    partial_packet_found = False

    # combine previous input buffer and new data
    stream = self.input_buffer + data

    max_loop = 1000

    # while a partial packet has not been found and there is data in the buffer continue to process the buffer
    while not partial_packet_found and len(stream) > 0 and max_loop > 0:

      max_loop -= 1

      #print("Stream len = %d" % len(stream))

      # if the first byte is not a start byte, then create a garbage segment of discarded bytes until a
      # start byte is found
      if stream[0] != self.START_MARKER:
        garbage_bytes = bytearray()
        while len(stream) > 0 and stream[0] != self.START_MARKER:
          #print("Start g dump")
          garbage_bytes += bytearray([stream[0]])
          stream = stream[1:]
        if return_garbage:
          outputs.append(('Garbage', garbage_bytes))
        #print("dumped %d garbage bytes searching for first StartMarker" % len(garbage_bytes))

      # remove start marker if the stream isn't empty
      # if len(stream) > 0:
      #   stream = stream[1:]

      # if there are any bytes left then they must start with a start marker so start to identify packet
      packet_encap_length = 1
      decapsulated_packet = bytearray()
      packet_complete = False
      garbage_packet_found = False
      while len(stream) > packet_encap_length:  # and not packet_complete and not garbage_packet_found:
        #print("Inner s[%d] p[%d]" % (len(stream), packet_encap_length))
        # if a stop marker is found then complete the packet
        if stream[packet_encap_length] == self.STOP_MARKER:
          packet_encap_length += 1
          packet_complete = True
          #print("end ")
          break

        # if a start marker was found (should not happen unless some garbage data happened to include a start marker)
        # then create a garbage section
        elif stream[packet_encap_length] == self.START_MARKER:
          garbage_section = ('Garbage', bytearray(stream[:packet_encap_length]))
          outputs.append(garbage_section)
          stream = stream[packet_encap_length:]
          garbage_packet_found = True
          #print("illegal start")
          break

        # if an escape sequence was found then decode it
        elif self.match_esc(stream[packet_encap_length:packet_encap_length + 2]) is not False:  # len(stream) > packet_encap_length + 1:
          #print("Esc found")
          decapsulated_packet.append(self.match_esc(stream[packet_encap_length:packet_encap_length + 2]))
          packet_encap_length += 2

        # if the byte was not a start/stop marker or escape sequence then add verbatim
        else:
          decapsulated_packet.append(stream[packet_encap_length])
          packet_encap_length += 1
          #print("added byte")

      # if a complete packet was found then add it to the output and remove the encapsulated
      # version from the input stream
      if packet_complete:
        outputs.append(('Packet', decapsulated_packet))
        stream = stream[packet_encap_length:]

      # if the packet was not complete then flag an incomplete packet on the input buffer
      else:
        partial_packet_found = True

    # return any remaining stream data to the global variable and return found sections
    self.input_buffer = stream
    return outputs

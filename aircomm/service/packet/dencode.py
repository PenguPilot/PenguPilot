
#
# ARCADE NRF PACKET FORMAT
#
# purpose: packet pack/unpack functions and constant definitions
# author: Tobias Simon, Ilmenau University of Technology
#


import struct



MAX_LEN = 32
HEADER_LEN = 4
MAX_PAYLOAD_LEN = MAX_LEN - HEADER_LEN
BCAST = 0xFF


class Packet:

   def __init__(self, src, dst, seq, type, data):
      if src == BCAST:
         raise ValueError('broadcast address (%d) must not be used for src' % BCAST)
      self.src = src
      self.dst = dst
      self.seq = seq
      self.type = type
      self.data = data

   def __str__(self):
      return 'msg src: %d, dst: %d, seq: %d, type: %d, data: %s' % (self.src, self.dst, self.seq, self.type, self.data)


def encode(msg):
   data_len = len(msg.data)
   header = struct.pack('BBBB', msg.src, msg.dst, msg.seq, msg.type)
   if HEADER_LEN + data_len > MAX_LEN:
      raise ValueError('length of payload must be < %d' % MAX_PAYLOAD_LEN)
   return header + msg.data


def decode(packet):
   if len(packet) < HEADER_LEN:
      raise ValueError('length of packet must be >= %d' % HEADER_LEN)
   if len(packet) > 32:
      raise ValueError('length of packet must be < %d' % MAX_LEN)
   header = packet[0 : HEADER_LEN]
   data = packet[HEADER_LEN : ]
   src, dst, seq, type = struct.unpack('BBBB', header)
   if len(data) > MAX_PAYLOAD_LEN:
      raise ValueError('length of payload must be < %d' % MAX_PAYLOAD_LEN)
   return Packet(src, dst, seq, type, data)


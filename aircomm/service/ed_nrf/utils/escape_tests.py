#!/usr/bin/env python

import serial

bus=serial.Serial('/dev/ttyACM0',timeout=1)

in_str='A'+'B'+chr(0xC0)+chr(0xC0)+chr(0xC0)+chr(0xC0)+'C'+'D'+chr(0xC0)+chr(0xC1)
print map(ord,in_str)
bus.write(in_str)
print map(ord,bus.read(10))

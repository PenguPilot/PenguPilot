#!/usr/bin/env python
"""
  ___________________________________________________
 |  _____                       _____ _ _       _    |
 | |  __ \                     |  __ (_) |     | |   |
 | | |__) |__ _ __   __ _ _   _| |__) || | ___ | |_  |
 | |  ___/ _ \ '_ \ / _` | | | |  ___/ | |/ _ \| __| |
 | | |  |  __/ | | | (_| | |_| | |   | | | (_) | |_  |
 | |_|   \___|_| |_|\__, |\__,_|_|   |_|_|\___/ \__| |
 |                   __/ |                           |
 |  GNU/Linux based |___/  Multi-Rotor UAV Autopilot |
 |___________________________________________________|
  
 OLED Display Showing Various Information

 Copyright (C) 2013 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. """


from PIL import Image, ImageDraw, ImageFont
import pyssd1306 as oled
from time import sleep, time
from psutil import cpu_percent
from threading import Thread, Lock
from scl import generate_map
from gps_data_pb2 import GpsData
from math import sin, cos, pi
from misc import daemonize
from os import getenv
from power_pb2 import PowerState


WHITE = 1
BLACK = 0
W = 128
H = 64
flying = False
socket_map = None


def flying_reader():
   global flying, socket_map
   socket = socket_map['flying']
   while True:
      if socket.recv() == 'flying':
         flying = True
      else:
         flying = False


def gps():
   global gps_data, socket_map
   gps_data = None
   socket = socket_map['gps']
   while True:
      with gps_lock:
         gps_data = GpsData()
         gps_data.ParseFromString(socket.recv())


def cpuavg():
   global load
   load = None
   while True:
      if load is None:
         load = cpu_percent()
      else:
         load = 0.9 * load + 0.1 * cpu_percent()
      sleep(0.05)


def pmreader():
   s = socket_map['power']
   p = PowerState()
   global voltage, critical
   critical = False
   voltage = None
   while True:
      p.ParseFromString(s.recv())
      critical = p.critical
      if voltage is None:
         voltage = p.voltage
      else:
         voltage = 0.9 * voltage + 0.1 * p.voltage


def show_image(image):
   data = ''.join(map(chr, image.getdata()))
   oled.blit(data)

def hline(draw, h):
   draw.line([(0, h), (W - 1, h)], WHITE)

def decorations(draw):
   hline(draw, 0)
   hline(draw, 1)
   hline(draw, 7)
   hline(draw, H - 8)
   hline(draw, H - 2)
   hline(draw, H - 1)


def fit_text_width(txt, img_fraction, font_path = getenv('PENGUPILOT_PATH') + '/display/service/verdana.ttf'):
   # taken from: http://stackoverflow.com/a/4902713
   fontsize = 1
   font = ImageFont.truetype(font_path, fontsize)
   while font.getsize(txt)[0] < img_fraction * W:
      fontsize += 1
      font = ImageFont.truetype(font_path, fontsize)
   fontsize -= 1
   return ImageFont.truetype(font_path, fontsize)

def batt_low():
   image = Image.new("1", (W, H), BLACK)
   txt = "BATTERY LOW"
   font = fit_text_width(txt, 1.0)
   draw = ImageDraw.Draw(image)
   dim = font.getsize(txt)
   draw.text(((W - dim[0]) / 2, 10), txt, WHITE, font = font)
   txt = 'Voltage: %.1f' % voltage
   dim = font.getsize(txt)
   draw.text(((W - dim[0]) / 2, 31), txt, WHITE, font = font)
   decorations(draw)
   show_image(image) 

def caution():
   image = Image.new("1", (W, H), BLACK)
   txt = "CAUTION"
   font = fit_text_width(txt, 1.0)
   draw = ImageDraw.Draw(image)
   dim = font.getsize(txt)
   draw.text(((W - dim[0]) / 2, (H - dim[1]) / 2), txt, WHITE, font = font)
   decorations(draw)
   show_image(image) 

class Alert(Thread):

   def __init__(self, pre, inner, cnt, init_draw, contin_draw = False):
      Thread.__init__(self)
      self.pre = pre
      self.inner = inner
      self.cnt = cnt
      self.init_draw = init_draw
      self.contin_draw = contin_draw
      self.running = True
      self.daemon = True

   def _init_draw(self):
      while True:
         try:
            self.init_draw()
            return
         except:
            pass

   def run(self):
      self._init_draw()
      while self.running:
         if self.contin_draw:
            self._init_draw()
         sleep(self.pre)
         for _ in range(self.cnt):
            oled.invert(True)
            sleep(self.inner)
            oled.invert(False)
            sleep(self.inner)

def bar(draw, x, y, w, h, p):
   draw.polygon([(x, y), (x + w, y), (x + w, y + h), (x, y + h)], False, WHITE)
   pw = round((w - 1) * p, 0)
   draw.polygon([(x + 1, y + 1), (x + 1 + pw, y + 1), (x + 1 + pw, y + h - 1), (x + 1, y + h - 1)], True, WHITE)

def mem_used():
   lines = file('/proc/meminfo').readlines()
   total = float(lines[0].split(' ')[-2])
   free = float(lines[1].split(' ')[-2])
   cached = float(lines[3].split(' ')[-2])
   return 100 * (1.0 - (free + cached) / total)

def draw_health(draw):
   mem = mem_used()
   draw.text((0, 0), 'CPU: %d%%' % load, WHITE, font = font)
   draw.text((64 + 2, 0), 'Mem: %d%%' % mem, WHITE, font = font)
   bar(draw, 0, 14, 61, 6, load / 100.0)
   bar(draw, 64 + 2, 14, 61, 6, mem / 100.0)

   vmin = 13.2
   vmax = 16.4
   batt = min(1.0, max(0.0, (voltage - vmin) / (vmax - vmin)))

   draw.text((0, 24), 'Battery: %.1f%%' % (100.0 * batt), WHITE, font = font)
   bar(draw, 0, 38, 127, 6, batt)
   
def circle(draw, x, y, rad, i, o):
   draw.ellipse((x - rad, y - rad, x + rad, y + rad), i, o)

def pol2cart(az, el, x, y, r):
   az *= (pi / 180.0)
   el = ((90.0 - el) / 90.0)
   xout = x + sin(az) * el * r
   yout = y - cos(az) * el * r
   return (int(xout), int(yout))      

def draw_gps(draw):
   with gps_lock:
      fix_txt = {0: '--', 2: '2D', 3: '3D'}
      draw.text((0, 0), 'Sats: %d' % gps_data.sats, WHITE, font = font)
      draw.text((0, 13), 'Fix: %s' % fix_txt[gps_data.fix], WHITE, font = font)
      if gps_data.fix >= 2:
         draw.text((0, 13 * 2), 'HD: %.1f' % gps_data.hdop, WHITE, font = font)
         if gps_data.fix == 3:
            draw.text((0, 13 * 3), 'VD: %.1f' % gps_data.vdop, WHITE, font = font)
         else:
            draw.text((0, 13 * 3), '-------', WHITE, font = font)
      outer_rad = 31
      x_pos = 64 + outer_rad
      y_pos = outer_rad
      circle(draw, x_pos, y_pos, outer_rad, BLACK, WHITE)
      draw.line([(x_pos, y_pos - outer_rad), (x_pos, y_pos + outer_rad)], WHITE)
      draw.line([(x_pos - outer_rad, y_pos), (x_pos + outer_rad, y_pos)], WHITE)
      sig = 0.0
      for sat in gps_data.satinfo:
         sig += sat.sig
         x, y = pol2cart(sat.azimuth, sat.elv, x_pos, y_pos, outer_rad)
         if sat.in_use:
            circle(draw, x, y, 3, WHITE, WHITE)
         else:
            circle(draw, x, y, 3, BLACK, WHITE)
      sig /= len(gps_data.satinfo)
      draw.text((0, 13 * 4), 'Sig: %.1f' % sig, WHITE, font = font)

def draw_gps2(draw):
   with gps_lock:
      if gps_data.fix >= 2:
         draw.text((0, 13 * 0), 'Lon: %f' % gps_data.lon, WHITE, font = font)
         draw.text((0, 13 * 1), 'Lat: %f' % gps_data.lat, WHITE, font = font)
         if gps_data.fix == 3:
            draw.text((0, 13 * 2), 'Alt: %.1f' % gps_data.alt, WHITE, font = font)
      else:
         raise Exception


def main(name):
   global socket_map, gps_lock, font, flying
   socket_map = generate_map(name)

   gps_lock = Lock()
   font = ImageFont.truetype(getenv('PENGUPILOT_PATH') + '/display/service/verdana.ttf', 11)
   
   t = Thread(target = flying_reader)
   t.daemon = True
   t.start()

   t1 = Thread(target = cpuavg)
   t1.daemon = True
   t1.start()

   t2 = Thread(target = pmreader)
   t2.daemon = True
   t2.start()

   t3 = Thread(target = gps)
   t3.daemon = True
   t3.start()

   screens = [(draw_health, 15),
              (draw_gps, 15),
              (draw_gps2, 15)]

   screen = 0
   oled.init('/dev/i2c-3', W, H)
   sleep(5)
   try:
      while True:
         try:
            if not flying:
               t = time()
               while time() < t + screens[screen][1]:
                  image = Image.new("1", (W, H), BLACK)
                  draw = ImageDraw.Draw(image)
                  screens[screen][0](draw)
                  show_image(image)
                  sleep(1)
                  if critical:
                     alert = Alert(1.0, 0.1, 1, batt_low, True)
                     alert.start()
                     alert.join()
         except Exception, e:
            print e
         sleep(1)
         screen = (screen + 1) % len(screens)
   except:
      oled.invert(False)
      oled.clear()
      oled.update()


daemonize('display', main)


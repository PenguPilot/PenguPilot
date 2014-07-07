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

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

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
from threading import Thread
from scl import generate_map
from math import sin, cos, pi
from misc import daemonize, RateTimer
from os import getenv
from msgpack import loads
from gps_msgpack import *


WHITE = 1
BLACK = 0
W = 128
H = 64
spinning = False
socket_map = None


def spinning_reader():
   global spinning, socket_map
   socket = socket_map['motors_spinning']
   while True:
      if socket.recv() == 'true':
         spinning = True
      else:
         spinning = False


def gps_reader():
   global gps
   socket = socket_map['gps']
   rt = RateTimer(1)
   while True:
      raw = socket.recv()
      if spinning:
         continue
      if rt.expired():
         gps = loads(raw)


def sats_reader():
   global sats
   socket = socket_map['sats']
   rt = RateTimer(1)
   while True:
      raw = socket.recv()
      if spinning:
         continue
      if rt.expired():
         sats = loads(raw)


def cpu_reader():
   global spinning, load
   load = None
   while True:
      if spinning:
         sleep(1)
      else:
         if load is None:
            load = cpu_percent()
         else:
            load = 0.85 * load + 0.15 * cpu_percent()
         sleep(0.1)


def remote_reader():
   s = socket_map['remote']
   global channels
   rt = RateTimer(5)
   while True:
      raw = s.recv()
      if spinning:
         continue
      if rt.expired():
         channels = loads(raw)


decl = 0.0
def decl_reader():
   s = socket_map['decl']
   global decl
   while True:
      decl = float(s.recv())


def pm_reader():
   s = socket_map['powerman']
   global spinning, voltage, estimate, critical
   critical = False
   voltage = None
   estimate = None
   while True:
      raw = s.recv()
      if spinning:
         continue
      _voltage, current, remaining, critical = loads(raw)
      estimate = remaining / current
      if voltage is None:
         voltage = _voltage
      else:
         voltage = 0.9 * voltage + 0.1 * _voltage

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


caution_written = False
def caution():
   global caution_written
   if not caution_written:
      caution_written = True
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
   if not estimate:
      etimate = ''

   draw.text((0, 24), 'BAT: %.1f%%, T: %.1fh' % (100.0 * batt, estimate), WHITE, font = font)
   bar(draw, 0, 38, 127, 6, batt)
 

def draw_remote(draw):
   if channels[0]:
      draw.text((21, 0), 'RC signal: valid', WHITE, font = font)
   else:
      draw.text((15, 0), 'RC signal: invalid', WHITE, font = font)
   for i in range(6):
      bar(draw, 0, 17 + i * 8, 127, 6, channels[1 + i] + 0.5)


def circle(draw, x, y, rad, i, o):
   draw.ellipse((x - rad, y - rad, x + rad, y + rad), i, o)


def pol2cart(az, el, x, y, r):
   az *= (pi / 180.0)
   el = ((90.0 - el) / 90.0)
   xout = x + sin(az) * el * r
   yout = y - cos(az) * el * r
   return (int(xout), int(yout))      


def draw_gps(draw):
   fix_txt = {0: '--', 2: '2D', 3: '3D'}
   in_use = len(filter(lambda sat : sat[USE], sats))
   draw.text((0, 0), 'Sats: %d / %d' % (in_use, len(sats)), WHITE, font = font)
   draw.text((0, 13), 'Fix: %s' % fix_txt[fix(gps)], WHITE, font = font)
   if fix(gps) >= 2:
      draw.text((0, 13 * 2), 'HD: %.1f' % gps[HDOP], WHITE, font = font)
      if fix(gps) == 3:
         draw.text((0, 13 * 3), 'VD: %.1f' % gps[VDOP], WHITE, font = font)
      else:
         draw.text((0, 13 * 3), '-------', WHITE, font = font)
   outer_rad = 31
   x_pos = 64 + outer_rad
   y_pos = outer_rad
   circle(draw, x_pos, y_pos, outer_rad, BLACK, WHITE)
   draw.line([(x_pos, y_pos - outer_rad), (x_pos, y_pos + outer_rad)], WHITE)
   draw.line([(x_pos - outer_rad, y_pos), (x_pos + outer_rad, y_pos)], WHITE)
   sig = 0.0
   for sat in sats:
      if sat[SIG] > sig:
         sig = sat[SIG]
      x, y = pol2cart(sat[AZI], sat[ELV], x_pos, y_pos, outer_rad)
      if sat[USE]:
         circle(draw, x, y, 3, WHITE, WHITE)
      else:
         circle(draw, x, y, 3, BLACK, WHITE)
   draw.text((0, 13 * 4), 'Sig: %.1f' % sig, WHITE, font = font)


def draw_gps2(draw):
   try:
      draw.text((0, 13 * 0), 'Lat: %f' % gps[LAT], WHITE, font = font)
      draw.text((0, 13 * 1), 'Lon: %f' % gps[LON], WHITE, font = font)
      draw.text((0, 13 * 2), 'Declination: %.1f' % decl, WHITE, font = font)
      try:
         draw.text((0, 13 * 3), 'Altitude: %.1f' % gps[ALT], WHITE, font = font)
      except:
         pass
   except:
      raise Exception


def main(name):
   global socket_map, font, spinning, caution_written
   socket_map = generate_map(name)

   font = ImageFont.truetype(getenv('PENGUPILOT_PATH') + '/display/service/verdana.ttf', 11)
   
   t = Thread(target = spinning_reader)
   t.daemon = True
   t.start()

   t1 = Thread(target = cpu_reader)
   t1.daemon = True
   t1.start()

   t2 = Thread(target = pm_reader)
   t2.daemon = True
   t2.start()

   t3 = Thread(target = gps_reader)
   t3.daemon = True
   t3.start()
   
   t4 = Thread(target = sats_reader)
   t4.daemon = True
   t4.start()
   
   t5 = Thread(target = remote_reader)
   t5.daemon = True
   t5.start()

   t6 = Thread(target = decl_reader)
   t6.daemon = True
   t6.start()


   screens = [(draw_health, 10),
              (draw_gps, 10),
              (draw_gps2, 10),
              (draw_remote, 10)]

   screen = 0
   oled.init('/dev/i2c-3', W, H)
   sleep(5)
   try:
      while True:
         try:
            if not spinning:
               caution_written = False
               oled.invert(False)
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
            else:
               caution()
               sleep(0.2)
         except:
            sleep(1)
         screen = (screen + 1) % len(screens)
   except:
      oled.invert(False)
      oled.clear()
      oled.update()


daemonize('display', main)


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
  
 Crypto Functionality for Aircomm

 Copyright (C) 2013 Tobias Simon, Ilmenau University of Technology
 Original Code Copyright (C) 2009 joonis new media
 Author: Thimo Kraemer <thimo.kraemer@joonis.de>

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. """


import random
from hashlib import sha1


def init(_key, _salt_len = 2):
   global key, salt_len
   key = _key
   salt_len = _salt_len


def _crypt(data, key):
   x = 0
   box = range(256)
   for i in range(256):
      x = (x + box[i] + ord(key[i % len(key)])) % 256
      box[i], box[x] = box[x], box[i]
   x = y = 0
   out = []
   for char in data:
      x = (x + 1) % 256
      y = (y + box[x]) % 256
      box[x], box[y] = box[y], box[x]
      out.append(chr(ord(char) ^ box[(box[x] + box[y]) % 256]))
   return ''.join(out)


def encrypt(data):
   salt = ''
   for n in range(salt_length):
      salt += chr(random.randrange(256))
   data = salt + _crypt(data, sha1(key + salt).digest())
   return data


def decrypt(data):
   salt = data[:salt_length]
   return _crypt(data[salt_length:], sha1(key + salt).digest())


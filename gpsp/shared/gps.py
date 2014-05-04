
# satellite info elements:
PRN = 0
USE = 1
ELV = 2
AZI = 3
SIG = 4

# gps data elements:
TIME =   0
LAT =    1
LON =    2
SATS =   3
SPEED =  4
COURSE = 5
HDOP =   6
ALT =    7
VDOP =   8

def fix(gps):
   if len(gps) == 0:
      return 0
   elif len(gps) >= 7:
      return 2
   else:
      return 3





from sys import float_info
from math import hypot


def euclid_dist(a, b):
   return hypot(a[0] - b[0], a[1] - b[1])


class LandingSpots:

   def __init__(self, epsilon):
      self.spots = []
      self.epsilon = epsilon

   def add(self, new_spot):
      '''
      add landing spot, if it is not close to an already
      existing spot (using epsilon as distance)
      '''
      for spot in self.spots:
         if euclid_dist(new_spot, spot) < self.epsilon:
            return
      self.spots.append(new_spot)

   def get_closest(self, pos):
      '''
      get landing spot which is closest to pos
      '''
      min_dist = float_info.max
      for spot in self.spots:
         dist = euclid_dist(pos, spot)
         if dist < min_dist:
            min_dist = dist
            min_spot = spot
      try:
         return min_spot
      except:
         pass



if __name__ == '__main__':

   import unittest

   class TestLandingSpots(unittest.TestCase):

      def test_empty(self):
         spots = LandingSpots(2.0)
         self.assertEqual(spots.get_closest((10, 10)), None)

      def test_closest(self):
         spots = LandingSpots(2.0)
         spots.add((0, 0))
         spots.add((10, 0))
         spots.add((11, 0))
         spots.add((0, 10))
         self.assertEqual(len(spots.spots), 3)
         self.assertEqual(spots.get_closest((10, 5)), (10, 0))

   unittest.main()


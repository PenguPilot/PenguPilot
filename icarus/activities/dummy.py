
from activity import Activity


class DummyActivity(Activity):

   def __init__(self):
      Activity.__init__(self, None)

   def run(self):
      pass


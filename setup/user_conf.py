

# settings are selectable parameter sets for customizing
# the UAV to the user's needs


class Settings(type):
   pass  

class Indoor(Settings):
   mode = 'acc'
   angle_max = 20.0
   yaw_p = 2.0

class Outdoor(Settings):
   mode = 'gyro'
   pitch_roll_p = 2.0
   yaw_p = 3.0


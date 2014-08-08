
from optimizer import optimize

optimize(0.1, 1000, 'autopilot.controllers.stabilizing.att_', ['yaw_kp', 'yaw_ki', 'yaw_kii', 'yaw_kd'], [40])


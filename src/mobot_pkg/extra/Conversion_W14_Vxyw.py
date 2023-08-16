
def cmdvel2Vxyw(self):
  Vx = cmdvel.linear.x
  Vy = cmdvel.linear.y
  W0 = cmdvel.angular.z
  print(Vx, Vy, W0)

def Angular2Vxyw(self):
  Vx = (W1 + W2 + W3 + W4) * R / 4
  Vy = (W1 - W2 - W3 + W4) * R / 4
  W0 = (-W1*LS + W2*LS - W3*LS + W4*LS) * R / 4

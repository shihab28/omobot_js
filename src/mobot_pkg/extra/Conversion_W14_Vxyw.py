

def Angular2Vxyw(W1, W2, W3, W4):
  Vx = (W1 + W2 + W3 + W4) * R / 4
  Vy = (W1 - W2 - W3 + W4) * R / 4
  W0 = (-W1*LS + W2*LS - W3*LS + W4*LS) * R / 4
  print(Vx, Vy, W0)

W_pwm = [-3.120419,-3.142890,-3.293564,-3.294403]




L  = .1185
W  = .0825
SL = (L+W)
LS = 1/SL
r  = .0398
R  = 2*3.14159*r
def Angular2Vxyw(W_pwm):
  print(W_pwm)
  W1, W2, W3, W4 = W_pwm
  Vx = (W1 + W2 + W3 + W4) * R / 4
  Vy = (W1 - W2 - W3 + W4) * R / 4
  W0 = (-W1*LS + W2*LS - W3*LS + W4*LS) * R / 4
  print(Vx, Vy, W0)
Angular2Vxyw(W_pwm)


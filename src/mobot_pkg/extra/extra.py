vy_samples = []
for vals in range(700,1401,50):
    vy_samples.append(vals/1000)
    vy_samples.append(-vals/1000)

print(vy_samples)

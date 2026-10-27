import math
import time

t = 0.0

while True:

    x = math.cos(t)
    y = math.sin(t)

    print(
        f"{x:.3f},{y:.3f}"
    )

    t += 0.05

    time.sleep(0.05)

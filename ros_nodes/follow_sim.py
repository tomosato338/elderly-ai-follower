import math
import time
import csv

robot_x = 2.0
robot_y = 0.0

t = 0.0

with open(
    "/home/ubuntu/elderly_ai/data/follow_dataset.csv",
    "w",
    newline=""
) as f:

    writer = csv.writer(f)

    writer.writerow(
        [
            "actor_x",
            "actor_y",
            "robot_x",
            "robot_y",
            "linear_x"
        ]
    )

    while True:

        actor_x = math.cos(t)
        actor_y = math.sin(t)

        dx = actor_x - robot_x
        dy = actor_y - robot_y

        distance = math.sqrt(
            dx**2 + dy**2
        )

        linear_x = 0.5 * (
            distance - 1.0
        )

        linear_x = max(
            -0.5,
            min(
                0.5,
                linear_x
            )
        )

        dt = 0.05

        heading = math.atan2(
            dy,
            dx
        )

        robot_x += (
            linear_x *
            math.cos(heading) *
            dt
        )

        robot_y += (
            linear_x *
            math.sin(heading) *
            dt
        )

        writer.writerow(
            [
                actor_x,
                actor_y,
                robot_x,
                robot_y,
                linear_x
            ]
        )

        print(
            f"A({actor_x:.2f},{actor_y:.2f}) "
            f"R({robot_x:.2f},{robot_y:.2f}) "
            f"V={linear_x:.2f}"
        )

        t += 0.05

        time.sleep(0.05)

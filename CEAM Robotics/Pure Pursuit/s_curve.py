import numpy as np
import matplotlib.pyplot as plt

# ---------------------------
# scurve PATH
# ---------------------------

path_x = np.linspace(0, 10, 500)
path_y = np.sin(path_x)

# ---------------------------
# ROBOT STATE
# ---------------------------

x = 0.0
y = -1.0
yaw = 0.0

velocity = 0.2
lookahead = 0.4
dt = 0.05

actual_x = []
actual_y = []

# ---------------------------
# SIMULATION LOOP
# ---------------------------

plt.ion()

dt = 0.1

for _ in range(1000):

    distances = np.sqrt(
        (path_x - x)**2 +
        (path_y - y)**2
    )

    nearest_index = np.argmin(distances)
    
    target_index = nearest_index

    while (
        target_index < len(path_x)-1 and
        distances[target_index] < lookahead
    ):
        target_index += 1

    tx = path_x[target_index]
    ty = path_y[target_index]

    alpha = np.arctan2(
        ty - y,
        tx - x
    ) - yaw

    alpha = np.arctan2(
        np.sin(alpha),
        np.cos(alpha)
    )

    curvature = 2 * np.sin(alpha) / lookahead

    omega = 0.3 * velocity * curvature

    x += velocity * np.cos(yaw) * dt
    y += velocity * np.sin(yaw) * dt
    yaw += omega * dt

    actual_x.append(x)
    actual_y.append(y)
    if x >= path_x[-1]:
    	break

    plt.clf()

    plt.plot(path_x, path_y, 'b', label='Desired Path')
    plt.plot(actual_x, actual_y, 'orange', label='Actual Path')

    plt.scatter(x, y, s=80, label='Robot')

    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("Pure Pursuit - S Curve")

    plt.legend()
    plt.grid()
    plt.axis("equal")

    plt.pause(0.01)

plt.ioff()

# ---------------------------
# PLOT
# ---------------------------

plt.plot(path_x, path_y, label="Desired Path")
plt.plot(actual_x, actual_y, label="Actual Path")

plt.xlabel("X")
plt.ylabel("Y")
plt.title("Pure Pursuit - S Curve")

plt.legend()
plt.grid()
plt.axis("equal")

plt.savefig("S.png")
plt.show()

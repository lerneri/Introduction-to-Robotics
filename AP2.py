import numpy as np
import matplotlib.pyplot as plt

def cubcoef(th0, thf, thdot0, thdotf):
    a = th0
    b = thdot0
    c = (3 * (thf - th0) - 2 * thdot0 - thdotf)
    d = (2 * (th0 - thf) + thdot0 + thdotf)

    cc = np.array([a, b, c, d])
    return cc

def generate_path(coefficients, duration, update_rate):
    t = np.arange(0, duration, 1/update_rate)
    n = len(coefficients)

    position = np.zeros_like(t)
    velocity = np.zeros_like(t)
    acceleration = np.zeros_like(t)

    for i in range(n):
        dt = t[t <= (i+1)*duration/n][-1]  # Time within the current segment

        position[t <= (i+1)*duration/n] = (
            coefficients[i, 0] +
            coefficients[i, 1] * dt +
            coefficients[i, 2] * dt**2 +
            coefficients[i, 3] * dt**3
        )

        velocity[t <= (i+1)*duration/n] = (
            coefficients[i, 1] +
            2 * coefficients[i, 2] * dt +
            3 * coefficients[i, 3] * dt**2
        )

        acceleration[t <= (i+1)*duration/n] = (
            2 * coefficients[i, 2] +
            6 * coefficients[i, 3] * dt
        )

    return t, position, velocity, acceleration

def plan_and_execute_path(initial, via_points, final, duration, update_rate):
    all_points = [initial] + via_points + [final]
    all_points = np.deg2rad(all_points)  # Convert angles to radians

    coefficients = []
    for i in range(len(all_points) - 1):
        cc = cubcoef(all_points[i], all_points[i+1], 0, 0)
        coefficients.append(cc)

    t, pos, vel, acc = generate_path(np.array(coefficients), duration, update_rate)

    return t, np.rad2deg(pos), np.rad2deg(vel), np.rad2deg(acc)

# Define the points
initial_point = np.array([0.758, 0.173, 0.0])
via_point1 = np.array([0.6, -0.3, 45.0])
via_point2 = np.array([-0.4, 0.3, 120.0])
final_point = np.array([0.758, 0.173, 0.0])

# Define manipulator parameters
l1 = l2 = 0.5

# Convert points to joint angles
initial_joint_angles = np.array([0, 0, 0])  # Initial joint angles
via_joint_angles1 = np.array([0, 0, 0])  # Adjust these based on your heuristic
via_joint_angles2 = np.array([0, 0, 0])  # Adjust these based on your heuristic
final_joint_angles = np.array([0, 0, 0])  # Final joint angles

# Plan and execute the path
duration_per_segment = 3.0
update_rate = 40  # Updated update rate

t, pos, vel, acc = plan_and_execute_path(
    initial_joint_angles,
    [via_joint_angles1, via_joint_angles2],
    final_joint_angles,
    duration_per_segment,
    update_rate
)

# Plot the results
plt.figure(figsize=(10, 6))
plt.subplot(3, 1, 1)
plt.plot(t, pos)
plt.title('Joint Position')

plt.subplot(3, 1, 2)
plt.plot(t, vel)
plt.title('Joint Velocity')

plt.subplot(3, 1, 3)
plt.plot(t, acc)
plt.title('Joint Acceleration')

plt.tight_layout()
plt.show()

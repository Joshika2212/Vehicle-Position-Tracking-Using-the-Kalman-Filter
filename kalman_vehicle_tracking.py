import numpy as np
import matplotlib.pyplot as plt
import csv

np.random.seed(42)

dt = 1.0
num_steps = 100
initial_position = 0.0
true_velocity = 15.0
acceleration_noise_std = 0.5
measurement_noise_std = 8.0

actual_positions = np.zeros(num_steps)
actual_velocities = np.zeros(num_steps)
measurements = np.zeros(num_steps)

actual_positions[0] = initial_position
actual_velocities[0] = true_velocity

for k in range(1, num_steps):
    acceleration = np.random.normal(0, acceleration_noise_std)
    actual_velocities[k] = actual_velocities[k - 1] + acceleration * dt
    actual_positions[k] = (
        actual_positions[k - 1] + actual_velocities[k - 1] * dt
        + 0.5 * acceleration * dt**2
    )

measurements = actual_positions + np.random.normal(
    0, measurement_noise_std, num_steps
)

F = np.array([
    [1, dt],
    [0, 1]
])

H = np.array([
    [1, 0]
])

Q = (acceleration_noise_std**2) * np.array([
    [dt**4 / 4, dt**3 / 2],
    [dt**3 / 2, dt**2]
])

R = np.array([[measurement_noise_std**2]])

state = np.array([
    [measurements[0]],
    [0.0]
])

P = np.array([
    [100.0, 0.0],
    [0.0, 100.0]
])

estimated_positions = np.zeros(num_steps)
estimated_velocities = np.zeros(num_steps)

I = np.eye(2)

for k in range(num_steps):
    if k > 0:
        state = F @ state
        P = F @ P @ F.T + Q

    predicted_state = state.copy()

    innovation = np.array([[measurements[k]]]) - H @ state
    S = H @ P @ H.T + R
    K = P @ H.T @ np.linalg.inv(S)

    state = state + K @ innovation
    P = (I - K @ H) @ P

    estimated_positions[k] = state[0, 0]
    estimated_velocities[k] = state[1, 0]

measurement_rmse = np.sqrt(np.mean((measurements - actual_positions) ** 2))

position_rmse = np.sqrt(np.mean((estimated_positions - actual_positions) ** 2))

velocity_rmse = np.sqrt(np.mean((estimated_velocities - actual_velocities) ** 2))

improvement = ((measurement_rmse - position_rmse) / measurement_rmse) * 100

print("KALMAN FILTER VEHICLE TRACKING RESULTS")
print("--------------------------------------")
print(f"Measurement RMSE: {measurement_rmse:.2f} m")
print(f"Filtered Position RMSE: {position_rmse:.2f} m")
print(f"Estimated Velocity RMSE: {velocity_rmse:.2f} m/s")
print(f"Position Error Reduction: {improvement:.2f}%")

print("\nFirst 10 Results")
print("Time | Actual Position | GPS Measurement | Estimated Position | Estimated Velocity")

for k in range(10):
    print(
        f"{k:4d} | {actual_positions[k]:15.2f} | "
        f"{measurements[k]:16.2f} | "
        f"{estimated_positions[k]:18.2f} | "
        f"{estimated_velocities[k]:18.2f}"
    )

with open("kalman_results.csv", "w", newline="") as file:
    writer = csv.writer(file)
    writer.writerow([
        "Time", "Actual Position", "GPS Measurement",
        "Estimated Position", "Actual Velocity", "Estimated Velocity"
    ])

    for k in range(num_steps):
        writer.writerow([
            k, actual_positions[k], measurements[k],
            estimated_positions[k], actual_velocities[k],
            estimated_velocities[k]
        ])

plt.figure(figsize=(12, 6))
plt.plot(actual_positions, label="Actual Position", linewidth=2)
plt.scatter(range(num_steps), measurements,label="Noisy GPS Measurements", s=15)
plt.plot(estimated_positions,label="Kalman Filter Estimate", linewidth=2)
plt.xlabel("Time Step")
plt.ylabel("Position (metres)")
plt.title("Vehicle Position Tracking Using Kalman Filter")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig("kalman_position_tracking.png", dpi=300)

plt.figure(figsize=(12, 6))
plt.plot(actual_velocities, label="Actual Velocity", linewidth=2)
plt.plot(estimated_velocities,label="Estimated Velocity", linewidth=2)
plt.xlabel("Time Step")
plt.ylabel("Velocity (m/s)")
plt.title("Vehicle Velocity Estimation Using Kalman Filter")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig("kalman_velocity_tracking.png", dpi=300)

plt.show()
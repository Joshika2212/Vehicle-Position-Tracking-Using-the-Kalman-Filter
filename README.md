# Vehicle Position Tracking Using the Kalman Filter

## Overview

This project demonstrates the application of the Kalman Filter to estimate the position and velocity of a moving vehicle from noisy sensor measurements.

A simulated vehicle moves along a straight road while random acceleration disturbances affect its motion. Noisy position measurements are generated to represent an imperfect GPS or radar sensor. The Kalman Filter combines motion predictions with sensor measurements to produce improved state estimates.

The project is implemented in Python using NumPy and Matplotlib.

## Objectives

- Understand the working principle of the Kalman Filter.
- Simulate vehicle movement and noisy position measurements.
- Estimate vehicle position and velocity.
- Implement the prediction and correction stages.
- Compare raw measurements with filtered estimates.
- Evaluate performance using Root Mean Square Error (RMSE).

## Technologies Used

- Python
- NumPy
- Matplotlib
- CSV module

## Working Principle

The Kalman Filter is a recursive state-estimation algorithm that estimates the state of a dynamic system using a mathematical model and noisy measurements.

It operates in two main stages:

1. **Prediction:** Predicts the next vehicle state and its uncertainty using the motion model.
2. **Correction:** Uses the latest sensor measurement and the Kalman Gain to update the predicted state.

These steps repeat whenever a new measurement arrives.

### State Vector

The vehicle state consists of position and velocity:

$$
X_k = \begin{bmatrix} p_k \\ v_k \end{bmatrix}
$$

Where:

- $p_k$ is the vehicle position in metres.
- $v_k$ is the vehicle velocity in metres per second.

### Prediction Equations

$$
\hat{X}_k^- = F\hat{X}_{k-1}
$$

$$
P_k^- = FP_{k-1}F^T + Q
$$

### Correction Equations

$$
K_k = P_k^-H^T(HP_k^-H^T + R)^{-1}
$$

$$
\hat{X}_k = \hat{X}_k^- + K_k(z_k-H\hat{X}_k^-)
$$

Here, $Q$ represents process-noise covariance, $R$ represents measurement-noise covariance, $P$ represents estimation uncertainty, and $K$ is the Kalman Gain.

## Project Structure

```text
kalman-filter-vehicle-tracking/
├── kalman_vehicle_tracking.py
├── README.md
├── kalman_results.csv
├── kalman_position_tracking.png
└── kalman_velocity_tracking.png
```

The CSV and graph files are generated when the Python program runs.

## Installation

Make sure Python 3 is installed.

Install the required libraries:

```bash
python -m pip install numpy matplotlib
```

## How to Run

Clone the repository:

```bash
git clone https://github.com/YOUR-USERNAME/kalman-filter-vehicle-tracking.git
```

Open the project directory:

```bash
cd kalman-filter-vehicle-tracking
```

Run the program:

```bash
python kalman_vehicle_tracking.py
```

Replace `YOUR-USERNAME` with your GitHub username.

## Expected Outputs

### Vehicle Position Tracking

Compares the actual vehicle position, noisy sensor measurements, and Kalman Filter estimates.

### Vehicle Velocity Estimation

Compares the actual simulated velocity with the estimated velocity.

### Performance Metrics

The program calculates:

- Measurement RMSE
- Filtered position RMSE
- Estimated velocity RMSE
- Percentage reduction in position RMSE

A lower filtered position RMSE than measurement RMSE indicates improved position estimation for that simulation.

## Applications

- GPS navigation
- Vehicle tracking
- Autonomous driving
- Robot localization
- Radar-based object tracking
- Sensor fusion

## Limitations

- The standard Kalman Filter assumes a linear system model.
- Accuracy depends on appropriate motion and noise models.
- Sudden changes in vehicle movement may reduce accuracy.
- Incorrect noise settings can produce poor estimates.

## Future Enhancements

- Extend the model to two-dimensional vehicle tracking.
- Use real GPS trajectory data.
- Combine GPS with other sensor measurements.
- Compare the Kalman Filter with other estimation algorithms.
- Develop a real-time tracking visualization.

## Conclusion

This project demonstrates how the Kalman Filter can estimate vehicle position and velocity by combining motion predictions with noisy sensor measurements. The Python implementation generates visualizations and calculates RMSE to evaluate estimation performance.

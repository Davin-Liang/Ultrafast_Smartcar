import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
from math import cos, sin, atan2, sqrt

def load_path(file):
    df = pd.read_csv(file)
    return df['x'].values, df['y'].values, df['yaw'].values

def compute_cumulative_distance(x, y):
    dist = [0.0]
    for i in range(1, len(x)):
        d = np.hypot(x[i] - x[i-1], y[i] - y[i-1])
        dist.append(dist[-1] + d)
    return np.array(dist)

def wrap_to_pi(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

def interpolate_actual_path(x, y, yaw, ref_s):
    s = compute_cumulative_distance(x, y)
    fx = interp1d(s, x, kind='linear', fill_value='extrapolate')
    fy = interp1d(s, y, kind='linear', fill_value='extrapolate')
    fyaw = interp1d(s, yaw, kind='linear', fill_value='extrapolate')
    return fx(ref_s), fy(ref_s), fyaw(ref_s)

def compute_lateral_error(x_ref, y_ref, yaw_ref, x_act, y_act):
    dx = x_act - x_ref
    dy = y_act - y_ref
    lat_error = -sin(yaw_ref) * dx + cos(yaw_ref) * dy
    return lat_error

def compute_heading_error(yaw_ref, yaw_act):
    return wrap_to_pi(yaw_act - yaw_ref)

def main():
    # Load paths
    x_ref, y_ref, yaw_ref = load_path("planned_path.csv")
    x_act_raw, y_act_raw, yaw_act_raw = load_path("actual_path.csv")

    # Reference path cumulative distance
    s_ref = compute_cumulative_distance(x_ref, y_ref)

    # Interpolate actual path to match ref
    x_act, y_act, yaw_act = interpolate_actual_path(x_act_raw, y_act_raw, yaw_act_raw, s_ref)

    # Compute errors
    lateral_errors = compute_lateral_error(x_ref, y_ref, yaw_ref, x_act, y_act)
    heading_errors = compute_heading_error(yaw_ref, yaw_act)

    # Metrics
    avg_lat_error = np.mean(np.abs(lateral_errors))
    max_lat_error = np.max(np.abs(lateral_errors))
    yaw_std = np.std(heading_errors)

    print("📏 平均横向误差: {:.4f} m".format(avg_lat_error))
    print("📏 最大横向误差: {:.4f} m".format(max_lat_error))
    print("🧭 航向误差标准差: {:.4f} rad".format(yaw_std))

    # Optional: plot error over distance
    plt.figure(figsize=(10,5))
    plt.subplot(1,2,1)
    plt.plot(s_ref, lateral_errors)
    plt.title("Lateral Error")
    plt.xlabel("Distance along path [m]")
    plt.ylabel("Lateral error [m]")

    plt.subplot(1,2,2)
    plt.plot(s_ref, heading_errors)
    plt.title("Heading Error")
    plt.xlabel("Distance along path [m]")
    plt.ylabel("Heading error [rad]")

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()

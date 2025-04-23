import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d

def load_csv(path):
    df = pd.read_csv(path)
    return df["x"].values, df["y"].values, df["yaw"].values

def align_and_analyze(ref_x, ref_y, ref_yaw, actual_x, actual_y, actual_yaw, label):
    ref_s = np.insert(np.cumsum(np.hypot(np.diff(ref_x), np.diff(ref_y))), 0, 0)
    actual_s = np.insert(np.cumsum(np.hypot(np.diff(actual_x), np.diff(actual_y))), 0, 0)

    # 插值对齐
    if len(ref_s) >= len(actual_s):
        fx = interp1d(actual_s, actual_x, kind='linear', fill_value="extrapolate")
        fy = interp1d(actual_s, actual_y, kind='linear', fill_value="extrapolate")
        fyaw = interp1d(actual_s, actual_yaw, kind='linear', fill_value="extrapolate")
        interp_s = ref_s
        actual_x = fx(interp_s)
        actual_y = fy(interp_s)
        actual_yaw = fyaw(interp_s)
        ref_x, ref_y, ref_yaw = ref_x, ref_y, ref_yaw
    else:
        fx = interp1d(ref_s, ref_x, kind='linear', fill_value="extrapolate")
        fy = interp1d(ref_s, ref_y, kind='linear', fill_value="extrapolate")
        fyaw = interp1d(ref_s, ref_yaw, kind='linear', fill_value="extrapolate")
        interp_s = actual_s
        ref_x = fx(interp_s)
        ref_y = fy(interp_s)
        ref_yaw = fyaw(interp_s)
        actual_x, actual_y, actual_yaw = actual_x, actual_y, actual_yaw

    # 误差计算
    lateral_errors = []
    heading_errors = []
    for i in range(len(interp_s)):
        dx = np.cos(ref_yaw[i] + np.pi / 2)
        dy = np.sin(ref_yaw[i] + np.pi / 2)
        vx = actual_x[i] - ref_x[i]
        vy = actual_y[i] - ref_y[i]
        lateral_errors.append(vx * dx + vy * dy)

        yaw_diff = actual_yaw[i] - ref_yaw[i]
        yaw_diff = np.arctan2(np.sin(yaw_diff), np.cos(yaw_diff))
        heading_errors.append(np.degrees(yaw_diff))

    print(f"\n====== {label} Path Analysis ======")
    print(f"Mean Absolute Lateral Error: {np.mean(np.abs(lateral_errors)):.3f} m")
    print(f"Max Lateral Error: {np.max(np.abs(lateral_errors)):.3f} m")
    print(f"Lateral Error STD: {np.std(lateral_errors):.3f} m")
    print(f"Heading Error STD: {np.std(heading_errors):.3f} deg")

    # 2D 路径对比图
    plt.figure(figsize=(8, 6))
    plt.plot(ref_x, ref_y, label="Reference Path", linewidth=2)
    plt.plot(actual_x, actual_y, label="Actual Path", linewidth=2, linestyle='--')
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.title(f"{label} Path Comparison")
    plt.legend()
    plt.grid(True)
    plt.axis("equal")
    plt.tight_layout()
    plt.savefig(f"{label.lower()}_path_comparison.png")

    # 误差图
    fig, ax1 = plt.subplots(figsize=(10, 5))
    ax1.set_xlabel("Path Length (m)")
    ax1.set_ylabel("Lateral Error (m)", color='tab:blue')
    ax1.plot(interp_s, lateral_errors, color='tab:blue')
    ax1.tick_params(axis='y', labelcolor='tab:blue')
    ax1.grid(True)

    ax2 = ax1.twinx()
    ax2.set_ylabel("Heading Error (deg)", color='tab:red')
    ax2.plot(interp_s, heading_errors, color='tab:red')
    ax2.tick_params(axis='y', labelcolor='tab:red')

    plt.title(f"{label} Lateral & Heading Error")
    fig.tight_layout()
    plt.savefig(f"{label.lower()}_error_comparison.png")

if __name__ == "__main__":
    path_pairs = [
        ("planned_path.csv", "actual_path.csv", "Hybird A*"),
        ("bspline_path.csv", "actual_bspline_path.csv", "Bspline")
    ]

    for ref_path, actual_path, label in path_pairs:
        try:
            ref_x, ref_y, ref_yaw = load_csv(ref_path)
            act_x, act_y, act_yaw = load_csv(actual_path)
            align_and_analyze(ref_x, ref_y, ref_yaw, act_x, act_y, act_yaw, label)
        except Exception as e:
            print(f"处理 {label} 时出错：{e}")

    plt.show()

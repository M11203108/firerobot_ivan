import numpy as np
import matplotlib.pyplot as plt
import csv
import os

# === 常數與參數 ===
g = 9.81
v0 = 6.28
pitch_range = (-25, 35)
yaw_range = (-81, 81)
colors = plt.cm.viridis(np.linspace(0, 1, 13))

# === 火源參數 ===
cx, cy, cz = -3.5, 0.0, 0.0
lx, ly, lz = 1.5, 1.5, 0.2
hx, hy, hz = -3.5, 0.0, 0.0

def nozzle_height(pitch_deg):
    theta = np.radians(pitch_deg)
    offset = np.arcsin(6.0 / 22.5)
    return (167 + 22.5 * np.sin(theta + offset)) / 100

def compute_distance(pitch_rad, h0):
    t_land = (v0 * np.sin(pitch_rad) + np.sqrt((v0 * np.sin(pitch_rad))**2 + 2 * g * h0)) / g
    return v0 * np.cos(pitch_rad) * t_land

def xy_landing(pitch_deg, yaw_deg):
    pitch_rad = np.radians(pitch_deg)
    yaw_rad = np.radians(yaw_deg)
    h0 = nozzle_height(pitch_deg)
    t_land = (v0 * np.sin(pitch_rad) + np.sqrt((v0 * np.sin(pitch_rad))**2 + 2 * g * h0)) / g
    d = v0 * np.cos(pitch_rad) * t_land
    x = -d * np.cos(yaw_rad)
    y = d * np.sin(yaw_rad)
    return x, y

def save_trajectory_to_csv(trajectory_list, filename="trajectory_angles.csv"):
    base_path = os.path.dirname(os.path.abspath(__file__))
    file_path = os.path.join(base_path, filename)
    with open(file_path, mode='w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['pitch', 'yaw'])
        for pitch, yaw in trajectory_list:
            writer.writerow([pitch, yaw])
    print(f"[INFO] Saved {len(trajectory_list)} points to {file_path}")

# === 火源邊界 ===
margin = 0.2
x_min, x_max = cx - lx/2 - margin, cx + lx/2 + margin
y_min, y_max = cy - ly/2 - margin, cy + ly/2 + margin

# === 先粗略掃描 pitch 計算距離 ===
initial_pitch_values = np.linspace(pitch_range[0], pitch_range[1], 30)
pitch_subdivision_counts = []

for pitch_deg in initial_pitch_values:
    x_est, y_est = xy_landing(pitch_deg, 0)  # yaw=0 作預估
    dist_x = abs(x_est - cx)  # 只看 X 軸差距

    # 根據 x 距離決定此 pitch 細分數
    if dist_x < 0.2:
        sub_count = 3
    elif dist_x < 0.6:
        sub_count = 2
    else:
        sub_count = 1
    pitch_subdivision_counts.append(sub_count)

# === 建立 pitch 值（每段自訂細分） ===
pitch_values = []
for i in range(len(initial_pitch_values)-1):
    p_start = initial_pitch_values[i]
    p_end = initial_pitch_values[i+1]
    count = pitch_subdivision_counts[i]
    sub_pitches = np.linspace(p_start, p_end, count, endpoint=False)
    pitch_values.extend(sub_pitches)
pitch_values.append(initial_pitch_values[-1])  # 最後一點

print(f"[INFO] Total pitch points: {len(pitch_values)}")

# === 掃描 ===
yaw_values = np.linspace(yaw_range[0], yaw_range[1], 200)
trajectory_x, trajectory_y = [], []
trajectory_list = []

toggle = True
for pitch_deg in pitch_values:
    yaws = yaw_values if toggle else yaw_values[::-1]
    for yaw_deg in yaws:
        x, y = xy_landing(pitch_deg, yaw_deg)
        if x_min <= x <= x_max and y_min <= y <= y_max:
            trajectory_x.append(x)
            trajectory_y.append(y)
            trajectory_list.append((pitch_deg, yaw_deg))
    toggle = not toggle

save_trajectory_to_csv(trajectory_list)

# === 畫圖 ===
fig2, ax2 = plt.subplots(figsize=(10, 6))

for i, pitch_deg in enumerate(np.linspace(pitch_range[0], pitch_range[1], 13)):
    pitch_rad = np.radians(pitch_deg)
    h0 = nozzle_height(pitch_deg)
    d = compute_distance(pitch_rad, h0)
    xs, ys = [], []
    for yaw_deg in np.linspace(*yaw_range, 25):
        yaw_rad = np.radians(yaw_deg)
        x = -d * np.cos(yaw_rad)
        y = d * np.sin(yaw_rad)
        xs.append(x)
        ys.append(y)
    ax2.scatter(xs, ys, color=colors[i%len(colors)], s=10, label=f"{pitch_deg}°")

ax2.plot(trajectory_x, trajectory_y, color='deepskyblue', linewidth=2, label='Sweep Trajectory')

rect = plt.Rectangle((cx - lx/2, cy - ly/2), lx, ly, edgecolor='gray', facecolor='none', linewidth=2, linestyle='--', label='Fire Box')
ax2.add_patch(rect)
ax2.scatter(hx, hy, color='red', s=70)
ax2.scatter(0, 0, color='blue', s=70, marker='x')

ax2.set_title("XY Plane: Landing Points + Sweep Trajectory")
ax2.set_xlabel("x (Forward, m)")
ax2.set_ylabel("y (Left-Right, m)")
ax2.set_xlim(-7, 0.5)
ax2.set_ylim(-6, 6)
ax2.set_aspect('equal')
ax2.grid(True)
ax2.legend()

plt.show()

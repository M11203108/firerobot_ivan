import numpy as np
import matplotlib.pyplot as plt
import csv
import os

# === 常數與參數 ===
g = 9.81
v0 = 6.28
pitch_range = (-25, 35)
yaw_range = (-81, 81)
pitch_angles_deg = np.linspace(pitch_range[0], pitch_range[1], 13)
colors = plt.cm.viridis(np.linspace(0, 1, len(pitch_angles_deg)))

# === 火源參數 ===
# cx, cy, cz = -2.0, 2.5, 0.0
# lx, ly, lz = 1.5, 1.5, 0.2
# hx, hy, hz = -2.5, 2.0, 0.0

cx, cy, cz = -3.5, 0.0, 0.0
lx, ly, lz = 1.5, 1.5, 0.2
hx, hy, hz = -3.5, 0.0, 0.0

# cx, cy, cz = -3.5, 0.0, 0.0
# lx, ly, lz = 0.5, 0.5, 0.2
# hx, hy, hz = -3.5, 0.0, 0.0

base_corners = np.array([
    [cx - lx/2, cy - ly/2, cz - lz/2],
    [cx - lx/2, cy + ly/2, cz - lz/2],
    [cx + lx/2, cy - ly/2, cz - lz/2],
    [cx + lx/2, cy + ly/2, cz - lz/2],
])

# === 定義函式 ===
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

# === 檔案儲存 ===
def save_trajectory_to_csv(trajectory_list, filename="trajectory_angles.csv"):
    base_path = os.path.dirname(os.path.abspath(__file__))  # 取得這個檔案的目錄
    save_dir = os.path.join(base_path)  # 存在同一層，也可以指定其他目錄
    file_path = os.path.join(save_dir, filename)
    
    with open(file_path, mode='w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['pitch', 'yaw'])
        for pitch, yaw in trajectory_list:
            writer.writerow([pitch, yaw])
    print(f"[INFO] Saved {len(trajectory_list)} points to {file_path}")


# === 建立火源覆蓋邊界 ===
margin = 0.2
x_min, x_max = cx - lx/2 - margin, cx + lx/2 + margin
y_min, y_max = cy - ly/2 - margin, cy + ly/2 + margin

pitch_values = np.linspace(pitch_range[0], pitch_range[1], 40)
yaw_values = np.linspace(yaw_range[0], yaw_range[1], 200)

trajectory_x, trajectory_y = [], []
trajectory_list = []  # <== 新增：記錄 (pitch, yaw)

toggle = True
for pitch_deg in pitch_values:
    yaws = yaw_values if toggle else yaw_values[::-1]
    for yaw_deg in yaws:
        x, y = xy_landing(pitch_deg, yaw_deg)
        if x_min <= x <= x_max and y_min <= y <= y_max:
            trajectory_x.append(x)
            trajectory_y.append(y)
            trajectory_list.append((pitch_deg, yaw_deg))  # <== 新增：把角度也記錄起來
    toggle = not toggle

# === 儲存軌跡資料到 CSV ===
save_trajectory_to_csv(trajectory_list)

# === 畫 XY 平面圖（加入原本圖與軌跡）===
fig2, ax2 = plt.subplots(figsize=(10, 6))

for i, pitch_deg in enumerate(pitch_angles_deg):
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
    ax2.scatter(xs, ys, color=colors[i], s=10, label=f"{pitch_deg}°")

# 加入連續掃描軌跡
ax2.plot(trajectory_x, trajectory_y, color='deepskyblue', linewidth=2, label='Sweep Trajectory')

# 火源區塊與標記
rect = plt.Rectangle((cx - lx/2, cy - ly/2), lx, ly, edgecolor='gray', facecolor='none', linewidth=2, linestyle='--', label='Fire Box')
ax2.add_patch(rect)
ax2.scatter(hx, hy, color='red', s=70)
ax2.scatter(0, 0, color='blue', s=70, marker='x')

# 圖設置
ax2.set_title("XY Plane: Landing Points + Sweep Trajectory")
ax2.set_xlabel("x (Forward, m)")
ax2.set_ylabel("y (Left-Right, m)")
ax2.set_xlim(-7, 0.5)
ax2.set_ylim(-6, 6)
ax2.set_aspect('equal')
ax2.grid(True)
ax2.legend()

plt.show()

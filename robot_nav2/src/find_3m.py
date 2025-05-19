import yaml
import cv2
import numpy as np
import matplotlib.pyplot as plt
import math

def load_map(yaml_path, pgm_path):
    with open(yaml_path, 'r') as f:
        map_info = yaml.safe_load(f)
    resolution = map_info['resolution']
    origin = map_info['origin']
    img = cv2.imread(pgm_path, cv2.IMREAD_UNCHANGED)
    # img = cv2.flip(img, 0)
    return img, resolution, origin


def world_to_map(x, y, origin, resolution, map_img):
    mx = int((x - origin[0]) / resolution)
    my = map_img.shape[0] - int((y - origin[1]) / resolution)
    return mx, my

def generate_circle_points(center_xy, radius, num_points):
    """
    以世界座標中心 `center_xy` 為中心，產生圓周上的點
    """
    cx, cy = center_xy
    points = []
    for i in range(num_points):
        angle = 2 * math.pi * i / num_points
        x = cx + radius * math.cos(angle)
        y = cy + radius * math.sin(angle)
        points.append((x, y))
    return points

def is_area_navigable(x, y, map_img, origin, resolution, radius):
    """
    判斷圓形區域是否可通行
    """
    mx_center, my_center = world_to_map(x, y, origin, resolution, map_img)
    pixel_radius = int(radius / resolution)
    h, w = map_img.shape

    for dx in range(-pixel_radius, pixel_radius + 1):
        for dy in range(-pixel_radius, pixel_radius + 1):
            if dx**2 + dy**2 <= pixel_radius**2:
                mx = mx_center + dx
                my = my_center + dy
                if 0 <= mx < w and 0 <= my < h:
                    if map_img[my, mx] < 254:  # ✅ 只允許完全白色
                        return False
                else:
                    return False  # 出界也視為不可行
    return True

def find_clear_line(p1, p2, map_img, origin, resolution, line_width=1.0):
    """
    檢查從 p1 到 p2 之間是否有障礙物。
    p1, p2 是 (x, y) 世界座標
    """
    mx1, my1 = world_to_map(p1[0], p1[1], origin, resolution, map_img)
    mx2, my2 = world_to_map(p2[0], p2[1], origin, resolution, map_img)

    mask = np.zeros_like(map_img, dtype=np.uint8)

    # 線寬轉為像素
    line_thickness = max(1, int((line_width / resolution)))

    # 畫白色線
    cv2.line(mask, (mx1, my1), (mx2, my2), 255, thickness=line_thickness)

    # 找出線上所有的像素 (非0)
    ys, xs = np.where(mask > 0)

    # 檢查線上是否有障礙物（<254）
    for (x, y) in zip(xs, ys):
        if map_img[y, x] < 254:
            return False
    return True

def find_nearest_navigable_point(points, robot_pos, map_img, origin, resolution):
    """
    找到距離機器人最近的可導航點
    """
    valid_points = []
    for point in points:
        if is_area_navigable(point[0], point[1], map_img, origin, resolution, radius=1.0):
            dist = math.hypot(point[0] - robot_pos[0], point[1] - robot_pos[1])
            valid_points.append((dist,point))
    if valid_points:
        valid_points.sort()
        return valid_points[0][1]
    else:
        return None
    
def draw_result_map(map_img, resolution, origin, fire_pos, robot_pos, target_pos, valid_points, invalid_points):
    height, width = map_img.shape
    extent = [
        origin[0],                      # xmin = origin x
        origin[0] + width * resolution, # xmax = origin x + width
        origin[1],                      # ymin = origin y
        origin[1] + height * resolution # ymax = origin y + height
    ]

    fig, ax = plt.subplots(figsize=(10, 10))
    ax.imshow(map_img, cmap='gray', extent=extent, origin='upper')  # ← 這裡關鍵

    # 標示點位
    ax.plot(fire_pos[0], fire_pos[1], 'ro', label='🔥 Fire Point')
    ax.plot(robot_pos[0], robot_pos[1], 'bo', label='🤖 Robot Start')
    ax.plot(target_pos[0], target_pos[1], 'yo', markersize=8, label='✅ Target Point')

    # 畫出圓周點（小綠點）
    # if valid_points:
    #     valid_x = [pt[0] for pt in valid_points]
    #     valid_y = [pt[1] for pt in valid_points]
    #     ax.plot(valid_x, valid_y, 'go', markersize=4, label='🟢 Valid Points')

    # # 畫出無效點（小紅點）
    # if invalid_points:
    #     invalid_x = [pt[0] for pt in invalid_points]
    #     invalid_y = [pt[1] for pt in invalid_points]
    #     ax.plot(invalid_x, invalid_y, 'ro', markersize=4, label='🔴 Invalid Points')

    # 標題與軸標籤
    ax.set_title('Map with Real-World Coordinates')
    ax.set_xlabel('X (meters)')
    ax.set_ylabel('Y (meters)')
    ax.legend()
    ax.grid(True)
    plt.show()


def main():
    # 替換為你的地圖路徑
    yaml_path = "/home/robot/ivan_ws/src/robot_nav2/maps/map.yaml"
    pgm_path = "/home/robot/ivan_ws/src/robot_nav2/maps/map.pgm"
    
    # 載入地圖與資訊
    img, resolution, origin = load_map(yaml_path, pgm_path)  # 改成 False
    print(f"地圖大小: {img.shape}")
    print(f"解析度: {resolution}")
    print(f"原點: {origin}")
    # 假設熱點位置（世界座標）
    hotspot_world = (4.26, -0.73)  # 單位：meter
    robot_origin_world = (0, 0)  # 單位：meter

    circle_points = generate_circle_points(hotspot_world, radius=3.0, num_points=72)

    valid_points = []
    invalid_points = []
    for point in circle_points:
        if is_area_navigable(point[0], point[1], img, origin, resolution, radius=1.0):
            if find_clear_line(point, hotspot_world, img, origin, resolution):
                valid_points.append(point)
            else:
                invalid_points.append(point)
        else:
            invalid_points.append(point)

    best_point = find_nearest_navigable_point(valid_points, robot_origin_world, img, origin, resolution)

    if best_point:
        print(f"✅ 找到的最佳導航點: {best_point}")
        draw_result_map(img, resolution, origin, hotspot_world, robot_origin_world, best_point, valid_points, invalid_points)
    else:
        print("❌ 沒有找到可導航的點")


if __name__ == "__main__":
    main()

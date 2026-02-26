#!/usr/bin/env python3
import os
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

def create_map_from_pcd(pcd_path, output_image_path, yaml_output_path,
                        resolution=0.05, z_min=0.10, z_max=1.50):
    pcd = o3d.io.read_point_cloud(pcd_path)
    points = np.asarray(pcd.points)
    print(f"✅ Loaded {len(points)} points from {pcd_path}")

    if points.size == 0:
        raise ValueError("입력 포인트가 비어 있습니다.")

    z = points[:, 2]
    mask = (z >= z_min) & (z <= z_max)
    filtered_points = points[mask]
    print(f"✅ Filtered points (z in [{z_min}, {z_max}]): {len(filtered_points)}")

    if filtered_points.size == 0:
        raise ValueError("필터 후 포인트가 없습니다. z 범위를 확인하세요.")

    x_min, x_max = np.min(filtered_points[:, 0]), np.max(filtered_points[:, 0])
    y_min, y_max = np.min(filtered_points[:, 1]), np.max(filtered_points[:, 1])

    width  = int(np.ceil((x_max - x_min) / resolution)) + 1
    height = int(np.ceil((y_max - y_min) / resolution)) + 1

    bev = np.full((height, width), 255, np.uint8)

    # 점 -> 픽셀 투영
    for (x, y, _) in filtered_points:
        px = int((x - x_min) / resolution)
        py = height - 1 - int((y - y_min) / resolution)
        if 0 <= px < width and 0 <= py < height:
            bev[py, px] = 0

    # 이미지 저장 (그레이스케일)
    plt.imsave(output_image_path, bev, cmap="gray", vmin=0, vmax=255)
    print(f"✅ Saved BEV image → {output_image_path}")

    # ROS map_server용 YAML (image는 파일명만)
    image_basename = os.path.basename(output_image_path)
    yaml_content = (
        f"image: {image_basename}\n"
        f"resolution: {resolution}\n"
        f"origin: [{x_min}, {y_min}, 0.0]\n"
        f"negate: 0\n"
        f"occupied_thresh: 0.65\n"
        f"free_thresh: 0.196\n"
    )

    with open(yaml_output_path, "w") as f:
        f.write(yaml_content)

    print(f"✅ Saved YAML → {yaml_output_path}")
    print("🎉 모든 작업이 완료되었습니다.")

if __name__ == "__main__":
    pcd_input_path = "/home/irop/catkin_ws/src/fast_lio/PCD/scans.pcd"

    out_dir = os.path.dirname(pcd_input_path)
    image_output_path = os.path.join(out_dir, "scans.png")
    yaml_output_path  = os.path.join(out_dir, "scans.yaml")

    map_resolution = 0.05

    create_map_from_pcd(
        pcd_path=pcd_input_path,
        output_image_path=image_output_path,
        yaml_output_path=yaml_output_path,
        resolution=map_resolution,
        z_min=0.10,
        z_max=1.50,
    )


import numpy as np

def save_ply(points, filename):
    """
    points: Nx3 numpy array
    filename: output .ply path
    """
    with open(filename, 'w') as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {points.shape[0]}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write("end_header\n")

        for p in points:
            f.write(f"{p[0]} {p[1]} {p[2]}\n")


if __name__ == "__main__":
    # 修改为你的点云文件
    input_npy = "sfm_points_20251114_171953.npy"
    output_ply = "sfm_points_20251114_171953.ply"

    # 加载 npy 点云
    points = np.load(input_npy)

    # 如果 points 是 Nx4（齐次坐标），去掉最后一列
    if points.shape[1] == 4:
        points = points[:, :3]

    save_ply(points, output_ply)
    print(f"PLY saved to: {output_ply}")

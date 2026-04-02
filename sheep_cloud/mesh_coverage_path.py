"""
网格表面全覆盖路径规划
Full-Coverage Path Planning on Mesh Surface

算法流程:
  1. 解析 PLY 网格文件
  2. PCA 投影 → 展开到最优 2D 平面
  3. 构建网格轮廓（Boundary）
  4. 在 2D 平面内生成 Boustrophedon（弓字形）扫描路径  ← 从左到右
  5. 将 2D 路径通过 Open3D RaycastingScene 快速映射回 3D 网格表面  ← 加速
  6. 插值 + 法向量对齐（用于喷涂/打磨等末端执行器姿态）
  7. 姿态平滑 + 前进方向计算 (切向向量)
  8. 可视化 & 导出

修改说明（v2.1）:
  · 增加了法向量的滑动均值平滑 (smooth_normals)
  · 增加了基于差分的轨迹前进方向计算 (compute_path_directions)
  · CSV 导出增加了 dx, dy, dz 用于机器人末端 X 轴对齐
"""

import numpy as np
import struct
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.collections import LineCollection
from scipy.spatial import KDTree, ConvexHull
from scipy.interpolate import splprep, splev
import open3d as o3d
import open3d.core as o3c

import warnings
warnings.filterwarnings('ignore')


# ─────────────────────────────────────────────
# 1. PLY 解析
# ─────────────────────────────────────────────
def read_ply(path):
    """
    使用 Open3D 读取 PLY 网格文件，支持任意顶点数量和格式。
    返回: verts (N,9), faces (M,3), mesh_o3d (TriangleMesh 原始对象)
          mesh_o3d 供后续快速投影使用，避免重复解析。
    """
    mesh = o3d.io.read_triangle_mesh(path)

    if not mesh.has_vertex_normals():
        mesh.compute_vertex_normals()

    xyz     = np.asarray(mesh.vertices)
    normals = np.asarray(mesh.vertex_normals)

    if mesh.has_vertex_colors():
        colors = np.asarray(mesh.vertex_colors) * 255.0
    else:
        colors = np.zeros_like(xyz)

    verts = np.hstack((xyz, normals, colors))
    faces = np.asarray(mesh.triangles, dtype=np.int32)

    return verts, faces, mesh


# ─────────────────────────────────────────────
# 2. 工具函数 & Open3D 加速投影
# ─────────────────────────────────────────────
def compute_face_normals(xyz, faces):
    fn = []
    for f in faces:
        v0, v1, v2 = xyz[f[0]], xyz[f[1]], xyz[f[2]]
        n = np.cross(v1 - v0, v2 - v0)
        norm = np.linalg.norm(n)
        fn.append(n / norm if norm > 1e-12 else np.array([0, 1, 0]))
    return np.array(fn)


def build_raycasting_scene(mesh_o3d):
    """预构建 Open3D RaycastingScene，用于快速最近点查询"""
    scene   = o3d.t.geometry.RaycastingScene()
    mesh_t  = o3d.t.geometry.TriangleMesh.from_legacy(mesh_o3d)
    scene.add_triangles(mesh_t)
    return scene, mesh_o3d


def point_to_mesh_projection_fast(query_pts, scene, mesh_o3d):
    """将查询点批量投影到网格最近点（全量向量化）"""
    query_t  = o3c.Tensor(query_pts.astype(np.float32), dtype=o3c.float32)
    result   = scene.compute_closest_points(query_t)

    proj_pts   = result['points'].numpy().astype(np.float64)
    tri_ids    = result['primitive_ids'].numpy()

    triangles = np.asarray(mesh_o3d.triangles)
    vertices  = np.asarray(mesh_o3d.vertices)

    tri_verts = triangles[tri_ids]
    v0 = vertices[tri_verts[:, 0]]
    v1 = vertices[tri_verts[:, 1]]
    v2 = vertices[tri_verts[:, 2]]

    face_normals = np.cross(v1 - v0, v2 - v0)
    norms = np.linalg.norm(face_normals, axis=1, keepdims=True)
    safe  = norms > 1e-12
    face_normals = np.where(safe, face_normals / np.where(safe, norms, 1.0),
                            np.array([[0.0, 1.0, 0.0]]))

    return proj_pts, face_normals


def resample_path(pts, target_spacing):
    """按目标间距重采样路径"""
    if len(pts) < 2:
        return pts
    out = [pts[0]]
    accum = 0.0
    for i in range(1, len(pts)):
        seg = pts[i] - pts[i - 1]
        seg_len = np.linalg.norm(seg)
        if seg_len < 1e-12:
            continue
        while accum + seg_len >= target_spacing:
            t = (target_spacing - accum) / seg_len
            out.append(pts[i - 1] + t * seg)
            pts_rem = pts[i - 1] + t * seg
            seg_len -= target_spacing - accum
            pts[i - 1] = pts_rem
            seg = pts[i] - pts[i - 1]
            accum = 0.0
        accum += seg_len
    out.append(pts[-1])
    return np.array(out)


# ─────────────────────────────────────────────
# 3. 核心：Boustrophedon 覆盖路径生成
# ─────────────────────────────────────────────
def boustrophedon_coverage(xyz_2d, hull_pts, step_size, scan_dir='y'):
    """在 2D 凸包区域内生成弓字形扫描路径"""
    from matplotlib.path import Path as MplPath

    if scan_dir == 'y':
        pts  = hull_pts[:, [1, 0]]
        data = xyz_2d[:, [1, 0]]
    else:
        pts  = hull_pts.copy()
        data = xyz_2d.copy()

    min_u = pts[:, 0].min() - step_size * 0.1
    max_u = pts[:, 0].max() + step_size * 0.1
    min_v = pts[:, 1].min()
    max_v = pts[:, 1].max()

    v_lines = np.arange(min_v + step_size / 2, max_v, step_size)
    if len(v_lines) == 0:
        v_lines = np.array([(min_v + max_v) / 2])

    path_segments = []

    for vi, v in enumerate(v_lines):
        crossings = []
        n = len(pts)
        for i in range(n):
            a = pts[i]
            b = pts[(i + 1) % n]
            if (a[1] <= v < b[1]) or (b[1] <= v < a[1]):
                t_c = (v - a[1]) / (b[1] - a[1] + 1e-15)
                u_c = a[0] + t_c * (b[0] - a[0])
                crossings.append(u_c)

        if len(crossings) < 2:
            continue
        crossings.sort()
        u_start, u_end = crossings[0], crossings[-1]

        u_pts = np.arange(u_start, u_end + 1e-9, step_size / 4)
        if len(u_pts) == 0:
            u_pts = np.array([(u_start + u_end) / 2])

        if vi % 2 == 1:
            u_pts = u_pts[::-1]

        row = np.column_stack([u_pts, np.full(len(u_pts), v)])
        path_segments.append(row)

    if not path_segments:
        return np.array([]).reshape(0, 2)

    full_path = []
    for i, seg in enumerate(path_segments):
        full_path.append(seg)
        if i < len(path_segments) - 1:
            conn = np.array([seg[-1], path_segments[i + 1][0]])
            full_path.append(conn)

    path_2d = np.vstack(full_path)

    if scan_dir == 'y':
        path_2d = path_2d[:, [1, 0]]

    return path_2d


# ─────────────────────────────────────────────
# 4. 路径与姿态平滑 & 方向计算
# ─────────────────────────────────────────────
def smooth_path(path_3d, window=5):
    """滑动均值平滑位置"""
    if len(path_3d) < window * 2:
        return path_3d
    smoothed = path_3d.copy()
    for i in range(window, len(path_3d) - window):
        smoothed[i] = path_3d[i - window:i + window + 1].mean(axis=0)
    return smoothed


def smooth_normals(normals, window=8):
    """对法向量进行滑动均值平滑，并重新归一化"""
    if len(normals) < window * 2:
        return normals
    smoothed = normals.copy()
    for i in range(window, len(normals) - window):
        avg_n = normals[i - window : i + window + 1].mean(axis=0)
        norm_len = np.linalg.norm(avg_n)
        if norm_len > 1e-12:
            smoothed[i] = avg_n / norm_len
    return smoothed


def compute_path_directions(path_3d):
    """计算轨迹的前进方向向量 (切向向量)，并归一化"""
    N = len(path_3d)
    directions = np.zeros_like(path_3d)

    if N < 2:
        return directions

    directions[1:-1] = path_3d[2:] - path_3d[:-2]
    directions[0] = path_3d[1] - path_3d[0]
    directions[-1] = path_3d[-1] - path_3d[-2]

    norms = np.linalg.norm(directions, axis=1, keepdims=True)
    safe = norms > 1e-12
    directions = np.where(safe, directions / np.where(safe, norms, 1.0), np.array([[1.0, 0.0, 0.0]]))

    return directions


def smooth_directions(directions, window=8):
    """
    对方向向量进行滑动均值平滑，并重新归一化。
    与 smooth_normals 逻辑一致，单独封装便于独立调用。
    """
    if len(directions) < window * 2 + 1:
        return directions
    smoothed = directions.copy()
    for i in range(window, len(directions) - window):
        avg_d = directions[i - window : i + window + 1].mean(axis=0)
        norm_len = np.linalg.norm(avg_d)
        if norm_len > 1e-12:
            smoothed[i] = avg_d / norm_len
    return smoothed


def filter_direction_outliers(path_3d, path_normals, path_directions,
                               window=10, angle_threshold_deg=45.0):
    """
    在 ±window 邻域内检测方向向量跳变过大的点并删除。

    算法:
      对每个点 i，计算其 [i-window, i+window] 邻域（排除自身）的
      平均方向向量 mean_dir，若 angle(d_i, mean_dir) > angle_threshold，
      则标记为跳变点并移除。

    Parameters
    ----------
    window              : 邻域半径（单侧点数），总窗口为 2*window 个邻居
    angle_threshold_deg : 判定为跳变的角度阈值（°），推荐 30~60°

    Returns
    -------
    path_3d, path_normals, path_directions : 剔除跳变点后的数组
    mask                                   : bool 数组，True 表示保留
    """
    N = len(path_directions)
    if N == 0:
        return path_3d, path_normals, path_directions, np.ones(0, dtype=bool)

    cos_thresh = np.cos(np.radians(angle_threshold_deg))
    mask = np.ones(N, dtype=bool)

    for i in range(N):
        i_start = max(0, i - window)
        i_end   = min(N, i + window + 1)

        # 收集邻域（排除 i 自身）
        neighbor_dirs = np.vstack([
            path_directions[i_start : i],
            path_directions[i + 1  : i_end]
        ]) if i > i_start else path_directions[i + 1 : i_end]

        if len(neighbor_dirs) < 3:          # 邻居太少，不做判断
            continue

        # 邻域均值方向（归一化）
        mean_dir  = neighbor_dirs.mean(axis=0)
        mean_norm = np.linalg.norm(mean_dir)
        if mean_norm < 1e-12:
            continue
        mean_dir /= mean_norm

        # 当前点与邻域均值的夹角余弦
        cos_angle = np.dot(path_directions[i], mean_dir)

        if cos_angle < cos_thresh:
            mask[i] = False

    n_removed = int((~mask).sum())
    print(f"    跳变点检测: 共 {N} 个点，删除 {n_removed} 个"
          f"（{n_removed / N * 100:.2f}%），阈值 {angle_threshold_deg}°")

    return (path_3d[mask],
            path_normals[mask],
            path_directions[mask],
            mask)

# ─────────────────────────────────────────────
# 5. Open3D 交互式可视化
# ─────────────────────────────────────────────
def show_path_on_pointcloud(pcd_path, path_3d, path_normals, normal_length=0.02):
    """使用 Open3D 交互式显示原点云、轨迹和法向"""
    print(f"\n[9] 启动 Open3D 交互式轨迹及姿态(法向)可视化 ...")

    pcd = o3d.io.read_point_cloud(pcd_path)
    if pcd.is_empty():
        print(f"[警告] 无法读取原点云文件: {pcd_path}，请检查路径。将仅显示轨迹。")
    else:
        pcd.paint_uniform_color([0.3, 0.3, 0.3])
        print(f"    成功加载原点云，包含 {len(pcd.points)} 个点。")

    lines = [[i, i + 1] for i in range(len(path_3d) - 1)]
    import matplotlib.pyplot as plt
    cmap = plt.get_cmap("plasma")
    line_colors = [cmap(i / len(lines))[:3] for i in range(len(lines))]

    path_line_set = o3d.geometry.LineSet()
    path_line_set.points = o3d.utility.Vector3dVector(path_3d)
    path_line_set.lines = o3d.utility.Vector2iVector(lines)
    path_line_set.colors = o3d.utility.Vector3dVector(line_colors)

    step = max(1, len(path_3d) // 3000)

    normal_points = []
    normal_lines = []
    normal_colors = []

    for idx, i in enumerate(range(0, len(path_3d), step)):
        start_pt = path_3d[i]
        end_pt = start_pt + path_normals[i] * normal_length

        normal_points.extend([start_pt, end_pt])
        normal_lines.append([2 * idx, 2 * idx + 1])
        normal_colors.append([0.0, 1.0, 1.0])

    normal_line_set = o3d.geometry.LineSet()
    if normal_points:
        normal_line_set.points = o3d.utility.Vector3dVector(np.array(normal_points))
        normal_line_set.lines = o3d.utility.Vector2iVector(np.array(normal_lines))
        normal_line_set.colors = o3d.utility.Vector3dVector(np.array(normal_colors))

    coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])

    print(f"    💡 提示: 渐变色线为走刀轨迹，【青色短线】代表加工法向（每隔 {step} 个点显示一个，长 {normal_length * 100}cm）。")
    print("    💡 操作: 鼠标左键拖拽旋转，滚轮缩放。按 'Q' 或 'ESC' 退出。")
    o3d.visualization.draw_geometries(
        [pcd, path_line_set, normal_line_set, coord_frame],
        window_name="轨迹与法向量姿态检查",
        point_show_normal=False,
        mesh_show_back_face=True
    )


# ─────────────────────────────────────────────
# 6. 路径导出
# ─────────────────────────────────────────────
def export_path(path_3d, path_normals, path_directions, out_csv, out_gcode=None):
    """导出为 CSV（位置+法向+前进方向）和可选 G-code"""
    import csv
    with open(out_csv, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['idx', 'x', 'y', 'z', 'nx', 'ny', 'nz', 'dx', 'dy', 'dz',
                         'seg_len', 'cumulative_len'])
        cum = 0.0
        for i, (pt, n, d) in enumerate(zip(path_3d, path_normals, path_directions)):
            if i > 0:
                cum += np.linalg.norm(pt - path_3d[i - 1])
            seg = np.linalg.norm(pt - path_3d[i - 1]) if i > 0 else 0.0
            writer.writerow([i,
                             f'{pt[0]:.6f}', f'{pt[1]:.6f}', f'{pt[2]:.6f}',
                             f'{n[0]:.6f}',  f'{n[1]:.6f}',  f'{n[2]:.6f}',
                             f'{d[0]:.6f}',  f'{d[1]:.6f}',  f'{d[2]:.6f}',
                             f'{seg:.6f}',   f'{cum:.6f}'])
    print(f"[✓] 路径 CSV 已保存: {out_csv}")

    if out_gcode:
        with open(out_gcode, 'w') as f:
            f.write("; Full-Coverage Path (Surface Machining, Left-to-Right)\n")
            f.write("; Generated by mesh_coverage_path.py v2.1\n")
            f.write(f"; Total points: {len(path_3d)}\n\n")
            f.write("G21        ; mm units\n")
            f.write("G90        ; absolute positioning\n")
            f.write("G28        ; home\n\n")
            prev_line = False
            threshold = 0.05
            for i, pt in enumerate(path_3d):
                mm = pt * 1000
                if i > 0:
                    dist = np.linalg.norm(pt - path_3d[i - 1])
                    is_transition = dist > threshold
                    if is_transition and not prev_line:
                        f.write("\n; -- transition --\n")
                    prev_line = is_transition
                f.write(f"G1 X{mm[0]:.3f} Y{mm[1]:.3f} Z{mm[2]:.3f} F500\n")
            f.write("\nM2 ; end\n")
        print(f"[✓] G-code 已保存: {out_gcode}")


# ─────────────────────────────────────────────
# 7. 主流程
# ─────────────────────────────────────────────
def main():
    print("=" * 60)
    print("  网格表面全覆盖路径规划  v2.1")
    print("  ★ 扫描方向：从左到右  ★ 投影：RaycastingScene 加速")
    print("  ★ 新增: 姿态平滑 & 轨迹切向(前进)向量计算")
    print("=" * 60)

    # ── 加载网格 ──────────────────────────────
    ply_path = 'processed_mesh.ply'
    verts, faces, mesh_o3d = read_ply(ply_path)
    xyz     = verts[:, :3]
    normals = verts[:, 3:6]
    print(f"\n[1] 网格解析完成")
    print(f"    顶点数: {len(xyz)}, 面数: {len(faces)}")

    # ── 预构建 RaycastingScene ────────────────
    print(f"    构建 RaycastingScene ...")
    raycast_scene, _ = build_raycasting_scene(mesh_o3d)

    # ── 计算表面积 & 包围盒 ────────────────────
    total_area = 0.0
    for f in faces:
        v0, v1, v2 = xyz[f[0]], xyz[f[1]], xyz[f[2]]
        total_area += 0.5 * np.linalg.norm(np.cross(v1 - v0, v2 - v0))
    print(f"    总表面积: {total_area:.4f} m²")

    bbox_min  = xyz.min(axis=0)
    bbox_max  = xyz.max(axis=0)
    bbox_size = bbox_max - bbox_min
    print(f"    包围盒: {bbox_size[0]:.3f} × {bbox_size[1]:.3f} × {bbox_size[2]:.3f} m")

    # ── PCA 求投影平面 ────────────────────────
    center   = xyz.mean(axis=0)
    xyz_c    = xyz - center
    cov      = np.cov(xyz_c.T)
    eigvals, eigvecs = np.linalg.eigh(cov)
    idx_sort = np.argsort(eigvals)[::-1]
    eigvals  = eigvals[idx_sort]
    eigvecs  = eigvecs[:, idx_sort]

    # ── 保留 PCA 法向量，但重建与世界 X 轴对齐的坐标系 ──────────────
    proj_normal = eigvecs[:, 2]

    avg_n = normals.mean(axis=0)
    avg_n /= np.linalg.norm(avg_n)
    if np.dot(proj_normal, avg_n) < 0:
        proj_normal = -proj_normal

    # 将世界 X 轴投影到曲面切平面，作为 u_axis（扫描行方向）
    WORLD_X = np.array([1.0, 0.0, 0.0])
    u_axis  = WORLD_X - np.dot(WORLD_X, proj_normal) * proj_normal
    u_norm  = np.linalg.norm(u_axis)

    if u_norm < 1e-6:
        # 极端情况：曲面法向与 X 轴几乎平行，退化为 Y 轴
        print("    [警告] 世界 X 轴接近曲面法向，自动改用世界 Y 轴作为扫描方向")
        WORLD_X = np.array([0.0, 1.0, 0.0])
        u_axis  = WORLD_X - np.dot(WORLD_X, proj_normal) * proj_normal
        u_axis /= np.linalg.norm(u_axis)
    else:
        u_axis /= u_norm

    # v_axis 由法向量叉乘 u_axis 得到，保证右手系
    v_axis = np.cross(proj_normal, u_axis)
    v_axis /= np.linalg.norm(v_axis)

    print(f"\n[2] 坐标系（X 轴对齐模式）")
    print(f"    曲面法向量 n : {proj_normal}")
    print(f"    扫描行方向 u : {u_axis}  ← 与世界 X 轴对齐")
    print(f"    步进方向   v : {v_axis}")


    # ── 投影到 2D ─────────────────────────────
    xyz_2d = np.column_stack([xyz_c @ u_axis, xyz_c @ v_axis])
    hull     = ConvexHull(xyz_2d)
    hull_pts = xyz_2d[hull.vertices]
    print(f"\n[3] 2D 投影完成")

    # ── 路径规划参数 ──────────────────────────
    TOOL_DIAMETER  = 0.04
    OVERLAP_RATIO  = 0.15
    STEP_SIZE      = TOOL_DIAMETER * (1 - OVERLAP_RATIO)
    RESAMPLE_DIST  = STEP_SIZE / 8

    print(f"\n[4] 路径规划参数")
    print(f"    工具直径:   {TOOL_DIAMETER * 100:.1f} cm")
    print(f"    列间距:     {STEP_SIZE * 100:.2f} cm")

    # ── 生成 Boustrophedon 路径 ───────────────
    SCAN_DIR = 'x'
    print(f"\n[5] 生成 Boustrophedon 路径 (扫描方向=从左到右)")
    path_2d = boustrophedon_coverage(xyz_2d, hull_pts, STEP_SIZE, SCAN_DIR)

    if len(path_2d) == 0:
        print("[ERROR] 路径生成失败，请检查参数")
        return

    path_2d = resample_path(path_2d, RESAMPLE_DIST)
    print(f"    重采样后点数: {len(path_2d)}")

    # ── 快速反投影到 3D ───────────────────────
    print(f"\n[6] 将 2D 路径快速投影到 3D 网格表面 ...")
    import time
    path_3d_approx = (path_2d[:, 0:1] * u_axis + path_2d[:, 1:2] * v_axis + center)
    path_3d, path_normals = point_to_mesh_projection_fast(path_3d_approx, raycast_scene, mesh_o3d)

    # ── 路径平滑 & 重算姿态 ───────────────────
    path_3d = smooth_path(path_3d, window=3)
    path_3d, path_normals = point_to_mesh_projection_fast(path_3d, raycast_scene, mesh_o3d)

    # ── 法向平滑 & 方向计算 ───────────────────
    print(f"    执行法向量平滑处理...")
    path_normals = smooth_normals(path_normals, window=10)

    # ── 方向向量计算 → 平滑 → 跳变点剔除 ────────────────────────
    print(f"    计算轨迹前进方向向量(切向)...")
    path_directions = compute_path_directions(path_3d)

    print(f"    执行方向向量滑动均值平滑 (window=8)...")
    path_directions = smooth_directions(path_directions, window=8)

    print(f"    检测并删除邻域内方向跳变点 (window=10, 阈值=45°)...")
    path_3d, path_normals, path_directions, valid_mask = filter_direction_outliers(
        path_3d, path_normals, path_directions,
        window=10,
        angle_threshold_deg=45.0   # 根据曲面曲率调整：平面可用 20°，复杂曲面用 50~60°
    )

    # 剔除后重新平滑一遍，消除边界残留噪声
    path_directions = smooth_directions(path_directions, window=5)

    # ── 统计信息 ──────────────────────────────
    seg_lens  = np.linalg.norm(np.diff(path_3d, axis=0), axis=1)
    total_len = seg_lens.sum()
    coverage  = min(100.0, total_len * TOOL_DIAMETER / total_area * 100)

    print(f"\n[7] 路径统计")
    print(f"    总路径长度: {total_len:.3f} m")
    print(f"    路径点数:   {len(path_3d)}")
    print(f"    理论覆盖率: {coverage:.1f}%")

    # ── 导出 ──────────────────────────────────
    import os
    os.makedirs('outputs', exist_ok=True)
    csv_path   = 'outputs/coverage_path.csv'
    gcode_path = 'outputs/coverage_path.gcode'
    export_path(path_3d, path_normals, path_directions, csv_path, gcode_path)

    # 打印样本
    print(f"\n[样本路径点 (前 5 个)]")
    print(f"{'idx':>4}  {'x':>8} {'y':>8} {'z':>8} | {'nx':>6} {'ny':>6} {'nz':>6} | {'dx':>6} {'dy':>6} {'dz':>6}")
    for i in range(5):
        p, n, d = path_3d[i], path_normals[i], path_directions[i]
        print(f"{i:>4}  {p[0]:>8.4f} {p[1]:>8.4f} {p[2]:>8.4f} | {n[0]:>6.3f} {n[1]:>6.3f} {n[2]:>6.3f} | {d[0]:>6.3f} {d[1]:>6.3f} {d[2]:>6.3f}")

    print(f"\n{'='*60}")
    print("  全覆盖路径规划完成！")
    print(f"{'='*60}")

    # ── 交互式可视化 ──────────────────────────
    show_path_on_pointcloud('cropped_cloud.pcd', path_3d, path_normals, normal_length=0.015)

    return path_3d, path_normals, path_directions


if __name__ == '__main__':
    main()

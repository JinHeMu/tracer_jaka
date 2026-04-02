"""
网格表面全覆盖路径规划 —— 分步可视化版
Full-Coverage Path Planning on Mesh Surface — Step-by-Step Visualization

每个算法步骤都会生成一张独立的可视化图像，保存在 outputs/steps/ 目录下：
  step1_mesh_parse.png          - 原始网格解析
  step2_pca_projection.png      - PCA 主轴与最优投影平面
  step3_2d_boundary.png         - 2D 投影与凸包边界
  step4_boustrophedon.png       - 弓字形扫描路径（2D）
  step5_3d_backprojection.png   - 2D→3D 反投影结果
  step6_smoothing_normals.png   - 路径平滑与法向量修正对比
  step7_final_result.png        - 最终综合结果
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from scipy.spatial import KDTree, ConvexHull
import open3d as o3d
import os
import warnings
warnings.filterwarnings('ignore')

# ─────────────────────────────────────────────────────────────
# 全局样式常量
# ─────────────────────────────────────────────────────────────
DARK      = '#0d1117'
GRID_COL  = '#21262d'
PANE_COL  = '#30363d'
TEXT_COL  = '#c9d1d9'
DIM_COL   = '#8b949e'
BLUE      = '#58a6ff'
GREEN     = '#3fb950'
RED       = '#f85149'
ORANGE    = '#ff9500'
PURPLE    = '#bc8cff'
YELLOW    = '#f0e68c'
CYAN      = '#79c0ff'

OUT_DIR = 'outputs/steps'

def _setup_3d_ax(ax, title=''):
    ax.set_facecolor(DARK)
    ax.xaxis.pane.fill = False
    ax.yaxis.pane.fill = False
    ax.zaxis.pane.fill = False
    ax.xaxis.pane.set_edgecolor(PANE_COL)
    ax.yaxis.pane.set_edgecolor(PANE_COL)
    ax.zaxis.pane.set_edgecolor(PANE_COL)
    ax.tick_params(colors=DIM_COL, labelsize=7)
    ax.set_xlabel('X (m)', color=DIM_COL, fontsize=8)
    ax.set_ylabel('Y (m)', color=DIM_COL, fontsize=8)
    ax.set_zlabel('Z (m)', color=DIM_COL, fontsize=8)
    if title:
        ax.set_title(title, color=TEXT_COL, fontsize=10, pad=8)

def _setup_2d_ax(ax, title='', xlabel='', ylabel=''):
    ax.set_facecolor(DARK)
    ax.tick_params(colors=DIM_COL, labelsize=7)
    for spine in ax.spines.values():
        spine.set_edgecolor(PANE_COL)
    if title:
        ax.set_title(title, color=TEXT_COL, fontsize=10)
    if xlabel:
        ax.set_xlabel(xlabel, color=DIM_COL, fontsize=8)
    if ylabel:
        ax.set_ylabel(ylabel, color=DIM_COL, fontsize=8)
    ax.grid(True, color=GRID_COL, linewidth=0.5, alpha=0.6)

def _save(fig, name, step_num, step_title):
    os.makedirs(OUT_DIR, exist_ok=True)
    path = os.path.join(OUT_DIR, name)
    fig.savefig(path, dpi=150, bbox_inches='tight',
                facecolor=DARK, edgecolor='none')
    plt.close(fig)
    print(f'  [Step {step_num}] ✓ 已保存: {path}  —— {step_title}')
    return path

# ─────────────────────────────────────────────────────────────
# 核心算法函数（与原版相同，此处内联）
# ─────────────────────────────────────────────────────────────

def read_ply(path):
    mesh = o3d.io.read_triangle_mesh(path)
    if not mesh.has_vertex_normals():
        mesh.compute_vertex_normals()
    xyz     = np.asarray(mesh.vertices)
    normals = np.asarray(mesh.vertex_normals)
    colors  = np.asarray(mesh.vertex_colors) * 255.0 if mesh.has_vertex_colors() \
              else np.zeros_like(xyz)
    verts = np.hstack((xyz, normals, colors))
    faces = np.asarray(mesh.triangles, dtype=np.int32)
    return verts, faces

def compute_face_normals(xyz, faces):
    fn = []
    for f in faces:
        v0, v1, v2 = xyz[f[0]], xyz[f[1]], xyz[f[2]]
        n = np.cross(v1 - v0, v2 - v0)
        norm = np.linalg.norm(n)
        fn.append(n / norm if norm > 1e-12 else np.array([0, 1, 0]))
    return np.array(fn)

def project_point_to_triangle(p, v0, v1, v2):
    edge1, edge2 = v1 - v0, v2 - v0
    n    = np.cross(edge1, edge2)
    nn   = np.linalg.norm(n)
    if nn < 1e-12:
        mid = (v0 + v1 + v2) / 3
        return mid, np.linalg.norm(p - mid), np.array([0,1,0])
    n_hat = n / nn
    t     = np.dot(n_hat, v0 - p)
    proj  = p + t * n_hat
    denom = np.dot(n, n)
    u = np.dot(n, np.cross(v2 - v1, proj - v1)) / denom
    v = np.dot(n, np.cross(v0 - v2, proj - v2)) / denom
    w = 1 - u - v
    if u >= -1e-6 and v >= -1e-6 and w >= -1e-6:
        return proj, abs(t), n_hat
    def seg_closest(a, b, pt):
        ab = b - a
        tc = np.clip(np.dot(pt - a, ab) / (np.dot(ab, ab) + 1e-12), 0, 1)
        cp = a + tc * ab
        return cp, np.linalg.norm(pt - cp)
    cands = [seg_closest(v0, v1, p), seg_closest(v1, v2, p), seg_closest(v2, v0, p)]
    best  = min(cands, key=lambda x: x[1])
    return best[0], best[1], n_hat


def point_to_mesh_projection_fast(query_pts, mesh_xyz, mesh_faces):
    """
    使用 Open3D 的 Tensor RaycastingScene 进行极速最近点投影
    """
    # 1. 将 numpy 数据转为 Open3D 的 Tensor 格式
    mesh_t = o3d.t.geometry.TriangleMesh()
    mesh_t.vertex.positions = o3d.core.Tensor(mesh_xyz, o3d.core.float32)
    mesh_t.triangle.indices = o3d.core.Tensor(mesh_faces, o3d.core.int32)

    # 2. 构建射线投射场景 (底层由 Intel Embree 硬件级加速)
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(mesh_t)

    # 3. 批量计算最近点
    query_tensor = o3d.core.Tensor(query_pts, o3d.core.float32)
    ans = scene.compute_closest_points(query_tensor)

    # 4. 提取投影点坐标和所落面片的法向量
    closest_pts = ans['points'].cpu().numpy()
    normals = ans['primitive_normals'].cpu().numpy()

    # 返回与原代码兼容的格式 [(pt1, normal1), (pt2, normal2), ...]
    results = [(closest_pts[i], normals[i]) for i in range(len(closest_pts))]
    return results


def resample_path(pts, target_spacing):
    if len(pts) < 2:
        return pts
    out   = [pts[0]]
    accum = 0.0
    for i in range(1, len(pts)):
        seg     = pts[i] - pts[i - 1]
        seg_len = np.linalg.norm(seg)
        if seg_len < 1e-12:
            continue
        while accum + seg_len >= target_spacing:
            t       = (target_spacing - accum) / seg_len
            new_pt  = pts[i - 1] + t * seg
            out.append(new_pt)
            seg_len -= (target_spacing - accum)
            pts[i - 1] = new_pt
            seg     = pts[i] - pts[i - 1]
            accum   = 0.0
        accum += seg_len
    out.append(pts[-1])
    return np.array(out)

def boustrophedon_coverage(hull_pts, step_size, scan_dir='x'):
    from matplotlib.path import Path as MplPath
    if scan_dir == 'y':
        pts = hull_pts[:, [1, 0]]
    else:
        pts = hull_pts.copy()

    min_v, max_v = pts[:, 1].min(), pts[:, 1].max()
    v_lines = np.arange(min_v + step_size / 2, max_v, step_size)
    if len(v_lines) == 0:
        v_lines = np.array([(min_v + max_v) / 2])

    path_segments = []
    for vi, v in enumerate(v_lines):
        crossings = []
        n = len(pts)
        for i in range(n):
            a, b = pts[i], pts[(i + 1) % n]
            if (a[1] <= v < b[1]) or (b[1] <= v < a[1]):
                tc = (v - a[1]) / (b[1] - a[1] + 1e-15)
                crossings.append(a[0] + tc * (b[0] - a[0]))
        if len(crossings) < 2:
            continue
        crossings.sort()
        u_start, u_end = crossings[0], crossings[-1]
        u_pts = np.arange(u_start, u_end + 1e-9, step_size / 4)
        if len(u_pts) == 0:
            u_pts = np.array([(u_start + u_end) / 2])
        if vi % 2 == 1:
            u_pts = u_pts[::-1]
        path_segments.append(np.column_stack([u_pts, np.full(len(u_pts), v)]))

    if not path_segments:
        return np.array([]).reshape(0, 2)

    full_path = []
    for i, seg in enumerate(path_segments):
        full_path.append(seg)
        if i < len(path_segments) - 1:
            full_path.append(np.array([seg[-1], path_segments[i + 1][0]]))

    path_2d = np.vstack(full_path)
    if scan_dir == 'y':
        path_2d = path_2d[:, [1, 0]]
    return path_2d

def smooth_path(path_3d, window=3):
    if len(path_3d) < window * 2:
        return path_3d
    smoothed = path_3d.copy()
    for i in range(window, len(path_3d) - window):
        smoothed[i] = path_3d[i - window:i + window + 1].mean(axis=0)
    return smoothed

def smooth_normals(normals, window=3):
    if len(normals) < window * 2:
        return normals
    smoothed = normals.copy()
    for i in range(window, len(normals) - window):
        avg = normals[i - window:i + window + 1].mean(axis=0)
        norm_len = np.linalg.norm(avg)
        smoothed[i] = avg / norm_len if norm_len > 1e-12 else normals[i]
    return smoothed

def export_path_csv(path_3d, path_normals, out_csv):
    import csv
    with open(out_csv, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['idx', 'x', 'y', 'z', 'nx', 'ny', 'nz', 'seg_len', 'cumulative_len'])
        cum = 0.0
        for i, (pt, n) in enumerate(zip(path_3d, path_normals)):
            seg = np.linalg.norm(pt - path_3d[i-1]) if i > 0 else 0.0
            cum += seg
            writer.writerow([i, f'{pt[0]:.6f}', f'{pt[1]:.6f}', f'{pt[2]:.6f}',
                              f'{n[0]:.6f}', f'{n[1]:.6f}', f'{n[2]:.6f}',
                              f'{seg:.6f}', f'{cum:.6f}'])
    print(f'  [Export] ✓ CSV 已保存: {out_csv}')

# ═════════════════════════════════════════════════════════════
# ★ 分步可视化函数
# ═════════════════════════════════════════════════════════════

def viz_step1_mesh(xyz, normals, faces):
    """Step 1: 原始网格解析与统计"""
    print('\n[Step 1] 可视化原始网格...')
    fig = plt.figure(figsize=(18, 7), facecolor=DARK)
    fig.suptitle('Step 1 — 网格解析  |  Mesh Parsing',
                 color=TEXT_COL, fontsize=14, fontweight='bold')

    # ── 子图1: 3D 网格 ──
    ax1 = fig.add_subplot(131, projection='3d')
    _setup_3d_ax(ax1, '3D 网格（半透明）')
    polys = [[xyz[f[0]], xyz[f[1]], xyz[f[2]]] for f in faces if len(f) == 3]
    mc = Poly3DCollection(polys, alpha=0.25,
                          facecolor=BLUE, edgecolor='#21262d', linewidth=0.15)
    ax1.add_collection3d(mc)
    ax1.auto_scale_xyz(xyz[:, 0], xyz[:, 1], xyz[:, 2])

    # 顶点散点（采样）
    s = max(1, len(xyz) // 300)
    ax1.scatter(xyz[::s, 0], xyz[::s, 1], xyz[::s, 2],
                c=CYAN, s=2, alpha=0.6, zorder=5)

    # ── 子图2: 顶点法向量分布（球面投影） ──
    ax2 = fig.add_subplot(132)
    _setup_2d_ax(ax2, '顶点法向量分布（UV 球面投影）',
                 xlabel='方位角 φ (°)', ylabel='仰角 θ (°)')
    phi   = np.degrees(np.arctan2(normals[:, 1], normals[:, 0]))
    theta = np.degrees(np.arcsin(np.clip(normals[:, 2], -1, 1)))
    sc = ax2.scatter(phi[::s], theta[::s], c=normals[::s, 2],
                     cmap='coolwarm', s=3, alpha=0.6)
    cb = plt.colorbar(sc, ax=ax2)
    cb.set_label('法向量 Z 分量', color=DIM_COL, fontsize=7)
    cb.ax.tick_params(colors=DIM_COL, labelsize=6)

    # ── 子图3: 统计信息文本面板 ──
    ax3 = fig.add_subplot(133)
    ax3.set_facecolor(DARK)
    ax3.axis('off')

    # 计算表面积
    total_area = sum(
        0.5 * np.linalg.norm(np.cross(xyz[f[1]] - xyz[f[0]], xyz[f[2]] - xyz[f[0]]))
        for f in faces if len(f) == 3
    )
    bbox_size = xyz.max(axis=0) - xyz.min(axis=0)

    stats = [
        ('顶点数 (Vertices)',   f'{len(xyz):,}'),
        ('面片数 (Faces)',      f'{len(faces):,}'),
        ('总表面积',            f'{total_area:.4f} m²\n({total_area*1e4:.2f} cm²)'),
        ('包围盒 X',            f'{bbox_size[0]*1000:.1f} mm'),
        ('包围盒 Y',            f'{bbox_size[1]*1000:.1f} mm'),
        ('包围盒 Z',            f'{bbox_size[2]*1000:.1f} mm'),
        ('顶点法向量',          '✓ 已计算/读取'),
        ('颜色属性',            '✓ 已读取'),
    ]
    y_pos = 0.92
    ax3.text(0.05, y_pos, '📊  网格统计信息', color=TEXT_COL,
             fontsize=11, fontweight='bold', transform=ax3.transAxes)
    y_pos -= 0.08
    for label, val in stats:
        ax3.text(0.05, y_pos, f'  {label}:', color=DIM_COL,
                 fontsize=9, transform=ax3.transAxes)
        ax3.text(0.55, y_pos, val, color=GREEN,
                 fontsize=9, transform=ax3.transAxes)
        y_pos -= 0.09

    plt.tight_layout(rect=[0, 0, 1, 0.93])
    return _save(fig, 'step1_mesh_parse.png', 1, '原始网格解析')


def viz_step2_pca(xyz, normals, center, u_axis, v_axis, proj_normal, eigvals):
    """Step 2: PCA 主轴与最优投影平面"""
    print('\n[Step 2] 可视化 PCA 投影平面...')
    fig = plt.figure(figsize=(18, 7), facecolor=DARK)
    fig.suptitle('Step 2 — PCA 最优投影平面  |  PCA Optimal Projection Plane',
                 color=TEXT_COL, fontsize=14, fontweight='bold')

    # ── 子图1: 3D 主轴可视化 ──
    ax1 = fig.add_subplot(131, projection='3d')
    _setup_3d_ax(ax1, '3D 网格 + PCA 主轴')
    s_sparse = max(1, len(xyz) // 200)
    ax1.scatter(xyz[::s_sparse, 0], xyz[::s_sparse, 1], xyz[::s_sparse, 2],
                c=BLUE, s=3, alpha=0.4)

    # 绘制三条主轴（从质心出发）
    scale = (xyz.max(axis=0) - xyz.min(axis=0)).max() * 0.35
    for axis, col, lbl in [
        (u_axis,      RED,    f'PC1 (λ={eigvals[0]:.4f}) U轴'),
        (v_axis,      GREEN,  f'PC2 (λ={eigvals[1]:.4f}) V轴'),
        (proj_normal, YELLOW, f'PC3 (λ={eigvals[2]:.4f}) 法向'),
    ]:
        end = center + axis * scale
        ax1.quiver(*center, *(axis * scale), color=col,
                   linewidth=2.5, arrow_length_ratio=0.12)
        ax1.text(*end, f'  {lbl}', color=col, fontsize=7)

    # 绘制 PCA 平面（半透明矩形）
    corners_uv = np.array([[-1,-1],[1,-1],[1,1],[-1,1]]) * scale * 0.9
    corners_3d = corners_uv[:, 0:1] * u_axis + corners_uv[:, 1:2] * v_axis + center
    plane_poly = Poly3DCollection([corners_3d], alpha=0.12,
                                  facecolor=ORANGE, edgecolor=ORANGE, linewidth=0.8)
    ax1.add_collection3d(plane_poly)
    ax1.auto_scale_xyz(xyz[:, 0], xyz[:, 1], xyz[:, 2])

    # ── 子图2: 特征值柱状图 ──
    ax2 = fig.add_subplot(132)
    _setup_2d_ax(ax2, '特征值 & 方差贡献', xlabel='主成分', ylabel='特征值')
    contrib = eigvals / eigvals.sum() * 100
    bars = ax2.bar(['PC1\n(U轴)', 'PC2\n(V轴)', 'PC3\n(法向)'],
                   eigvals, color=[RED, GREEN, YELLOW], alpha=0.8, width=0.5)
    ax2_r = ax2.twinx()
    ax2_r.plot(['PC1\n(U轴)', 'PC2\n(V轴)', 'PC3\n(法向)'],
               contrib, 'o--', color=CYAN, linewidth=1.5, markersize=8)
    ax2_r.set_ylabel('方差贡献率 (%)', color=CYAN, fontsize=8)
    ax2_r.tick_params(colors=CYAN, labelsize=7)
    for bar, c in zip(bars, contrib):
        ax2.text(bar.get_x() + bar.get_width()/2, bar.get_height() * 1.02,
                 f'{c:.1f}%', ha='center', color=TEXT_COL, fontsize=9)

    # ── 子图3: 2D 投影点云（对比三种投影）──
    ax3 = fig.add_subplot(133)
    _setup_2d_ax(ax3, 'PCA 最优 2D 投影（U-V 平面）', xlabel='U 轴 (m)', ylabel='V 轴 (m)')
    xyz_c  = xyz - center
    uv     = np.column_stack([xyz_c @ u_axis, xyz_c @ v_axis])
    s2 = max(1, len(uv) // 500)
    sc = ax3.scatter(uv[::s2, 0], uv[::s2, 1], c=xyz[::s2, 2],
                     cmap='viridis', s=4, alpha=0.6)
    cb = plt.colorbar(sc, ax=ax3)
    cb.set_label('原始 Z 高度 (m)', color=DIM_COL, fontsize=7)
    cb.ax.tick_params(colors=DIM_COL, labelsize=6)
    ax3.set_aspect('equal')

    # 标注 PCA 方向文字
    ax3.annotate('', xy=(uv[:, 0].max()*0.6, 0),
                 xytext=(0, 0),
                 arrowprops=dict(arrowstyle='->', color=RED, lw=2))
    ax3.text(uv[:, 0].max()*0.62, 0, '  PC1 (U)', color=RED, fontsize=8, va='center')
    ax3.annotate('', xy=(0, uv[:, 1].max()*0.6),
                 xytext=(0, 0),
                 arrowprops=dict(arrowstyle='->', color=GREEN, lw=2))
    ax3.text(0, uv[:, 1].max()*0.62, '  PC2 (V)', color=GREEN, fontsize=8)

    plt.tight_layout(rect=[0, 0, 1, 0.93])
    return _save(fig, 'step2_pca_projection.png', 2, 'PCA 最优投影平面')


def viz_step3_boundary(xyz_2d, hull_pts, hull):
    """Step 3: 2D 投影与凸包边界构建"""
    print('\n[Step 3] 可视化 2D 边界...')
    fig = plt.figure(figsize=(16, 7), facecolor=DARK)
    fig.suptitle('Step 3 — 2D 投影 & 凸包边界  |  2D Projection & Convex Hull Boundary',
                 color=TEXT_COL, fontsize=14, fontweight='bold')

    # ── 子图1: 投影点云 + 凸包 ──
    ax1 = fig.add_subplot(121)
    _setup_2d_ax(ax1, '2D 投影点云与凸包边界', xlabel='U (m)', ylabel='V (m)')
    s = max(1, len(xyz_2d) // 600)
    ax1.scatter(xyz_2d[::s, 0], xyz_2d[::s, 1], s=4, c=BLUE, alpha=0.5, label='投影顶点')

    # 凸包边界
    hull_closed = np.vstack([hull_pts, hull_pts[0]])
    ax1.fill(hull_pts[:, 0], hull_pts[:, 1], alpha=0.12, color=ORANGE)
    ax1.plot(hull_closed[:, 0], hull_closed[:, 1],
             color=ORANGE, linewidth=2, label=f'凸包边界 ({len(hull_pts)} 顶点)')

    # 标注凸包顶点
    for i, pt in enumerate(hull_pts):
        ax1.scatter(*pt, color=RED, s=40, zorder=10)
        ax1.text(pt[0], pt[1], f'  v{i}', color=RED, fontsize=7)

    ax1.set_aspect('equal')
    ax1.legend(fontsize=8, facecolor='#161b22', labelcolor=TEXT_COL)

    # ── 子图2: 凸包算法示意（展示射线交点原理） ──
    ax2 = fig.add_subplot(122)
    _setup_2d_ax(ax2, '扫描行与凸包交点求解原理', xlabel='U (m)', ylabel='V (m)')

    ax2.fill(hull_pts[:, 0], hull_pts[:, 1], alpha=0.12, color=ORANGE)
    ax2.plot(hull_closed[:, 0], hull_closed[:, 1], color=ORANGE, linewidth=1.5)

    u_min = hull_pts[:, 0].min()
    u_max = hull_pts[:, 0].max()
    v_min = hull_pts[:, 1].min()
    v_max = hull_pts[:, 1].max()
    v_range = v_max - v_min

    # 展示 5 条示例扫描线
    step_demo = v_range / 6
    for vi, v in enumerate(np.arange(v_min + step_demo, v_max, step_demo)[:5]):
        ax2.axhline(y=v, color=GRID_COL, linewidth=0.8, linestyle='--', alpha=0.7)

        # 求与凸包的交点
        crossings = []
        n = len(hull_pts)
        for i in range(n):
            a, b = hull_pts[i], hull_pts[(i + 1) % n]
            if (a[1] <= v < b[1]) or (b[1] <= v < a[1]):
                tc = (v - a[1]) / (b[1] - a[1] + 1e-15)
                crossings.append(a[0] + tc * (b[0] - a[0]))

        if len(crossings) >= 2:
            crossings.sort()
            ul, ur = crossings[0], crossings[-1]
            ax2.plot([ul, ur], [v, v], color=CYAN, linewidth=2, alpha=0.8,
                     label='扫描行段' if vi == 0 else '')
            ax2.scatter([ul, ur], [v, v], color=GREEN, s=50, zorder=10,
                        label='轮廓交点' if vi == 0 else '')
            ax2.annotate(f'u_L={ul:.3f}', xy=(ul, v), xytext=(ul - v_range*0.15, v + v_range*0.03),
                         color=GREEN, fontsize=7,
                         arrowprops=dict(arrowstyle='->', color=GREEN, lw=0.8))
            ax2.annotate(f'u_R={ur:.3f}', xy=(ur, v), xytext=(ur + v_range*0.02, v + v_range*0.03),
                         color=GREEN, fontsize=7,
                         arrowprops=dict(arrowstyle='->', color=GREEN, lw=0.8))

    ax2.set_aspect('equal')
    ax2.legend(fontsize=8, facecolor='#161b22', labelcolor=TEXT_COL, loc='upper right')

    plt.tight_layout(rect=[0, 0, 1, 0.93])
    return _save(fig, 'step3_2d_boundary.png', 3, '2D 投影与凸包边界')


def viz_step4_boustrophedon(path_2d, hull_pts, step_size):
    """Step 4: 弓字形扫描路径（2D）"""
    print('\n[Step 4] 可视化 2D 弓字形路径...')
    fig = plt.figure(figsize=(18, 7), facecolor=DARK)
    fig.suptitle('Step 4 — Boustrophedon 弓字形扫描路径  |  Boustrophedon Coverage Path',
                 color=TEXT_COL, fontsize=14, fontweight='bold')

    # ── 子图1: 完整路径（行程渐变色）──
    ax1 = fig.add_subplot(131)
    _setup_2d_ax(ax1, '完整弓字形路径（行程着色）', xlabel='U (m)', ylabel='V (m)')
    hull_closed = np.vstack([hull_pts, hull_pts[0]])
    ax1.fill(hull_pts[:, 0], hull_pts[:, 1], alpha=0.1, color=ORANGE)
    ax1.plot(hull_closed[:, 0], hull_closed[:, 1], color=ORANGE, linewidth=1.2, alpha=0.7)

    n = len(path_2d)
    colors_plasma = plt.cm.plasma(np.linspace(0.05, 0.95, n))
    for i in range(1, n):
        ax1.plot([path_2d[i-1, 0], path_2d[i, 0]],
                 [path_2d[i-1, 1], path_2d[i, 1]],
                 color=colors_plasma[i], linewidth=0.8, alpha=0.9)
    ax1.scatter(*path_2d[0], color=GREEN, s=80, zorder=10, label='起点')
    ax1.scatter(*path_2d[-1], color=RED, s=80, zorder=10, label='终点')
    ax1.set_aspect('equal')
    ax1.legend(fontsize=8, facecolor='#161b22', labelcolor=TEXT_COL)

    # ── colorbar for travel progress ──
    sm = plt.cm.ScalarMappable(cmap='plasma', norm=plt.Normalize(0, n))
    sm.set_array([])
    cb = plt.colorbar(sm, ax=ax1, fraction=0.04)
    cb.set_label('路径点序号', color=DIM_COL, fontsize=7)
    cb.ax.tick_params(colors=DIM_COL, labelsize=6)

    # ── 子图2: 扫描行识别（扫描段 vs 过渡段）──
    ax2 = fig.add_subplot(132)
    _setup_2d_ax(ax2, '扫描段 vs 过渡段', xlabel='U (m)', ylabel='V (m)')
    ax2.fill(hull_pts[:, 0], hull_pts[:, 1], alpha=0.08, color=ORANGE)
    ax2.plot(hull_closed[:, 0], hull_closed[:, 1], color=ORANGE, linewidth=1, alpha=0.6)

    seg_lens = np.linalg.norm(np.diff(path_2d, axis=0), axis=1)
    threshold = step_size * 1.5
    for i in range(1, n):
        col = CYAN if seg_lens[i-1] <= threshold else RED
        ax2.plot([path_2d[i-1, 0], path_2d[i, 0]],
                 [path_2d[i-1, 1], path_2d[i, 1]],
                 color=col, linewidth=0.9, alpha=0.85)

    ax2.plot([], [], color=CYAN, linewidth=2, label=f'扫描段 (≤{threshold*100:.1f}cm)')
    ax2.plot([], [], color=RED,  linewidth=2, label=f'过渡段 (>{threshold*100:.1f}cm)')
    ax2.set_aspect('equal')
    ax2.legend(fontsize=7, facecolor='#161b22', labelcolor=TEXT_COL)

    # 标注行间距
    v_vals = path_2d[:, 1]
    unique_rows = np.unique(np.round(v_vals / step_size)) * step_size
    for vr in unique_rows[:2]:
        ax2.axhline(y=vr, color=YELLOW, linewidth=0.6, linestyle=':', alpha=0.5)
    if len(unique_rows) > 1:
        ax2.annotate('', xy=(hull_pts[:, 0].min(), unique_rows[1]),
                     xytext=(hull_pts[:, 0].min(), unique_rows[0]),
                     arrowprops=dict(arrowstyle='<->', color=YELLOW, lw=1.2))
        ax2.text(hull_pts[:, 0].min() + 0.005, (unique_rows[0]+unique_rows[1])/2,
                 f'd={step_size*100:.1f}cm', color=YELLOW, fontsize=8)

    # ── 子图3: 段长分布直方图 ──
    ax3 = fig.add_subplot(133)
    _setup_2d_ax(ax3, '路径段长分布（扫描 vs 过渡）', xlabel='段长度 (m)', ylabel='段数量')
    travel     = seg_lens[seg_lens <= threshold]
    transition = seg_lens[seg_lens > threshold]
    bins = np.linspace(0, seg_lens.max() * 1.05, 40)
    ax3.hist(travel,     bins=bins, color=CYAN,  alpha=0.75,
             label=f'扫描段 ({len(travel):,})', edgecolor='none')
    ax3.hist(transition, bins=bins, color=RED,   alpha=0.75,
             label=f'过渡段 ({len(transition):,})', edgecolor='none')
    ax3.axvline(x=step_size, color=YELLOW, linestyle='--', linewidth=1.5,
                label=f'行间距 {step_size*100:.1f}cm')
    ax3.legend(fontsize=7, facecolor='#161b22', labelcolor=TEXT_COL)
    ax3.text(0.98, 0.98, f'总点数: {n:,}\n总段数: {n-1:,}',
             transform=ax3.transAxes, color=TEXT_COL, fontsize=8,
             ha='right', va='top',
             bbox=dict(facecolor='#161b22', edgecolor=PANE_COL, boxstyle='round'))

    plt.tight_layout(rect=[0, 0, 1, 0.93])
    return _save(fig, 'step4_boustrophedon.png', 4, '弓字形扫描路径生成')


def viz_step5_backprojection(xyz, faces, path_3d_raw, path_2d):
    """Step 5: 2D→3D 反投影结果"""
    print('\n[Step 5] 可视化 3D 反投影...')
    fig = plt.figure(figsize=(18, 7), facecolor=DARK)
    fig.suptitle('Step 5 — 2D→3D 反投影  |  2D to 3D Back-Projection',
                 color=TEXT_COL, fontsize=14, fontweight='bold')

    # ── 子图1: 3D 路径在网格上 ──
    ax1 = fig.add_subplot(131, projection='3d')
    _setup_3d_ax(ax1, '3D 路径 on 网格表面')
    polys = [[xyz[f[0]], xyz[f[1]], xyz[f[2]]] for f in faces if len(f) == 3]
    mc = Poly3DCollection(polys, alpha=0.10, facecolor=BLUE,
                          edgecolor='#21262d', linewidth=0.1)
    ax1.add_collection3d(mc)

    n = len(path_3d_raw)
    colors_v = plt.cm.viridis(np.linspace(0.05, 0.95, n))
    for i in range(1, n):
        ax1.plot([path_3d_raw[i-1,0], path_3d_raw[i,0]],
                 [path_3d_raw[i-1,1], path_3d_raw[i,1]],
                 [path_3d_raw[i-1,2], path_3d_raw[i,2]],
                 color=colors_v[i], linewidth=0.7, alpha=0.9)
    ax1.scatter(*path_3d_raw[0],  color=GREEN, s=50, zorder=10)
    ax1.scatter(*path_3d_raw[-1], color=RED,   s=50, zorder=10)
    ax1.auto_scale_xyz(xyz[:,0], xyz[:,1], xyz[:,2])

    # ── 子图2: 路径 Z 高度分布（验证吸附到曲面） ──
    ax2 = fig.add_subplot(132)
    _setup_2d_ax(ax2, '路径沿弧长的 Z 高度变化\n（验证曲面吸附效果）',
                 xlabel='累积弧长 (m)', ylabel='Z 高度 (m)')
    arc_len = np.concatenate([[0], np.cumsum(
        np.linalg.norm(np.diff(path_3d_raw, axis=0), axis=1))])
    ax2.plot(arc_len, path_3d_raw[:, 2], color=CYAN, linewidth=1.2, alpha=0.85)
    ax2.fill_between(arc_len, path_3d_raw[:, 2],
                     path_3d_raw[:, 2].min(), alpha=0.15, color=CYAN)

    z_range = path_3d_raw[:, 2].max() - path_3d_raw[:, 2].min()
    ax2.text(0.02, 0.95, f'Z 范围: {z_range*1000:.1f} mm\n'
             f'总弧长: {arc_len[-1]:.3f} m',
             transform=ax2.transAxes, color=TEXT_COL, fontsize=8, va='top',
             bbox=dict(facecolor='#161b22', edgecolor=PANE_COL, boxstyle='round'))

    # ── 子图3: 路径投影误差（2D→3D 偏差） ──
    ax3 = fig.add_subplot(133)
    _setup_2d_ax(ax3, '投影误差分布\n（路径点离网格表面的距离）',
                 xlabel='投影误差 (mm)', ylabel='点数量')

    # 对每个路径点找最近网格顶点距离作为误差代理
    kd = KDTree(xyz)
    dists, _ = kd.query(path_3d_raw)
    dists_mm = dists * 1000

    ax3.hist(dists_mm, bins=50, color=ORANGE, alpha=0.8, edgecolor='none')
    ax3.axvline(x=dists_mm.mean(), color=YELLOW, linestyle='--', linewidth=1.5,
                label=f'均值: {dists_mm.mean():.2f} mm')
    ax3.axvline(x=np.percentile(dists_mm, 95), color=RED, linestyle='--', linewidth=1.2,
                label=f'P95: {np.percentile(dists_mm, 95):.2f} mm')
    ax3.legend(fontsize=8, facecolor='#161b22', labelcolor=TEXT_COL)

    plt.tight_layout(rect=[0, 0, 1, 0.93])
    return _save(fig, 'step5_3d_backprojection.png', 5, '2D→3D 反投影')


def viz_step6_smoothing(path_3d_before, path_3d_after, normals_before, normals_after):
    """Step 6: 路径平滑与法向量修正对比"""
    print('\n[Step 6] 可视化路径平滑与法向量修正...')
    fig = plt.figure(figsize=(20, 8), facecolor=DARK)
    fig.suptitle('Step 6 — 路径平滑 & 法向量修正  |  Path Smoothing & Normal Correction',
                 color=TEXT_COL, fontsize=14, fontweight='bold')

    n = min(len(path_3d_before), len(path_3d_after))
    arc_before = np.concatenate([[0], np.cumsum(
        np.linalg.norm(np.diff(path_3d_before[:n], axis=0), axis=1))])
    arc_after  = np.concatenate([[0], np.cumsum(
        np.linalg.norm(np.diff(path_3d_after[:n],  axis=0), axis=1))])

    # ── 子图1: 位置偏差（平滑前后路径的 Z 分量对比）──
    ax1 = fig.add_subplot(141)
    _setup_2d_ax(ax1, 'Z 高度对比（平滑前 vs 后）',
                 xlabel='累积弧长 (m)', ylabel='Z (m)')
    ax1.plot(arc_before[:n], path_3d_before[:n, 2], color=RED,
             linewidth=1.2, alpha=0.8, label='平滑前（原始）')
    ax1.plot(arc_after[:n],  path_3d_after[:n, 2],  color=GREEN,
             linewidth=1.2, alpha=0.8, label='平滑后')
    ax1.legend(fontsize=7, facecolor='#161b22', labelcolor=TEXT_COL)

    # ── 子图2: 法向量 Z 分量对比 ──
    ax2 = fig.add_subplot(142)
    _setup_2d_ax(ax2, '法向量 nz 对比\n（修正前后连续性）',
                 xlabel='累积弧长 (m)', ylabel='法向量 Z 分量')
    ax2.plot(arc_before[:n], normals_before[:n, 2], color=RED,
             linewidth=0.8, alpha=0.7, label='修正前（面法向，跳变）')
    ax2.plot(arc_after[:n],  normals_after[:n, 2],  color=GREEN,
             linewidth=0.8, alpha=0.7, label='修正后（插值法向，平滑）')
    ax2.legend(fontsize=7, facecolor='#161b22', labelcolor=TEXT_COL)

    # ── 子图3: n·t（法向量与切向量内积，验证垂直性）──
    ax3 = fig.add_subplot(143)
    _setup_2d_ax(ax3, 'n·t 内积（应接近 0）\n法向量与路径切向垂直性',
                 xlabel='路径点索引', ylabel='n·t 内积')

    def compute_ndott(path, normals):
        tangents = np.diff(path, axis=0)
        norms    = np.linalg.norm(tangents, axis=1, keepdims=True)
        tangents = tangents / (norms + 1e-12)
        dots     = np.einsum('ij,ij->i', normals[:-1], tangents)
        return dots

    dots_before = compute_ndott(path_3d_before[:n], normals_before[:n])
    dots_after  = compute_ndott(path_3d_after[:n],  normals_after[:n])

    x_idx = np.arange(len(dots_before))
    ax3.plot(x_idx, dots_before, color=RED,   linewidth=0.7, alpha=0.6,
             label=f'修正前  μ={dots_before.mean():.3f}')
    ax3.plot(x_idx, dots_after,  color=GREEN, linewidth=0.7, alpha=0.6,
             label=f'修正后  μ={dots_after.mean():.3f}')
    ax3.axhline(y=0, color=YELLOW, linestyle='--', linewidth=1, alpha=0.8)
    ax3.set_ylim(-0.5, 0.5)
    ax3.legend(fontsize=7, facecolor='#161b22', labelcolor=TEXT_COL)

    # ── 子图4: |n·t| 直方图对比 ──
    ax4 = fig.add_subplot(144)
    _setup_2d_ax(ax4, '|n·t| 分布对比\n（越集中于 0 越好）',
                 xlabel='|n·t| 绝对值', ylabel='点数量')
    bins = np.linspace(0, max(np.abs(dots_before).max(), np.abs(dots_after).max()) * 1.05, 40)
    ax4.hist(np.abs(dots_before), bins=bins, color=RED,   alpha=0.6,
             label=f'修正前 (μ={np.abs(dots_before).mean():.3f})', edgecolor='none')
    ax4.hist(np.abs(dots_after),  bins=bins, color=GREEN, alpha=0.6,
             label=f'修正后 (μ={np.abs(dots_after).mean():.3f})',  edgecolor='none')
    ax4.legend(fontsize=7, facecolor='#161b22', labelcolor=TEXT_COL)

    plt.tight_layout(rect=[0, 0, 1, 0.93])
    return _save(fig, 'step6_smoothing_normals.png', 6, '路径平滑与法向量修正')


def viz_step7_final(xyz, normals, faces, path_3d, path_normals, path_2d, hull_pts, step_size):
    """Step 7: 最终综合结果"""
    print('\n[Step 7] 可视化最终结果...')
    fig = plt.figure(figsize=(22, 14), facecolor=DARK)
    fig.suptitle('Step 7 — 最终覆盖路径结果  |  Final Full-Coverage Path Result',
                 color=TEXT_COL, fontsize=15, fontweight='bold', y=0.99)

    ax1 = fig.add_subplot(231, projection='3d')
    ax2 = fig.add_subplot(232, projection='3d')
    ax3 = fig.add_subplot(233)
    ax4 = fig.add_subplot(234, projection='3d')
    ax5 = fig.add_subplot(235)
    ax6 = fig.add_subplot(236)

    for ax in [ax1, ax2, ax4]:
        _setup_3d_ax(ax)
    for ax in [ax3, ax5, ax6]:
        _setup_2d_ax(ax)

    polys = [[xyz[f[0]], xyz[f[1]], xyz[f[2]]] for f in faces if len(f) == 3]

    # ── ax1: 3D 路径总览 ──
    ax1.set_title('3D 路径总览', color=TEXT_COL, fontsize=10)
    mc = Poly3DCollection(polys, alpha=0.10, facecolor=BLUE,
                          edgecolor='#21262d', linewidth=0.15)
    ax1.add_collection3d(mc)
    n = len(path_3d)
    colors_p = plt.cm.plasma(np.linspace(0.05, 0.95, n))
    for i in range(1, n):
        ax1.plot([path_3d[i-1,0], path_3d[i,0]], [path_3d[i-1,1], path_3d[i,1]],
                 [path_3d[i-1,2], path_3d[i,2]], color=colors_p[i], linewidth=0.8, alpha=0.9)
    ax1.scatter(*path_3d[0],  color=GREEN, s=60, zorder=10, label='起点')
    ax1.scatter(*path_3d[-1], color=RED,   s=60, zorder=10, label='终点')
    ax1.legend(fontsize=7, facecolor='#161b22', labelcolor=TEXT_COL)
    ax1.auto_scale_xyz(xyz[:,0], xyz[:,1], xyz[:,2])

    # ── ax2: 工具姿态（位置+法向箭头） ──
    ax2.set_title('工具姿态（位置 + 法向量）', color=TEXT_COL, fontsize=10)
    mc2 = Poly3DCollection(polys, alpha=0.07, facecolor=BLUE,
                           edgecolor='#21262d', linewidth=0.1)
    ax2.add_collection3d(mc2)
    ax2.plot(path_3d[:,0], path_3d[:,1], path_3d[:,2],
             color=ORANGE, linewidth=0.7, alpha=0.6)
    pose_step = max(1, n // 70)
    arrow_scale = (xyz.max(axis=0) - xyz.min(axis=0)).max() * 0.04
    for i in range(0, n, pose_step):
        nv = path_normals[i]
        ax2.quiver(path_3d[i,0], path_3d[i,1], path_3d[i,2],
                   nv[0]*arrow_scale, nv[1]*arrow_scale, nv[2]*arrow_scale,
                   color=CYAN, linewidth=0.8, alpha=0.9)
    ax2.auto_scale_xyz(xyz[:,0], xyz[:,1], xyz[:,2])

    # ── ax3: 2D 路径 ──
    _setup_2d_ax(ax3, '2D 投影覆盖路径', xlabel='U (m)', ylabel='V (m)')
    hull_closed = np.vstack([hull_pts, hull_pts[0]])
    ax3.fill(hull_pts[:,0], hull_pts[:,1], alpha=0.1, color=ORANGE)
    ax3.plot(hull_closed[:,0], hull_closed[:,1], color=ORANGE, linewidth=1.2)
    n2 = len(path_2d)
    seg_c2 = plt.cm.plasma(np.linspace(0.05, 0.95, n2))
    for i in range(1, n2):
        ax3.plot([path_2d[i-1,0], path_2d[i,0]], [path_2d[i-1,1], path_2d[i,1]],
                 color=seg_c2[i], linewidth=0.7, alpha=0.85)
    ax3.scatter(*path_2d[0], color=GREEN, s=60, zorder=10, label='起点')
    ax3.scatter(*path_2d[-1], color=RED,  s=60, zorder=10, label='终点')
    ax3.set_aspect('equal')
    ax3.legend(fontsize=7, facecolor='#161b22', labelcolor=TEXT_COL)

    # ── ax4: 法向量一致性（着色显示 nz） ──
    ax4.set_title('曲面法向量（Nz 着色）', color=TEXT_COL, fontsize=10)
    fn_all = compute_face_normals(xyz, faces)
    face_cols = [plt.cm.coolwarm((fn_all[fi][2]+1)/2) for fi, f in enumerate(faces) if len(f)==3]
    mc3 = Poly3DCollection(polys, alpha=0.75, facecolor=face_cols, edgecolor='none')
    ax4.add_collection3d(mc3)
    ax4.auto_scale_xyz(xyz[:,0], xyz[:,1], xyz[:,2])

    # ── ax5: 路径密度热图 ──
    _setup_2d_ax(ax5, '路径点密度热图', xlabel='X (m)', ylabel='Z (m)')
    H, xe, ye = np.histogram2d(path_3d[:,0], path_3d[:,2], bins=40)
    im = ax5.imshow(H.T, origin='lower', aspect='auto',
                    extent=[xe[0], xe[-1], ye[0], ye[-1]],
                    cmap='hot', interpolation='bilinear')
    cb = plt.colorbar(im, ax=ax5)
    cb.set_label('点密度', color=DIM_COL, fontsize=7)
    cb.ax.tick_params(colors=DIM_COL, labelsize=6)

    # ── ax6: 最终统计面板 ──
    ax6.set_facecolor(DARK)
    ax6.axis('off')

    seg_lens = np.linalg.norm(np.diff(path_3d, axis=0), axis=1)
    total_len = seg_lens.sum()
    total_area = sum(
        0.5 * np.linalg.norm(np.cross(xyz[f[1]]-xyz[f[0]], xyz[f[2]]-xyz[f[0]]))
        for f in faces if len(f)==3)
    n_rows = int(np.round((hull_pts[:,1].max()-hull_pts[:,1].min()) / step_size))
    coverage = min(100.0, total_len * (step_size/(1-0.15)) / total_area * 100)

    summary = [
        ('总路径长度',  f'{total_len:.3f} m'),
        ('路径点数',    f'{len(path_3d):,}'),
        ('扫描行数',    f'{n_rows}'),
        ('理论覆盖率',  f'{coverage:.1f} %'),
        ('平均段长',    f'{seg_lens.mean()*1000:.2f} mm'),
        ('最大段长',    f'{seg_lens.max()*1000:.2f} mm'),
        ('行间距',      f'{step_size*100:.2f} cm'),
        ('总表面积',    f'{total_area:.4f} m²'),
    ]
    ax6.text(0.05, 0.95, '📋  最终路径统计', color=TEXT_COL,
             fontsize=12, fontweight='bold', transform=ax6.transAxes, va='top')
    for i, (label, val) in enumerate(summary):
        y = 0.85 - i * 0.1
        ax6.text(0.05, y, f'{label}:', color=DIM_COL,
                 fontsize=9, transform=ax6.transAxes, va='top')
        ax6.text(0.55, y, val, color=GREEN,
                 fontsize=9, fontweight='bold', transform=ax6.transAxes, va='top')

    plt.tight_layout(rect=[0, 0, 1, 0.96])
    return _save(fig, 'step7_final_result.png', 7, '最终综合结果')


# ═════════════════════════════════════════════════════════════
# 主流程
# ═════════════════════════════════════════════════════════════

def main():
    print('=' * 60)
    print('  网格表面全覆盖路径规划 — 分步可视化版')
    print('  Full-Coverage Path Planning — Step-by-Step Viz')
    print('=' * 60)

    os.makedirs(OUT_DIR, exist_ok=True)
    os.makedirs('outputs', exist_ok=True)

    # ── 加载网格 ──────────────────────────────
    ply_path = 'processed_mesh.ply'
    verts, faces = read_ply(ply_path)
    xyz     = verts[:, :3]
    normals = verts[:, 3:6]
    print(f'\n[加载] 顶点数: {len(xyz):,}, 面数: {len(faces):,}')

    # ★ Step 1
    viz_step1_mesh(xyz, normals, faces)

    # ── PCA ──────────────────────────────────
    center   = xyz.mean(axis=0)
    xyz_c    = xyz - center
    cov      = np.cov(xyz_c.T)
    eigvals, eigvecs = np.linalg.eigh(cov)
    idx_sort = np.argsort(eigvals)[::-1]
    eigvals  = eigvals[idx_sort]
    eigvecs  = eigvecs[:, idx_sort]
    u_axis, v_axis = eigvecs[:, 0], eigvecs[:, 1]
    proj_normal     = eigvecs[:, 2]
    avg_n = normals.mean(axis=0)
    avg_n /= np.linalg.norm(avg_n)
    if np.dot(proj_normal, avg_n) < 0:
        proj_normal = -proj_normal

    # ★ Step 2
    viz_step2_pca(xyz, normals, center, u_axis, v_axis, proj_normal, eigvals)

    # ── 2D 投影 & 凸包 ─────────────────────────
    xyz_2d = np.column_stack([xyz_c @ u_axis, xyz_c @ v_axis])
    hull   = ConvexHull(xyz_2d)
    hull_pts = xyz_2d[hull.vertices]

    # ★ Step 3
    viz_step3_boundary(xyz_2d, hull_pts, hull)

    # ── 路径规划参数 ──────────────────────────
    TOOL_DIAMETER = 0.04
    OVERLAP_RATIO = 0.15
    STEP_SIZE     = TOOL_DIAMETER * (1 - OVERLAP_RATIO)
    RESAMPLE_DIST = STEP_SIZE / 8

    u_range = xyz_2d[:, 0].max() - xyz_2d[:, 0].min()
    v_range = xyz_2d[:, 1].max() - xyz_2d[:, 1].min()
    scan_dir = 'x' if v_range >= u_range else 'y'

    path_2d = boustrophedon_coverage(hull_pts, STEP_SIZE, scan_dir)
    path_2d = resample_path(path_2d, RESAMPLE_DIST)

    # ★ Step 4
    viz_step4_boustrophedon(path_2d, hull_pts, STEP_SIZE)

    # ── 反投影 ────────────────────────────────
    print('\n[反投影] 将 2D 路径投影到 3D 网格表面（含重心坐标法向插值）...')
    kd_tree = KDTree(xyz)
    path_3d_approx = path_2d[:, 0:1] * u_axis + path_2d[:, 1:2] * v_axis + center

    proj_results    = point_to_mesh_projection_fast(path_3d_approx, xyz, normals)
    path_3d_raw     = np.array([r[0] for r in proj_results])
    normals_raw     = np.array([r[1] for r in proj_results])

    # ★ Step 5
    viz_step5_backprojection(xyz, faces, path_3d_raw, path_2d)

    # ── 平滑 ──────────────────────────────────
    path_3d_smooth  = smooth_path(path_3d_raw, window=3)
    re_proj         = point_to_mesh_projection_fast(path_3d_smooth, xyz, normals)
    normals_smooth  = np.array([r[1] for r in re_proj])
    normals_smooth  = smooth_normals(normals_smooth, window=3)

    # ★ Step 6
    viz_step6_smoothing(path_3d_raw, path_3d_smooth, normals_raw, normals_smooth)

    # ★ Step 7（最终结果）
    viz_step7_final(xyz, normals, faces,
                    path_3d_smooth, normals_smooth,
                    path_2d, hull_pts, STEP_SIZE)

    # ── 导出 CSV ──────────────────────────────
    export_path_csv(path_3d_smooth, normals_smooth, 'outputs/coverage_path.csv')

    # ── 汇总 ──────────────────────────────────
    print(f'\n{"="*60}')
    print('  分步可视化完成！输出文件：')
    for i, name in enumerate([
        'step1_mesh_parse.png',
        'step2_pca_projection.png',
        'step3_2d_boundary.png',
        'step4_boustrophedon.png',
        'step5_3d_backprojection.png',
        'step6_smoothing_normals.png',
        'step7_final_result.png',
    ], 1):
        print(f'    Step {i}: {OUT_DIR}/{name}')
    print(f'    CSV:    outputs/coverage_path.csv')
    print(f'{"="*60}')


if __name__ == '__main__':
    main()


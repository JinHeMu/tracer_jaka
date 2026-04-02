#!/usr/bin/env python3
"""
point_cloud_processor_node.py
==============================
ROS 2 节点：整合点云裁剪、预处理/重建、全覆盖路径规划三个模块。

提供的接口
----------
服务（Service）
  /crop_point_cloud   [CropPointCloud]
      快速同步裁剪，适合轻量交互。

动作（Action）
  /process_point_cloud  [ProcessPointCloud]
      完整预处理流水线：裁剪 → 降采样 → 滤波 → 法线 → 泊松重建
  /plan_coverage_path   [PlanCoveragePath]
      全覆盖路径规划：读网格 → PCA → Boustrophedon → 投影 → 平滑 → 导出

参数由 config/params.yaml 统一管理，节点启动时自动加载。
"""

import os
import time
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

# ROS 2 接口（由本包 CMakeLists.txt 生成）
from point_cloud_processor.srv import CropPointCloud
from point_cloud_processor.action import ProcessPointCloud, PlanCoveragePath


# ═══════════════════════════════════════════════════════════════
#  辅助：从参数服务器读列表
# ═══════════════════════════════════════════════════════════════
def _get_list(node: Node, name: str, default):
    try:
        val = node.get_parameter(name).value
        return list(val) if val is not None else default
    except Exception:
        return default


# ═══════════════════════════════════════════════════════════════
#  模块 1：空间裁剪（对应 box_crop.py）
# ═══════════════════════════════════════════════════════════════
class BoxCropper:
    """封装 Open3D AxisAlignedBoundingBox 裁剪逻辑"""

    @staticmethod
    def crop(input_path: str,
             output_path: str,
             min_bound: list,
             max_bound: list,
             logger=None) -> tuple[bool, str, int]:
        """
        Returns
        -------
        (success, message, point_count)
        """
        import open3d as o3d

        log = logger.info if logger else print

        log(f"[BoxCropper] 读取点云: {input_path}")
        pcd = o3d.io.read_point_cloud(input_path)
        if pcd.is_empty():
            msg = f"无法读取点云文件: {input_path}"
            return False, msg, 0

        bbox = o3d.geometry.AxisAlignedBoundingBox(
            min_bound=np.array(min_bound),
            max_bound=np.array(max_bound)
        )
        cropped = pcd.crop(bbox)
        n = len(cropped.points)

        if n == 0:
            return False, "裁剪后点云为空，请检查边界参数", 0

        os.makedirs(os.path.dirname(output_path) or ".", exist_ok=True)
        o3d.io.write_point_cloud(output_path, cropped)
        log(f"[BoxCropper] 裁剪完成，保留 {n} 个点 → {output_path}")
        return True, "裁剪成功", n


# ═══════════════════════════════════════════════════════════════
#  模块 2：预处理 + 泊松重建（对应 pre_process.py）
# ═══════════════════════════════════════════════════════════════
class PointCloudPreprocessor:
    """降采样 → 统计/半径滤波 → 法线估计 → 泊松重建"""

    def __init__(self, params: dict, logger=None):
        self.p = params
        self.log = logger.info if logger else print
        self.logw = logger.warn if logger else print

    def run(self, input_pcd_path: str,
            output_ply_path: str,
            feedback_cb=None) -> tuple[bool, str, int, int]:
        """
        feedback_cb(stage, desc, progress) — 可选回调，用于动作 Feedback

        Returns
        -------
        (success, message, point_count, face_count)
        """
        import open3d as o3d

        def fb(stage, desc, prog):
            self.log(f"  [{stage}] {desc} ({prog:.0f}%)")
            if feedback_cb:
                feedback_cb(stage, desc, prog)

        # ── 1. 加载 ────────────────────────────────────────────
        fb(0, "加载原始点云", 5)
        pcd = o3d.io.read_point_cloud(input_pcd_path)
        if pcd.is_empty():
            return False, f"无法读取: {input_pcd_path}", 0, 0
        self.log(f"[Preprocessor] 原始点数: {len(pcd.points)}")

        # ── 2. 降采样 ──────────────────────────────────────────
        fb(1, "体素降采样", 20)
        voxel = self.p["voxel_size"]
        pcd_down = pcd.voxel_down_sample(voxel_size=voxel)
        self.log(f"[Preprocessor] 降采样后: {len(pcd_down.points)} 点")

        # ── 3. 统计滤波 ────────────────────────────────────────
        fb(2, "统计滤波", 35)
        _, ind = pcd_down.remove_statistical_outlier(
            nb_neighbors=self.p["sor_nb_neighbors"],
            std_ratio=self.p["sor_std_ratio"]
        )
        pcd_sor = pcd_down.select_by_index(ind)

        # ── 4. 半径滤波 ────────────────────────────────────────
        fb(2, "半径滤波", 45)
        radius = voxel * self.p["radius_multiplier"]
        _, ind = pcd_sor.remove_radius_outlier(
            nb_points=self.p["radius_nb_points"],
            radius=radius
        )
        pcd_clean = pcd_sor.select_by_index(ind)
        self.log(f"[Preprocessor] 清洗后: {len(pcd_clean.points)} 点")

        # ── 5. 法线估计 ────────────────────────────────────────
        fb(3, "法线估计", 60)
        r_normal = voxel * self.p["normal_radius_multiplier"]
        pcd_clean.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=r_normal,
                max_nn=self.p["normal_max_nn"]
            )
        )

        # 统一让法线朝向原点
        pcd_clean.orient_normals_towards_camera_location(
            camera_location=np.array([0.0, 0.0, 0.0])
        )

        # ── 6. 泊松重建 ────────────────────────────────────────
        fb(4, "泊松表面重建", 75)
        mesh, densities = \
            o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
                pcd_clean,
                depth=self.p["poisson_depth"]
            )
        dens_arr = np.asarray(densities)
        thresh = np.quantile(dens_arr, self.p["poisson_density_quantile"])
        mask = dens_arr < thresh
        mesh.remove_vertices_by_mask(mask)
        mesh.compute_vertex_normals()

        # ── 7. 后处理 & 裁剪 ──────────────────────────────────
        fb(4, "网格后处理", 90)
        mesh.remove_degenerate_triangles()
        mesh.remove_duplicated_vertices()
        mesh.remove_unreferenced_vertices()
        bbox = pcd_clean.get_axis_aligned_bounding_box()
        mesh = mesh.crop(bbox)

        face_count = len(np.asarray(mesh.triangles))
        pt_count   = len(pcd_clean.points)

        # ── 8. 保存 ────────────────────────────────────────────
        fb(4, "保存网格文件", 98)
        os.makedirs(os.path.dirname(output_ply_path) or ".", exist_ok=True)
        o3d.io.write_triangle_mesh(output_ply_path, mesh)
        self.log(f"[Preprocessor] 网格已保存: {output_ply_path}"
                 f"（{face_count} 面）")

        fb(4, "完成", 100)
        return True, "预处理完成", pt_count, face_count


# ═══════════════════════════════════════════════════════════════
#  模块 3：全覆盖路径规划（对应 mesh_coverage_path.py）
# ═══════════════════════════════════════════════════════════════
class CoveragePathPlanner:
    """Boustrophedon 全覆盖路径规划，基于 mesh_coverage_path.py"""

    def __init__(self, params: dict, logger=None):
        self.p = params
        self.log = logger.info if logger else print

    # ── 内部工具方法（逐一对应原脚本函数）─────────────────────

    @staticmethod
    def _build_raycasting_scene(mesh_o3d):
        import open3d as o3d
        import open3d.core as o3c
        scene  = o3d.t.geometry.RaycastingScene()
        mesh_t = o3d.t.geometry.TriangleMesh.from_legacy(mesh_o3d)
        scene.add_triangles(mesh_t)
        return scene

    @staticmethod
    def _project_to_mesh(query_pts, scene, mesh_o3d):
        import open3d.core as o3c
        import numpy as np
        query_t = o3c.Tensor(query_pts.astype(np.float32), dtype=o3c.float32)
        result  = scene.compute_closest_points(query_t)
        proj    = result['points'].numpy().astype(np.float64)
        tri_ids = result['primitive_ids'].numpy()

        triangles = np.asarray(mesh_o3d.triangles)
        vertices  = np.asarray(mesh_o3d.vertices)
        tv = triangles[tri_ids]
        v0, v1, v2 = vertices[tv[:,0]], vertices[tv[:,1]], vertices[tv[:,2]]
        fn = np.cross(v1 - v0, v2 - v0)
        norms = np.linalg.norm(fn, axis=1, keepdims=True)
        safe  = norms > 1e-12
        fn = np.where(safe, fn / np.where(safe, norms, 1.0),
                      np.array([[0., 1., 0.]]))
        return proj, fn

    @staticmethod
    def _resample_path(pts, spacing):
        if len(pts) < 2:
            return pts
        out = [pts[0]]
        accum = 0.0
        for i in range(1, len(pts)):
            seg = pts[i] - pts[i - 1]
            seg_len = np.linalg.norm(seg)
            if seg_len < 1e-12:
                continue
            while accum + seg_len >= spacing:
                t = (spacing - accum) / seg_len
                new_pt = pts[i - 1] + t * seg
                out.append(new_pt)
                pts[i - 1] = new_pt
                seg = pts[i] - pts[i - 1]
                seg_len -= spacing - accum
                accum = 0.0
            accum += seg_len
        out.append(pts[-1])
        return np.array(out)

    @staticmethod
    def _boustrophedon(xyz_2d, hull_pts, step_size, scan_dir='x'):
        if scan_dir == 'y':
            pts  = hull_pts[:, [1, 0]]
        else:
            pts  = hull_pts.copy()

        min_v = pts[:, 1].min()
        max_v = pts[:, 1].max()
        v_lines = np.arange(min_v + step_size / 2, max_v, step_size)
        if len(v_lines) == 0:
            v_lines = np.array([(min_v + max_v) / 2])

        segments = []
        n = len(pts)
        for vi, v in enumerate(v_lines):
            crossings = []
            for i in range(n):
                a, b = pts[i], pts[(i + 1) % n]
                if (a[1] <= v < b[1]) or (b[1] <= v < a[1]):
                    t_c = (v - a[1]) / (b[1] - a[1] + 1e-15)
                    crossings.append(a[0] + t_c * (b[0] - a[0]))
            if len(crossings) < 2:
                continue
            crossings.sort()
            u_start, u_end = crossings[0], crossings[-1]
            u_pts = np.arange(u_start, u_end + 1e-9, step_size / 4)
            if len(u_pts) == 0:
                u_pts = np.array([(u_start + u_end) / 2])
            if vi % 2 == 1:
                u_pts = u_pts[::-1]
            segments.append(np.column_stack([u_pts, np.full(len(u_pts), v)]))

        if not segments:
            return np.array([]).reshape(0, 2)

        full = []
        for i, seg in enumerate(segments):
            full.append(seg)
            if i < len(segments) - 1:
                full.append(np.array([seg[-1], segments[i + 1][0]]))
        path_2d = np.vstack(full)

        if scan_dir == 'y':
            path_2d = path_2d[:, [1, 0]]
        return path_2d

    @staticmethod
    def _smooth_array(arr, window):
        if len(arr) < window * 2 + 1:
            return arr
        out = arr.copy()
        for i in range(window, len(arr) - window):
            avg = arr[i - window : i + window + 1].mean(axis=0)
            norm = np.linalg.norm(avg)
            out[i] = avg / norm if norm > 1e-12 else avg
        return out

    @staticmethod
    def _compute_directions(path_3d):
        N = len(path_3d)
        d = np.zeros_like(path_3d)
        if N < 2:
            return d
        d[1:-1] = path_3d[2:] - path_3d[:-2]
        d[0]    = path_3d[1] - path_3d[0]
        d[-1]   = path_3d[-1] - path_3d[-2]
        norms = np.linalg.norm(d, axis=1, keepdims=True)
        safe  = norms > 1e-12
        return np.where(safe, d / np.where(safe, norms, 1.0),
                        np.array([[1., 0., 0.]]))

    @staticmethod
    def _filter_outliers(path_3d, normals, dirs, window, angle_thresh_deg):
        N = len(dirs)
        cos_thresh = np.cos(np.radians(angle_thresh_deg))
        mask = np.ones(N, dtype=bool)
        for i in range(N):
            i0, i1 = max(0, i - window), min(N, i + window + 1)
            nbrs = np.vstack([dirs[i0:i], dirs[i + 1:i1]]) \
                   if i > i0 else dirs[i + 1:i1]
            if len(nbrs) < 3:
                continue
            mean_d = nbrs.mean(axis=0)
            mn = np.linalg.norm(mean_d)
            if mn < 1e-12:
                continue
            if np.dot(dirs[i], mean_d / mn) < cos_thresh:
                mask[i] = False
        removed = int((~mask).sum())
        return path_3d[mask], normals[mask], dirs[mask], mask, removed

    # ── 主流程 ─────────────────────────────────────────────────

    def run(self, ply_path: str,
            csv_path: str,
            gcode_path: str,
            feedback_cb=None) -> tuple[bool, str, int, float, float]:
        """
        Returns
        -------
        (success, message, point_count, total_length, coverage_pct)
        """
        import open3d as o3d
        from scipy.spatial import ConvexHull

        p = self.p

        def fb(stage, desc, prog):
            self.log(f"  [path {stage}] {desc} ({prog:.0f}%)")
            if feedback_cb:
                feedback_cb(stage, desc, prog)

        # ── 1. 读取网格 ──────────────────────────────────────
        fb(0, "读取网格文件", 5)
        mesh = o3d.io.read_triangle_mesh(ply_path)
        if not mesh.has_vertex_normals():
            mesh.compute_vertex_normals()
        xyz     = np.asarray(mesh.vertices)
        normals = np.asarray(mesh.vertex_normals)
        faces   = np.asarray(mesh.triangles, dtype=np.int32)
        self.log(f"[PathPlanner] 顶点: {len(xyz)}, 面: {len(faces)}")

        # ── 2. 构建 RaycastingScene ─────────────────────────
        fb(0, "构建 RaycastingScene", 12)
        scene = self._build_raycasting_scene(mesh)

        # ── 3. 表面积 & 包围盒 ───────────────────────────────
        v0s = xyz[faces[:, 0]]; v1s = xyz[faces[:, 1]]; v2s = xyz[faces[:, 2]]
        crosses = np.cross(v1s - v0s, v2s - v0s)
        total_area = 0.5 * np.linalg.norm(crosses, axis=1).sum()
        self.log(f"[PathPlanner] 表面积: {total_area:.4f} m²")

        # ── 4. PCA 求曲面法向 + 重建与世界 X 轴对齐的坐标系 ──────
        fb(1, "PCA 求投影平面（X 轴对齐模式）", 20)
        center = xyz.mean(axis=0)
        cov    = np.cov((xyz - center).T)
        eigvals, eigvecs = np.linalg.eigh(cov)
        idx     = np.argsort(eigvals)[::-1]
        eigvecs = eigvecs[:, idx]

        # PCA 最小特征值对应曲面法向量
        proj_normal = eigvecs[:, 2]

        # 令法向量与顶点法线均值方向一致
        avg_n = normals.mean(axis=0)
        avg_n_len = np.linalg.norm(avg_n)
        if avg_n_len > 1e-12:
            avg_n /= avg_n_len
            if np.dot(proj_normal, avg_n) < 0:
                proj_normal = -proj_normal

        # 将世界 X 轴投影到曲面切平面，作为扫描行方向 u_axis
        WORLD_X = np.array([1.0, 0.0, 0.0])
        u_axis  = WORLD_X - np.dot(WORLD_X, proj_normal) * proj_normal
        u_norm  = np.linalg.norm(u_axis)

        if u_norm < 1e-6:
            # 退化情况：法向量与 X 轴几乎平行，改用 Y 轴
            self.log("[PathPlanner] 警告：曲面法向接近世界 X 轴，自动改用 Y 轴作为扫描方向")
            WORLD_X = np.array([0.0, 1.0, 0.0])
            u_axis  = WORLD_X - np.dot(WORLD_X, proj_normal) * proj_normal
            u_axis /= np.linalg.norm(u_axis)
        else:
            u_axis /= u_norm

        # 右手系叉乘得 v_axis（步进方向）
        v_axis = np.cross(proj_normal, u_axis)
        v_axis /= np.linalg.norm(v_axis)

        angle_deg = float(np.degrees(np.arccos(np.clip(np.dot(u_axis, [1, 0, 0]), -1, 1))))
        self.log(f"[PathPlanner] 扫描行方向 u_axis: {np.round(u_axis, 4)}"
                f"  ← 与世界 X 轴夹角: {angle_deg:.2f}°")

        xyz_c  = xyz - center
        xyz_2d = np.column_stack([xyz_c @ u_axis, xyz_c @ v_axis])
        hull   = ConvexHull(xyz_2d)
        hull_pts = xyz_2d[hull.vertices]


        # ── 5. Boustrophedon 路径生成 ────────────────────────
        fb(2, "生成 Boustrophedon 路径", 35)
        tool_d    = p["tool_diameter"]
        step_size = tool_d * (1 - p["overlap_ratio"])
        resamp    = step_size / p["resample_divisor"]
        path_2d   = self._boustrophedon(xyz_2d, hull_pts, step_size,
                                         p["scan_direction"])
        if len(path_2d) == 0:
            return False, "路径生成失败，请检查网格和规划参数", 0, 0.0, 0.0

        path_2d = self._resample_path(path_2d, resamp)
        self.log(f"[PathPlanner] 重采样后点数: {len(path_2d)}")

        # ── 6. 反投影到 3D ───────────────────────────────────
        fb(3, "2D→3D 网格投影", 55)
        approx_3d = path_2d[:, 0:1] * u_axis + path_2d[:, 1:2] * v_axis + center
        path_3d, path_normals = self._project_to_mesh(approx_3d, scene, mesh)

        # ── 7. 路径平滑 → 重投影 ─────────────────────────────
        fb(3, "路径平滑", 65)
        smoothed = path_3d.copy()
        w = p["smooth_path_window"]
        for i in range(w, len(path_3d) - w):
            smoothed[i] = path_3d[i - w : i + w + 1].mean(axis=0)
        path_3d, path_normals = self._project_to_mesh(smoothed, scene, mesh)

        # ── 8. 法向平滑 & 方向计算 ───────────────────────────
        fb(3, "法向量平滑 & 方向向量计算", 75)
        path_normals   = self._smooth_array(path_normals,
                                            p["smooth_normal_window"])
        path_directions = self._compute_directions(path_3d)
        path_directions = self._smooth_array(path_directions,
                                             p["smooth_direction_window"])

        # ── 9. 跳变点剔除 ────────────────────────────────────
        fb(3, "跳变点检测与剔除", 85)
        path_3d, path_normals, path_directions, _, removed = \
            self._filter_outliers(
                path_3d, path_normals, path_directions,
                p["outlier_window"],
                p["outlier_angle_threshold_deg"]
            )
        self.log(f"[PathPlanner] 剔除跳变点: {removed}")
        path_directions = self._smooth_array(path_directions, 5)

        # ── 10. 统计 & 导出 ──────────────────────────────────
        seg_lens  = np.linalg.norm(np.diff(path_3d, axis=0), axis=1)
        total_len = float(seg_lens.sum())
        coverage  = min(100.0, total_len * tool_d / total_area * 100)
        n_pts     = len(path_3d)

        self.log(f"[PathPlanner] 总路径长度: {total_len:.3f} m, "
                 f"点数: {n_pts}, 覆盖率: {coverage:.1f}%")

        fb(4, "导出 CSV / G-code", 92)
        self._export(path_3d, path_normals, path_directions,
                     csv_path, gcode_path)

        fb(4, "完成", 100)
        return True, "路径规划完成", n_pts, total_len, coverage

    def _export(self, path_3d, normals, dirs, csv_path, gcode_path):
        import csv as csv_mod

        os.makedirs(os.path.dirname(csv_path) or ".", exist_ok=True)
        with open(csv_path, 'w', newline='') as f:
            w = csv_mod.writer(f)
            w.writerow(['idx', 'x', 'y', 'z',
                        'nx', 'ny', 'nz',
                        'dx', 'dy', 'dz',
                        'seg_len', 'cumulative_len'])
            cum = 0.0
            for i, (pt, n, d) in enumerate(zip(path_3d, normals, dirs)):
                seg = float(np.linalg.norm(pt - path_3d[i - 1])) if i > 0 else 0.0
                cum += seg
                w.writerow([i,
                             f'{pt[0]:.6f}', f'{pt[1]:.6f}', f'{pt[2]:.6f}',
                             f'{n[0]:.6f}',  f'{n[1]:.6f}',  f'{n[2]:.6f}',
                             f'{d[0]:.6f}',  f'{d[1]:.6f}',  f'{d[2]:.6f}',
                             f'{seg:.6f}',   f'{cum:.6f}'])
        self.log(f"[PathPlanner] CSV 已保存: {csv_path}")

        if gcode_path:
            os.makedirs(os.path.dirname(gcode_path) or ".", exist_ok=True)
            with open(gcode_path, 'w') as f:
                f.write("; Full-Coverage Path — generated by point_cloud_processor\n")
                f.write(f"; Total points: {len(path_3d)}\n\n")
                f.write("G21\nG90\nG28\n\n")
                threshold = 0.05
                prev_transition = False
                for i, pt in enumerate(path_3d):
                    mm = pt * 1000
                    if i > 0:
                        dist = np.linalg.norm(pt - path_3d[i - 1])
                        is_trans = dist > threshold
                        if is_trans and not prev_transition:
                            f.write("\n; -- transition --\n")
                        prev_transition = is_trans
                    f.write(f"G1 X{mm[0]:.3f} Y{mm[1]:.3f} Z{mm[2]:.3f} F500\n")
                f.write("\nM2 ; end\n")
            self.log(f"[PathPlanner] G-code 已保存: {gcode_path}")


# ═══════════════════════════════════════════════════════════════
#  ROS 2 主节点
# ═══════════════════════════════════════════════════════════════
class PointCloudProcessorNode(Node):

    def __init__(self):
        super().__init__('point_cloud_processor')
        self._cb_group = ReentrantCallbackGroup()

        # ── 声明并加载参数 ────────────────────────────────────
        self._declare_parameters()
        pp = self._build_preprocess_params()
        pl = self._build_path_params()

        self._preprocessor = PointCloudPreprocessor(pp, self.get_logger())
        self._planner       = CoveragePathPlanner(pl, self.get_logger())

        # ── 服务：裁剪 ────────────────────────────────────────
        self._crop_srv = self.create_service(
            CropPointCloud,
            'crop_point_cloud',
            self._handle_crop,
            callback_group=self._cb_group
        )

        # ── 动作：预处理 + 重建 ───────────────────────────────
        self._process_action = ActionServer(
            self,
            ProcessPointCloud,
            'process_point_cloud',
            execute_callback=self._execute_process,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=self._cb_group
        )

        # ── 动作：路径规划 ────────────────────────────────────
        self._plan_action = ActionServer(
            self,
            PlanCoveragePath,
            'plan_coverage_path',
            execute_callback=self._execute_plan,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=self._cb_group
        )

        self.get_logger().info(
            "PointCloudProcessorNode 已启动\n"
            "  服务  : /crop_point_cloud\n"
            "  动作  : /process_point_cloud\n"
            "  动作  : /plan_coverage_path"
        )

        # auto_pipeline 模式
        if self.get_parameter('node.auto_pipeline').value:
            self.get_logger().info("[auto_pipeline] 自动流水线已启用，"
                                   "将在启动后立即执行完整流程。")
            threading.Thread(target=self._auto_pipeline, daemon=True).start()

    # ── 参数声明 ──────────────────────────────────────────────

    def _declare_parameters(self):
        self.declare_parameter('paths.input_pcd',    'data/frame_00000.pcd')
        self.declare_parameter('paths.output_pcd',   'data/cropped_cloud.pcd')
        self.declare_parameter('paths.output_ply',   'data/processed_mesh.ply')
        self.declare_parameter('paths.output_csv',   'outputs/coverage_path.csv')
        self.declare_parameter('paths.output_gcode', 'outputs/coverage_path.gcode')

        self.declare_parameter('crop.min_bound', [0.1, -1.0, 0.5])
        self.declare_parameter('crop.max_bound', [0.5,  2.0, 0.8])

        self.declare_parameter('preprocess.voxel_size',                  0.01)
        self.declare_parameter('preprocess.sor_nb_neighbors',            30)
        self.declare_parameter('preprocess.sor_std_ratio',               1.5)
        self.declare_parameter('preprocess.radius_nb_points',            12)
        self.declare_parameter('preprocess.radius_multiplier',           2.5)
        self.declare_parameter('preprocess.normal_radius_multiplier',    3.0)
        self.declare_parameter('preprocess.normal_max_nn',               30)
        self.declare_parameter('preprocess.orient_tangent_k',            100)
        self.declare_parameter('preprocess.poisson_depth',               9)
        self.declare_parameter('preprocess.poisson_density_quantile',    0.1)

        self.declare_parameter('path_planning.tool_diameter',            0.04)
        self.declare_parameter('path_planning.overlap_ratio',            0.15)
        self.declare_parameter('path_planning.scan_direction',           'x')
        self.declare_parameter('path_planning.resample_divisor',         8)
        self.declare_parameter('path_planning.smooth_path_window',       3)
        self.declare_parameter('path_planning.smooth_normal_window',     10)
        self.declare_parameter('path_planning.smooth_direction_window',  8)
        self.declare_parameter('path_planning.outlier_window',           10)
        self.declare_parameter('path_planning.outlier_angle_threshold_deg', 45.0)
        self.declare_parameter('path_planning.normal_display_length',    0.015)

        self.declare_parameter('node.auto_pipeline', False)
        self.declare_parameter('node.log_level',     'INFO')

    def _gp(self, name):
        return self.get_parameter(name).value

    def _build_preprocess_params(self) -> dict:
        return {
            "voxel_size":                self._gp('preprocess.voxel_size'),
            "sor_nb_neighbors":          self._gp('preprocess.sor_nb_neighbors'),
            "sor_std_ratio":             self._gp('preprocess.sor_std_ratio'),
            "radius_nb_points":          self._gp('preprocess.radius_nb_points'),
            "radius_multiplier":         self._gp('preprocess.radius_multiplier'),
            "normal_radius_multiplier":  self._gp('preprocess.normal_radius_multiplier'),
            "normal_max_nn":             self._gp('preprocess.normal_max_nn'),
            "orient_tangent_k":          self._gp('preprocess.orient_tangent_k'),
            "poisson_depth":             self._gp('preprocess.poisson_depth'),
            "poisson_density_quantile":  self._gp('preprocess.poisson_density_quantile'),
        }

    def _build_path_params(self) -> dict:
        return {
            "tool_diameter":             self._gp('path_planning.tool_diameter'),
            "overlap_ratio":             self._gp('path_planning.overlap_ratio'),
            "scan_direction":            self._gp('path_planning.scan_direction'),
            "resample_divisor":          self._gp('path_planning.resample_divisor'),
            "smooth_path_window":        self._gp('path_planning.smooth_path_window'),
            "smooth_normal_window":      self._gp('path_planning.smooth_normal_window'),
            "smooth_direction_window":   self._gp('path_planning.smooth_direction_window'),
            "outlier_window":            self._gp('path_planning.outlier_window'),
            "outlier_angle_threshold_deg": self._gp('path_planning.outlier_angle_threshold_deg'),
        }

    # ── 通用动作回调 ──────────────────────────────────────────

    @staticmethod
    def _goal_cb(_goal_request):
        return GoalResponse.ACCEPT

    @staticmethod
    def _cancel_cb(_goal_handle):
        return CancelResponse.ACCEPT

    # ── 服务实现：裁剪 ────────────────────────────────────────

    def _handle_crop(self,
                     request: CropPointCloud.Request,
                     response: CropPointCloud.Response):
        self.get_logger().info("[Service] crop_point_cloud 请求收到")

        in_path  = request.input_pcd_path  or self._gp('paths.input_pcd')
        out_path = request.output_pcd_path or self._gp('paths.output_pcd')
        min_b = list(request.min_bound) if any(request.min_bound) \
                else self._gp('crop.min_bound')
        max_b = list(request.max_bound) if any(request.max_bound) \
                else self._gp('crop.max_bound')

        ok, msg, n = BoxCropper.crop(in_path, out_path, min_b, max_b,
                                     self.get_logger())
        response.success       = ok
        response.message       = msg
        response.point_count   = n
        response.output_pcd_path = out_path
        return response

    # ── 动作实现：预处理 ─────────────────────────────────────

    async def _execute_process(self, goal_handle):
        self.get_logger().info("[Action] process_point_cloud 开始执行")
        goal = goal_handle.request

        in_path  = goal.input_pcd_path  or self._gp('paths.output_pcd')
        out_path = goal.output_ply_path or self._gp('paths.output_ply')

        feedback_msg = ProcessPointCloud.Feedback()

        def fb_cb(stage, desc, prog):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                raise RuntimeError("动作已被取消")
            feedback_msg.stage             = stage
            feedback_msg.stage_description = desc
            feedback_msg.progress          = float(prog)
            goal_handle.publish_feedback(feedback_msg)

        result = ProcessPointCloud.Result()
        try:
            ok, msg, pt_cnt, face_cnt = self._preprocessor.run(
                in_path, out_path, feedback_cb=fb_cb
            )
            result.success        = ok
            result.message        = msg
            result.output_ply_path = out_path
            result.point_count    = pt_cnt
            result.face_count     = face_cnt
        except RuntimeError as e:
            result.success  = False
            result.message  = str(e)
            goal_handle.canceled()
            return result
        except Exception as e:
            self.get_logger().error(f"[Action] process_point_cloud 异常: {e}")
            result.success = False
            result.message = str(e)

        if result.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    # ── 动作实现：路径规划 ───────────────────────────────────

    async def _execute_plan(self, goal_handle):
        self.get_logger().info("[Action] plan_coverage_path 开始执行")
        goal = goal_handle.request

        ply_path   = goal.input_ply_path   or self._gp('paths.output_ply')
        csv_path   = goal.output_csv_path  or self._gp('paths.output_csv')
        gcode_path = goal.output_gcode_path or self._gp('paths.output_gcode')

        feedback_msg = PlanCoveragePath.Feedback()

        def fb_cb(stage, desc, prog):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                raise RuntimeError("动作已被取消")
            feedback_msg.stage             = stage
            feedback_msg.stage_description = desc
            feedback_msg.progress          = float(prog)
            goal_handle.publish_feedback(feedback_msg)

        result = PlanCoveragePath.Result()
        try:
            ok, msg, n_pts, length, cov = self._planner.run(
                ply_path, csv_path, gcode_path, feedback_cb=fb_cb
            )
            result.success          = ok
            result.message          = msg
            result.output_csv_path  = csv_path
            result.output_gcode_path = gcode_path
            result.point_count      = n_pts
            result.total_length_m   = float(length)
            result.coverage_percent = float(cov)
        except RuntimeError as e:
            result.success = False
            result.message = str(e)
            goal_handle.canceled()
            return result
        except Exception as e:
            self.get_logger().error(f"[Action] plan_coverage_path 异常: {e}")
            result.success = False
            result.message = str(e)

        if result.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    # ── auto_pipeline ────────────────────────────────────────

    def _auto_pipeline(self):
        """节点启动后自动依次执行：裁剪 → 预处理 → 路径规划"""
        time.sleep(1.0)  # 等待节点完全初始化
        self.get_logger().info("[auto_pipeline] 步骤 1/3: 裁剪点云")

        ok, msg, _ = BoxCropper.crop(
            self._gp('paths.input_pcd'),
            self._gp('paths.output_pcd'),
            self._gp('crop.min_bound'),
            self._gp('crop.max_bound'),
            self.get_logger()
        )
        if not ok:
            self.get_logger().error(f"[auto_pipeline] 裁剪失败: {msg}")
            return

        self.get_logger().info("[auto_pipeline] 步骤 2/3: 预处理 + 重建")
        ok, msg, _, _ = self._preprocessor.run(
            self._gp('paths.output_pcd'),
            self._gp('paths.output_ply')
        )
        if not ok:
            self.get_logger().error(f"[auto_pipeline] 预处理失败: {msg}")
            return

        self.get_logger().info("[auto_pipeline] 步骤 3/3: 路径规划")
        ok, msg, n_pts, length, cov = self._planner.run(
            self._gp('paths.output_ply'),
            self._gp('paths.output_csv'),
            self._gp('paths.output_gcode')
        )
        if ok:
            self.get_logger().info(
                f"[auto_pipeline] 完成！路径点: {n_pts}, "
                f"长度: {length:.3f}m, 覆盖率: {cov:.1f}%"
            )
        else:
            self.get_logger().error(f"[auto_pipeline] 路径规划失败: {msg}")


# ═══════════════════════════════════════════════════════════════
#  入口
# ═══════════════════════════════════════════════════════════════
def main(args=None):
    rclpy.init(args=args)
    node = PointCloudProcessorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

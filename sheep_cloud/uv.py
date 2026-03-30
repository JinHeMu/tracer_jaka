import numpy as np
import open3d as o3d
import igl
import matplotlib.pyplot as plt
import matplotlib.tri as mtri  # <--- 【关键修复】：补上这一行！
from shapely.geometry import Polygon, LineString, MultiLineString
from scipy.spatial import Delaunay
import os


# ================= 阶段一：数据加载与清洗 =================
def load_custom_mesh(file_path):
    """加载自定义的 PLY/OBJ 网格文件，并执行严苛的拓扑清理"""
    print(f"1. 正在读取网格文件: {file_path} ...")
    mesh = o3d.io.read_triangle_mesh(file_path)

    if mesh.is_empty():
        raise ValueError(f"无法读取文件 {file_path}，请检查路径是否正确！")

    # 【工程避坑】清理非流形结构
    mesh.remove_degenerate_triangles()  # 移除面积为 0 的退化三角形
    mesh.remove_duplicated_vertices()  # 移除重合的顶点
    mesh.remove_duplicated_triangles()  # 移除重合的面
    mesh.remove_unreferenced_vertices()  # 移除没有连接任何面的孤立顶点

    return mesh


def generate_test_open_mesh():
    """备用方案：生成一个用于测试的开放 3D 曲面 (波浪面)"""
    print("1. 未找到目标文件，正在生成测试用 3D 波浪网格...")
    x = np.linspace(-1, 1, 40)
    y = np.linspace(-1, 1, 40)
    X, Y = np.meshgrid(x, y)
    Z = 0.4 * np.sin(X * np.pi) * np.cos(Y * np.pi)

    pts = np.vstack([X.flatten(), Y.flatten(), Z.flatten()]).T
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)
    pcd.estimate_normals()

    mesh, _ = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=8)
    bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=(-0.8, -0.8, -1), max_bound=(0.8, 0.8, 1))
    mesh = mesh.crop(bbox)

    return mesh


# ================= 阶段二：3D 到 2D 的 UV 展开 =================
def parameterize_mesh_to_2d(v, f):
    """使用 libigl 执行调和映射，将 3D 网格展开到 2D UV 域"""
    print("2. 开始计算 3D 到 2D 的调和参数化...")
    bnd = igl.boundary_loop(f)
    if len(bnd) == 0:
        raise ValueError("致命错误：网格没有边界！闭合曲面无法展开，请检查第一阶段的裁剪。")

    print(f"   成功找到 {len(bnd)} 个边界顶点。")
    bnd_uv = igl.map_vertices_to_circle(v, bnd)

    # 执行一阶调和映射
    uv = igl.harmonic(v, f, bnd, bnd_uv, 1)
    return uv, bnd


# ================= 阶段三：在 2D 域生成蛇形轨迹 =================
def generate_lawnmower_path(uv_coords, boundary_indices, spacing=0.05):
    """在 2D UV 域内生成 Boustrophedon (蛇形/割草机) 轨迹"""
    print(f"\n3. 开始在 2D 域生成间距为 {spacing} 的 Lawnmower 轨迹...")

    boundary_pts = uv_coords[boundary_indices]
    poly = Polygon(boundary_pts)

    if not poly.is_valid:
        poly = poly.buffer(0)  # Shapely 修复自相交

    minx, miny, maxx, maxy = poly.bounds

    path_points = []
    current_y = miny + spacing / 2.0
    direction = 1  # 1 表示左到右, -1 表示右到左

    while current_y < maxy:
        # 水平射线扫掠
        scanline = LineString([(minx - 1, current_y), (maxx + 1, current_y)])
        intersection = scanline.intersection(poly)
        if intersection.is_empty:
            current_y += spacing
            continue

        # 提取交点
        line_segments_pts = []
        if isinstance(intersection, LineString):
            line_segments_pts.extend(list(intersection.coords))
        elif isinstance(intersection, MultiLineString):
            for geom in intersection.geoms:
                line_segments_pts.extend(list(geom.coords))

        if not line_segments_pts:
            current_y += spacing
            continue

        # 蛇形链接排序
        sorted_pts = sorted(line_segments_pts, key=lambda p: p[0], reverse=(direction == -1))
        path_points.extend(sorted_pts)

        direction *= -1
        current_y += spacing

    print(f"   轨迹生成完毕！共生成 {len(path_points)} 个 2D 轨迹关键点。")
    return np.array(path_points), poly


# ================= 阶段四：2D 轨迹逆映射回 3D 曲面 =================
def map_2d_path_to_3d(path_2d, uv_2d, v_3d, f_3d, mesh_3d):
    """
    阶段四（终极修复版）：强制使用原始网格拓扑 f_3d 查找重心，实现 100% 完美贴面！
    """
    print("\n4. 开始执行 2D 到 3D 的精准逆向重心映射...")

    if not mesh_3d.has_vertex_normals():
        mesh_3d.compute_vertex_normals()
    n_3d = np.asarray(mesh_3d.vertex_normals)

    print("   构建基于原始拓扑 (f_3d) 的 2D 查找树...")
    # 【核心修复】：放弃 SciPy 的 Delaunay，使用 matplotlib 将 UV 与原始面片强绑定！
    triangulation = mtri.Triangulation(uv_2d[:, 0], uv_2d[:, 1], f_3d)
    trifinder = triangulation.get_trifinder()

    # 批量查找每个 2D 路径点落在了哪个【原始】三角形面片中
    simplices = trifinder(path_2d[:, 0], path_2d[:, 1])

    path_3d_list = []
    normals_3d_list = []

    for i, p_2d in enumerate(path_2d):
        simplex_idx = simplices[i]

        # 如果找不到对应的三角形（点在网格外部或边界缝隙），则跳过
        if simplex_idx == -1:
            continue

        # 【关键】：这里提取出的是真正的、原始 3D 表面上的三角形顶点
        idx0, idx1, idx2 = f_3d[simplex_idx]
        uv0, uv1, uv2 = uv_2d[idx0], uv_2d[idx1], uv_2d[idx2]

        # 计算重心坐标 (解线性方程组)
        T = np.array([
            [uv0[0] - uv2[0], uv1[0] - uv2[0]],
            [uv0[1] - uv2[1], uv1[1] - uv2[1]]
        ])

        try:
            alpha_beta = np.linalg.solve(T, p_2d - uv2)
            alpha, beta = alpha_beta[0], alpha_beta[1]
            gamma = 1.0 - alpha - beta
        except np.linalg.LinAlgError:
            continue

        # 允许微小的浮点误差越界
        if alpha < -1e-3 or beta < -1e-3 or gamma < -1e-3:
            continue

        # 3D 坐标重构，由于顶点来源于真实面片，路径将完美跟随起伏！
        p_3d = alpha * v_3d[idx0] + beta * v_3d[idx1] + gamma * v_3d[idx2]
        path_3d_list.append(p_3d)

        # 3D 法线插值
        n_interp = alpha * n_3d[idx0] + beta * n_3d[idx1] + gamma * n_3d[idx2]
        n_interp = n_interp / np.linalg.norm(n_interp)
        normals_3d_list.append(n_interp)

    print(f"   逆映射完成！成功还原 {len(path_3d_list)} 个 3D 轨迹点及法线姿态。")
    return np.array(path_3d_list), np.array(normals_3d_list)

# ================= 主程序启动 =================
if __name__ == "__main__":
    print("========== 自由曲面路径规划 Pipeline 启动 ==========\n")

    # 【加载或生成模型】
    mesh_file = "processed_mesh.ply"
    if os.path.exists(mesh_file):
        mesh_3d = load_custom_mesh(mesh_file)
    else:
        mesh_3d = generate_test_open_mesh()

    v_3d, f_3d = np.asarray(mesh_3d.vertices), np.asarray(mesh_3d.triangles)
    print(f"   网格准备完毕！包含 {len(v_3d)} 个顶点, {len(f_3d)} 个面。")

    # 【2D 参数化展开】
    try:
        uv_2d, boundary_indices = parameterize_mesh_to_2d(v_3d, f_3d)
    except Exception as e:
        print(f"参数化展开失败: {e}")
        exit()

    # 【生成 2D 轨迹】
    # spacing 决定了加工路径的密集程度，可根据网格物理尺寸调整
    path_2d, boundary_poly = generate_lawnmower_path(uv_2d, boundary_indices, spacing=0.04)

    # 【中间验证：显示 2D 规划图】
    print("\n>> 准备显示 2D 轨迹蓝图。请关闭弹出的绘图窗口以继续执行 3D 映射...")
    plt.figure(figsize=(8, 8))
    plt.title("2D Lawnmower Path in UV Domain", fontsize=14)

    # 画底层网格、边界和轨迹
    plt.triplot(uv_2d[:, 0], uv_2d[:, 1], f_3d, color='lightgray', linewidth=0.5, alpha=0.5)
    x, y = boundary_poly.exterior.xy
    plt.plot(x, y, color='black', linewidth=2, label='Boundary Polygon')
    plt.plot(path_2d[:, 0], path_2d[:, 1], color='red', linewidth=1.5, marker='.', markersize=4, label='Toolpath')

    if len(path_2d) > 0:
        plt.plot(path_2d[0, 0], path_2d[0, 1], 'go', markersize=8, label='Start')
        plt.plot(path_2d[-1, 0], path_2d[-1, 1], 'bo', markersize=8, label='End')
    plt.axis('equal')
    plt.legend()
    plt.show()  # 此处会阻塞，必须关闭窗口才会进下一步

    # 【执行 3D 逆向映射】
    path_3d, normals_3d = map_2d_path_to_3d(path_2d, uv_2d, v_3d, f_3d, mesh_3d)

    # 【终极目标：3D 轨迹与法线可视化】
    print("\n>> 即将展示最终 3D 轨迹与法线姿态！")

    # 创建红色轨迹线
    lines = [[i, i + 1] for i in range(len(path_3d) - 1)]
    line_set = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(path_3d),
        lines=o3d.utility.Vector2iVector(lines),
    )
    line_set.colors = o3d.utility.Vector3dVector([[1, 0, 0] for _ in range(len(lines))])

    # 创建绿色法线（工具姿态Z轴），每隔10个点抽样显示避免太密
    normal_lines = []
    normal_points = []
    normal_length = 0.03  # 法线显示长度

    for i in range(0, len(path_3d), 10):
        start_pt = path_3d[i]
        end_pt = start_pt + normals_3d[i] * normal_length
        normal_points.extend([start_pt, end_pt])
        normal_lines.append([len(normal_points) - 2, len(normal_points) - 1])

    normal_set = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(normal_points),
        lines=o3d.utility.Vector2iVector(normal_lines),
    )
    normal_set.colors = o3d.utility.Vector3dVector([[0, 1, 0] for _ in range(len(normal_lines))])

    # 将底层网格变浅灰，凸显轨迹
    mesh_3d.compute_vertex_normals()
    mesh_3d.paint_uniform_color([0.8, 0.8, 0.8])

    o3d.visualization.draw_geometries([mesh_3d, line_set, normal_set], window_name="终极目标：3D 随形贴面轨迹与法线",
                                      mesh_show_back_face=True)

    print("\n========== 运行结束 ==========")

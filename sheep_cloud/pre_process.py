import open3d as o3d
import numpy as np

def crop_point_cloud(pcd, min_bound, max_bound):
    """
    使用包围盒裁剪点云，去除远处无关数据
    :param min_bound: 最小坐标列表 [x_min, y_min, z_min]
    :param max_bound: 最大坐标列表 [x_max, y_max, z_max]
    """
    print(f"0. 执行空间裁剪...")
    # 定义包围盒
    bbox = o3d.geometry.AxisAlignedBoundingBox(
        min_bound=np.array(min_bound),
        max_bound=np.array(max_bound)
    )
    # 裁剪
    pcd_cropped = pcd.crop(bbox)

    # 检查裁剪后是否还有点
    if len(pcd_cropped.points) == 0:
        print("警告：裁剪后点云为空！请检查坐标范围。")
    else:
        print(f"裁剪完成，保留点数: {len(pcd_cropped.points)}")

    return pcd_cropped


def preprocess_point_cloud_enhanced(pcd, voxel_size=0.01):
    """
    增强版点云预处理：不再重复读取文件，而是直接处理传入的 pcd 对象
    """
    # --- A. 降采样 (Downsampling) ---
    print(f"2. 执行体素降采样 (Voxel Size: {voxel_size})...")
    pcd_down = pcd.voxel_down_sample(voxel_size=voxel_size)
    print(f"降采样后点数: {len(pcd_down.points)}")

    # --- B. 离群点剔除 (Outlier Removal) ---
    print("3a. 执行统计滤波...")
    cl, ind = pcd_down.remove_statistical_outlier(nb_neighbors=30, std_ratio=1.5)
    pcd_sor = pcd_down.select_by_index(ind)

    print("3b. 执行半径滤波...")
    cl, ind = pcd_sor.remove_radius_outlier(nb_points=12, radius=voxel_size * 2.5)
    pcd_clean = pcd_sor.select_by_index(ind)
    print(f"清洗后剩余点数: {len(pcd_clean.points)}")

    # --- C. 法线估计 (Normal Estimation) ---
    print("4. 法线估计...")
    radius_normal = voxel_size * 3
    pcd_clean.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30)
    )

    # 让法线统一朝向原点
    pcd_clean.orient_normals_towards_camera_location(
        camera_location=np.array([0.0, 0.0, 0.0])
    )

    return pcd_clean



def meshing_poisson(pcd):
    print("\n--- 执行泊松表面重建 ---")
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=9)
    densities = np.asarray(densities)
    vertices_to_remove = densities < np.quantile(densities, 0.1)
    mesh.remove_vertices_by_mask(vertices_to_remove)
    mesh.compute_vertex_normals()
    return mesh


if __name__ == "__main__":
    file_path = "cropped_cloud.pcd"

    try:
        # 1. 加载点云
        raw_pcd = o3d.io.read_point_cloud(file_path)
        if raw_pcd.is_empty():
            raise ValueError("无法读取文件！")
        print(f"原始点数: {len(raw_pcd.points)}")

        # 2. 空间裁剪
        min_bound = [-1.0, -1.0, -1.0]
        max_bound = [1.0, 1.0, 1.0]
        cropped_pcd = crop_point_cloud(raw_pcd, min_bound, max_bound)

        # 3. 预处理 (降采样、滤波、法线)
        clean_pcd = preprocess_point_cloud_enhanced(cropped_pcd, voxel_size=0.01)

        # ---------------------------------------------------------
        # 新增：展示清洗后点云及其法线
        # ---------------------------------------------------------
        print("展示点云及其法线 (按 'n' 键可切换法线显示，按 '-' 或 '+' 调整法线长度)...")
        o3d.visualization.draw_geometries([clean_pcd],
                                          point_show_normal=True,
                                          window_name="点云法线检查")

        # 4. 网格化
        mesh_result = meshing_poisson(clean_pcd)

        # 5. 后处理与可视化
        mesh_result.remove_degenerate_triangles()
        mesh_result.remove_duplicated_vertices()
        mesh_result.remove_unreferenced_vertices()

        bbox = clean_pcd.get_axis_aligned_bounding_box()  # 获取真实点云的包围盒
        mesh_result = mesh_result.crop(bbox)  # 把多余伸出去的网格切掉

        print("展示最终生成的拓扑网格...")
        o3d.visualization.draw_geometries([mesh_result,clean_pcd],
                                          mesh_show_wireframe=True,
                                          point_show_normal=True,
                                          window_name="裁剪并清洗后的模型")

        o3d.io.write_triangle_mesh("processed_mesh.ply", mesh_result)
        print("网格已保存至 processed_mesh.ply")

    except Exception as e:
        print(f"发生错误: {e}")

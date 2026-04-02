import open3d as o3d
import numpy as np
import glob
import copy


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


def pass_through_filter(pcd, axis, min_val, max_val):
    """直通滤波"""
    min_bound = [-np.inf, -np.inf, -np.inf]
    max_bound = [np.inf, np.inf, np.inf]
    min_bound[axis] = min_val
    max_bound[axis] = max_val
    bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound, max_bound)
    return pcd.crop(bbox)


def preprocess_point_cloud(pcd, voxel_size=0.05):
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
    pcd_clean.orient_normals_consistent_tangent_plane(100)

    return pcd_clean


def extract_fpfh_features(pcd, voxel_size=0.005):
    """提取FPFH特征"""
    radius_feature = voxel_size * 5
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100)
    )
    return pcd_fpfh


def fast_global_registration(source, target, voxel_size=0.005):
    """快速全局配准（FGR）"""
    # 提取FPFH特征
    source_fpfh = extract_fpfh_features(source, voxel_size)
    target_fpfh = extract_fpfh_features(target, voxel_size)

    distance_threshold = voxel_size * 0.5

    # Fast Global Registration
    result = o3d.pipelines.registration.registration_fgr_based_on_feature_matching(
        source, target, source_fpfh, target_fpfh,
        o3d.pipelines.registration.FastGlobalRegistrationOption(
            maximum_correspondence_distance=distance_threshold,
            iteration_number=64,  # 增加迭代次数提高精度
            division_factor=1.4,
            use_absolute_scale=False,
            decrease_mu=True,
            maximum_tuple_count=1000
        )
    )
    return result.transformation


def multi_scale_icp(source, target, initial_transformation, voxel_size=0.005):
    """多尺度ICP配准"""
    current_transformation = initial_transformation
    thresholds = [voxel_size * 4, voxel_size * 2, voxel_size * 1]

    for i, threshold in enumerate(thresholds):
        print(f"ICP尺度 {i + 1}/3, 阈值: {threshold:.4f}")

        current_voxel_size = threshold / 2
        source_down = source.voxel_down_sample(current_voxel_size)
        target_down = target.voxel_down_sample(current_voxel_size)

        # 估计法向量（用于点对平面ICP）
        source_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=current_voxel_size * 2, max_nn=30))
        target_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=current_voxel_size * 2, max_nn=30))

        # 使用Generalized ICP（更鲁棒）
        reg_p2p = o3d.pipelines.registration.registration_icp(
            source_down, target_down, threshold, current_transformation,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),  # 点对平面，更稳定
            o3d.pipelines.registration.ICPConvergenceCriteria(
                relative_fitness=1e-6,
                relative_rmse=1e-6,
                max_iteration=50
            )
        )

        current_transformation = reg_p2p.transformation
        print(f"  迭代结果: fitness={reg_p2p.fitness:.4f}, RMSE={reg_p2p.inlier_rmse:.6f}")

    return current_transformation


def fast_gicp_registration(source, target, voxel_size=0.005):
    """快速G-ICP配准（结合FGR和ICP）"""
    print("开始快速全局配准...")

    # 1. 快速全局配准（粗配准）
    start_time = time.time()
    coarse_transformation = fast_global_registration(source, target, voxel_size)
    fgr_time = time.time() - start_time
    print(f"快速全局配准完成，耗时: {fgr_time:.2f}秒")

    # 评估粗配准质量
    source_temp = copy.deepcopy(source)
    source_temp.transform(coarse_transformation)
    distance_threshold = voxel_size * 1.5
    evaluation = o3d.pipelines.registration.evaluate_registration(
        source_temp, target, distance_threshold)

    print(f"粗配准质量: Fitness={evaluation.fitness:.4f}, RMSE={evaluation.inlier_rmse:.6f}")

    # 如果粗配准质量太差，尝试使用传统方法
    if evaluation.fitness < 0.5:
        print("快速全局配准效果不佳，尝试RANSAC...")
        try:
            # 回退到RANSAC
            source_fpfh = extract_fpfh_features(source, voxel_size)
            target_fpfh = extract_fpfh_features(target, voxel_size)

            ransac_result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
                source, target, source_fpfh, target_fpfh, True,
                voxel_size * 1.5,
                o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
                4,
                [o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
                 o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(voxel_size * 1.5)],
                o3d.pipelines.registration.RANSACConvergenceCriteria(40000, 500)
            )
            coarse_transformation = ransac_result.transformation
            print("RANSAC配准完成")
        except:
            print("RANSAC也失败，使用单位矩阵")
            coarse_transformation = np.eye(4)

    # 2. 多尺度ICP精配准
    print("开始多尺度ICP精配准...")
    start_time = time.time()
    fine_transformation = multi_scale_icp(source, target, coarse_transformation, voxel_size)
    icp_time = time.time() - start_time
    print(f"精配准完成，耗时: {icp_time:.2f}秒")

    # 最终质量评估
    source_transformed = copy.deepcopy(source)
    source_transformed.transform(fine_transformation)
    final_evaluation = o3d.pipelines.registration.evaluate_registration(
        source_transformed, target, distance_threshold)

    print(f"最终配准质量:")
    print(f"  Fitness: {final_evaluation.fitness:.4f}")
    print(f"  RMSE: {final_evaluation.inlier_rmse:.6f}")
    print(f"  总耗时: {fgr_time + icp_time:.2f}秒")

    return fine_transformation, final_evaluation.fitness, final_evaluation.inlier_rmse


def colored_icp_registration(source, target, initial_transformation, voxel_size=0.005):
    """彩色ICP配准（如果点云有颜色信息）"""
    if source.has_colors() and target.has_colors():
        print("检测到颜色信息，使用彩色ICP...")
        try:
            # 彩色ICP通常需要更小的阈值
            threshold = voxel_size * 0.5

            result = o3d.pipelines.registration.registration_colored_icp(
                source, target, threshold, initial_transformation,
                o3d.pipelines.registration.TransformationEstimationForColoredICP(),
                o3d.pipelines.registration.ICPConvergenceCriteria(
                    relative_fitness=1e-6,
                    relative_rmse=1e-6,
                    max_iteration=50
                )
            )
            return result.transformation, result.fitness, result.inlier_rmse
        except Exception as e:
            print(f"彩色ICP失败: {e}，回退到普通ICP")

    # 回退到普通ICP
    return multi_scale_icp(source, target, initial_transformation, voxel_size), 0, 0


# -------------------- 多帧点云文件列表 --------------------
pcd_files = sorted(glob.glob("saved_pcds_color/*.pcd"))
print(f"找到 {len(pcd_files)} 个点云文件")

# -------------------- 参数设置 --------------------
voxel_size = 0.01  # 体素大小
axis = 2  # 滤波轴 (Z轴)
min_val = 0  # 最小高度
max_val = 1 # 最大高度
fitness_threshold = 0  # 质量阈值
min_bound = [-0.5, -0.5, -0.5]
max_bound = [0.5, 0.5, 0.5]

# 导入时间模块
import time

# -------------------- 初始化全局点云 --------------------
print("处理第1帧点云...")
global_pcd = o3d.io.read_point_cloud(pcd_files[0], remove_nan_points=True, remove_infinite_points=True)
global_pcd = crop_point_cloud(global_pcd, min_bound, max_bound)
global_pcd = preprocess_point_cloud(global_pcd, voxel_size)

# -------------------- 多帧配准循环（使用快速G-ICP） --------------------
transformation_history = [np.eye(4)]
accepted_frames = [0]
rejected_frames = []
total_start_time = time.time()

for i in range(1, len(pcd_files)):
    print(f"\n{'=' * 50}")
    print(f"正在配准第 {i + 1}/{len(pcd_files)} 帧")
    print(f"{'=' * 50}")

    frame_start_time = time.time()

    # 读取并预处理当前帧
    source_pcd = o3d.io.read_point_cloud(pcd_files[i], remove_nan_points=True, remove_infinite_points=True)
    source_pcd = pass_through_filter(source_pcd, axis, min_val, max_val)
    source_pcd = preprocess_point_cloud(source_pcd, voxel_size)

    # 使用快速G-ICP配准
    transformation, fitness, rmse = fast_gicp_registration(source_pcd, global_pcd, voxel_size)

    frame_time = time.time() - frame_start_time
    print(f"本帧处理耗时: {frame_time:.2f}秒")

    # 质量检查
    if fitness >= fitness_threshold:
        # 应用变换并合并点云
        source_pcd.transform(transformation)
        global_pcd += source_pcd
        transformation_history.append(transformation)
        accepted_frames.append(i)
        print(f"✅ 第 {i + 1} 帧接受 (fitness={fitness:.4f})")
    else:
        rejected_frames.append(i)
        print(f"❌ 第 {i + 1} 帧拒绝 (fitness={fitness:.4f} < {fitness_threshold})")

    # 实时显示进度
    progress = (i + 1) / len(pcd_files) * 100
    elapsed_time = time.time() - total_start_time
    estimated_total = elapsed_time / (i + 1) * len(pcd_files)
    remaining = estimated_total - elapsed_time

    print(f"总体进度: {progress:.1f}% | 已用: {elapsed_time / 60:.1f}分钟 | 剩余: {remaining / 60:.1f}分钟")


# -------------------- 结果统计 --------------------
total_time = time.time() - total_start_time
print(f"\n=== 配准完成 ===")
print(f"总帧数: {len(pcd_files)}")
print(f"接受帧数: {len(accepted_frames)}")
print(f"拒绝帧数: {len(rejected_frames)}")
print(f"接受率: {len(accepted_frames) / len(pcd_files) * 100:.1f}%")
print(f"总耗时: {total_time / 60:.1f} 分钟")
print(f"平均每帧: {total_time / len(pcd_files):.1f} 秒")

# -------------------- 可视化 --------------------
print("\n显示配准结果...")
axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2, origin=[0, 0, 0])
o3d.visualization.draw_geometries([global_pcd, axis],
                                  window_name=f"快速G-ICP配准结果 (接受{len(accepted_frames)}帧)")

# -------------------- 保存结果 --------------------
timestamp = time.strftime("%Y%m%d_%H%M%S")
output_filename = f"merged_cloud_fast_gicp_{timestamp}.pcd"
o3d.io.write_point_cloud(output_filename, global_pcd)
print(f"已保存合并点云: {output_filename}")
#
# # 保存统计信息
# stats = {
#     'total_frames': len(pcd_files),
#     'accepted_frames': accepted_frames,
#     'rejected_frames': rejected_frames,
#     'acceptance_rate': len(accepted_frames) / len(pcd_files),
#     'total_points': len(global_pcd.points),
#     'total_time_seconds': total_time,
#     'voxel_size': voxel_size,
#     'fitness_threshold': fitness_threshold
# }
#
# import json
#
# with open(f'registration_stats_{timestamp}.json', 'w') as f:
#     json.dump(stats, f, indent=2)
# print("已保存统计信息")

print(f"\n配准完成！最终点云包含 {len(global_pcd.points)} 个点")
import open3d as o3d

# 1. 读取点云
pcd = o3d.io.read_point_cloud("filtered.pcd")

# 2. 定义轴对齐边界框 (AxisAlignedBoundingBox)
min_bound = (-1.0, -1.0, 0.0)
max_bound = (2, 0.33, 2)
bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound, max_bound)

# (可选) 给边界框设置颜色，例如红色 (R, G, B)，使其在点云中更醒目
bbox.color = (1.0, 0.0, 0.0)

# 3. 截取点云
cropped_pcd = pcd.crop(bbox)

# 4. 创建坐标系
# size 指定坐标轴的长度，origin 指定坐标系的原点位置
mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
    size=1.0,
    origin=[0.0, 0.0, 0.0]
)

# 5. 可视化所有元素
# 将 点云、边界框、坐标系 一起放入列表中渲染
o3d.visualization.draw_geometries(
    [cropped_pcd, bbox, mesh_frame],  # 这里我放了原点云 pcd，方便你看裁剪框的位置。你也可以换成 cropped_pcd
    window_name="Point Cloud with BBox and Coordinate Frame"
)

# 6. 保存结果
o3d.io.write_point_cloud("cropped_cloud.pcd", cropped_pcd)

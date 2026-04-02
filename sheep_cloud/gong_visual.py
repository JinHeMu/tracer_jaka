import numpy as np
import matplotlib.pyplot as plt

# ==========================================
# 步骤 0: 定义一个模拟的 2D 工件轮廓 (凸多边形)
# ==========================================
# 这里的坐标对应投影后的 (u, v) 平面
polygon = np.array([
    [1.0, 1.0],
    [8.0, 0.5],
    [9.0, 6.0],
    [4.0, 8.0],
    [0.5, 4.0],
    [1.0, 1.0]  # 闭合多边形
])

# ==========================================
# 步骤 1: 行间距计算
# ==========================================
D_tool = 1.0  # 工具直径
r_overlap = 0.15  # 重叠率
d_step = D_tool * (1 - r_overlap)  # 计算出平移步长 (约 0.85)

# ==========================================
# 步骤 2: 扫描行生成
# ==========================================
v_min = np.min(polygon[:, 1])
v_max = np.max(polygon[:, 1])

# 在 V 方向生成等间距的水平线高度 v_k
v_k_list = np.arange(v_min + d_step / 2, v_max, d_step)

path_points = []
all_intersections = []  # 仅用于画图记录

# ==========================================
# 步骤 3, 4, 5: 核心路径生成循环
# ==========================================
for k, v_k in enumerate(v_k_list):
    intersections = []

    # --- 步骤 3: 轮廓交点求解 (射线-边求交) ---
    for i in range(len(polygon) - 1):
        p1 = polygon[i]
        p2 = polygon[i + 1]

        # 判断水平线 v_k 是否穿过线段 p1-p2
        if (p1[1] <= v_k <= p2[1]) or (p2[1] <= v_k <= p1[1]):
            if p1[1] != p2[1]:  # 排除完全水平的边，防止除以零
                # 核心数学公式: 相似三角形求交点
                t = (v_k - p1[1]) / (p2[1] - p1[1])
                u_cross = p1[0] + t * (p2[0] - p1[0])
                intersections.append(u_cross)

    # 如果找到了交点（凸多边形通常是2个）
    if len(intersections) >= 2:
        u_L = min(intersections)  # 最左端起点
        u_R = max(intersections)  # 最右端终点
        all_intersections.append((u_L, u_R, v_k))

        # --- 步骤 4: 在行内密集采样 ---
        # 按照 d_step/4 的间距在 [u_L, u_R] 内布点
        u_samples = np.arange(u_L, u_R, d_step / 4)
        if len(u_samples) == 0 or u_samples[-1] != u_R:
            u_samples = np.append(u_samples, u_R)  # 确保最后一个点贴紧边缘

        # --- 步骤 5: 蛇形反向 ---
        if k % 2 == 1:
            u_samples = u_samples[::-1]  # 奇数行，数组逆序（从右往左走）

        # 将生成的点加入总路径
        for u in u_samples:
            path_points.append([u, v_k])

path_points = np.array(path_points)

# ==========================================
# 可视化绘图部分
# ==========================================
plt.figure(figsize=(10, 8))

# 1. 画出工件边界 (黑线)
plt.plot(polygon[:, 0], polygon[:, 1], 'k-', linewidth=2, label="Workpiece Contour")

# 2. 画出参考水平线和边缘交点 (红点)
for u_L, u_R, v_k in all_intersections:
    plt.hlines(v_k, xmin=u_L, xmax=u_R, colors='gray', linestyles='dashed', alpha=0.5)
    plt.plot([u_L, u_R], [v_k, v_k], 'ro')

# 3. 画出最终生成的弓字形/蛇形路径 (蓝线和蓝点)
plt.plot(path_points[:, 0], path_points[:, 1], 'b.-', linewidth=1.5, markersize=8, label="Boustrophedon Path")

# 4. 标记起点和终点
if len(path_points) > 0:
    plt.plot(path_points[0, 0], path_points[0, 1], 'gs', markersize=10, label="Start (Green)")
    plt.plot(path_points[-1, 0], path_points[-1, 1], 'md', markersize=10, label="End (Purple)")

plt.title("Boustrophedon 2D Path Generation", fontsize=14, fontweight='bold')
plt.xlabel("U axis", fontsize=12)
plt.ylabel("V axis", fontsize=12)
plt.legend(loc='upper right')
plt.grid(True, linestyle=':', alpha=0.7)
plt.axis('equal')  # 确保 X 和 Y 轴比例一致，避免视觉变形
plt.show()

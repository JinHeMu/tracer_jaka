#!/usr/bin/env python3
import numpy as np
from scipy.optimize import least_squares
import os

def identify():
    data_file = "calibration_data.npz"
    if not os.path.exists(data_file):
        print(f"错误: 找不到文件 {data_file}。请先运行采集脚本。")
        return

    loader = np.load(data_file)
    rot_list = loader['rot']
    f_list = loader['force']
    t_list = loader['torque']
    
    print(f"正在对 {len(f_list)} 组离线数据进行参数拟合...")

    def residuals(p):
        mass, com, f_bias, t_bias = p[0], p[1:4], p[4:7], p[7:10]
        g_acc = 9.81
        res = []
        g_world = np.array([0.0, 0.0, -1.0])
        
        for i in range(len(f_list)):
            R_mat = rot_list[i]
            g_s = R_mat.T @ g_world
            f_model = mass * g_acc * g_s + f_bias
            t_model = np.cross(com, (mass * g_acc * g_s)) + t_bias
            res.extend(f_list[i] - f_model)
            res.extend(t_list[i] - t_model)
        return np.array(res)

    # 初始值猜想
    p0 = np.array([1.3, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    result = least_squares(residuals, p0, method='lm')

    if result.success:
        p = result.x
        print("\n" + "--- 辨识结果 ---")
        print(f"MASS   = {p[0]:.6f}")
        print(f"COM    = [{p[1]:.7f}, {p[2]:.7f}, {p[3]:.7f}]")
        print(f"F_BIAS = [{p[4]:.4f}, {p[5]:.4f}, {p[6]:.4f}]")
        print(f"T_BIAS = [{p[7]:.4f}, {p[8]:.4f}, {p[9]:.4f}]")
        print("-" * 15)
    else:
        print("辨识失败。")

if __name__ == '__main__':
    identify()

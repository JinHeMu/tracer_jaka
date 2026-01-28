import numpy as np
import csv
from scipy.spatial.transform import Rotation as R

class GravityCompensator:
    def __init__(self, g_value=9.81):
        self.g = g_value
        self.data_R = []
        self.data_F = []
        self.data_T = []

    def add_data(self, R_matrix, F_measured, T_measured):
        self.data_R.append(np.array(R_matrix))
        self.data_F.append(np.array(F_measured))
        self.data_T.append(np.array(T_measured))

    def solve(self):
        N = len(self.data_F)
        if N == 0:
            return None
            
        print(f"正在基于 {N} 组有效数据进行计算...")

        # --- 1. 标定力 (Mass & Force Bias) ---
        # F_meas = R^T * [0, 0, -mg] + F_bias
        A_force = []
        B_force = []

        for i in range(N):
            # 重力在传感器系下的方向向量 (假设基座重力向下 -Z)
            # g_dir = R^T * [0, 0, -1]
            g_dir = self.data_R[i].T @ np.array([0.0, 0.0, -1.0])
            
            F_meas = self.data_F[i]
            
            # 构建 Ax = B
            A_force.append([g_dir[0], 1, 0, 0])
            B_force.append(F_meas[0])
            A_force.append([g_dir[1], 0, 1, 0])
            B_force.append(F_meas[1])
            A_force.append([g_dir[2], 0, 0, 1])
            B_force.append(F_meas[2])

        # 最小二乘求解
        x_F, residuals, _, _ = np.linalg.lstsq(A_force, B_force, rcond=None)
        
        W_calib = x_F[0]         # 重力 mg (N)
        mass_calib = W_calib / self.g # 质量 m (kg)
        F_bias_calib = x_F[1:]   # 力偏置 [Fx0, Fy0, Fz0]

        # --- 2. 标定力矩 (CoM & Torque Bias) ---
        # T_meas = P_com x F_gravity + T_bias
        A_torque = []
        B_torque = []

        for i in range(N):
            g_dir = self.data_R[i].T @ np.array([0.0, 0.0, -1.0])
            # 使用标定出的重力大小计算当前的重力向量
            F_grav = W_calib * g_dir 
            
            Fx, Fy, Fz = F_grav
            Tx, Ty, Tz = self.data_T[i]

            # 叉乘矩阵形式: P x F
            # [ 0, -Fz,  Fy, 1, 0, 0] * [cx, cy, cz, Tx0, Ty0, Tz0] = Tx
            A_torque.append([0, -Fz, Fy, 1, 0, 0])
            B_torque.append(Tx)
            A_torque.append([Fz, 0, -Fx, 0, 1, 0])
            B_torque.append(Ty)
            A_torque.append([-Fy, Fx, 0, 0, 0, 1])
            B_torque.append(Tz)

        x_T, _, _, _ = np.linalg.lstsq(A_torque, B_torque, rcond=None)
        com_calib = x_T[0:3]     # 重心 [cx, cy, cz]
        T_bias_calib = x_T[3:]   # 力矩偏置

        # 计算残差 (验证精度)
        rmse_F = np.sqrt(residuals[0] / (3*N)) if len(residuals) > 0 else 0.0
        
        return mass_calib, com_calib, F_bias_calib, T_bias_calib, rmse_F

def load_and_calculate(csv_path):
    calib = GravityCompensator()
    count = 0
    
    try:
        with open(csv_path, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                # 1. 解析四元数并转换为旋转矩阵
                # CSV header: quat_x, quat_y, quat_z, quat_w
                qx = float(row['quat_x'])
                qy = float(row['quat_y'])
                qz = float(row['quat_z'])
                qw = float(row['quat_w'])
                
                # scipy 的 Rotation 默认是 [x, y, z, w]
                r = R.from_quat([qx, qy, qz, qw])
                R_matrix = r.as_matrix()
                
                # 2. 解析力和力矩
                Fx = float(row['force_x'])
                Fy = float(row['force_y'])
                Fz = float(row['force_z'])
                
                Tx = float(row['torque_x'])
                Ty = float(row['torque_y'])
                Tz = float(row['torque_z'])
                
                calib.add_data(R_matrix, [Fx, Fy, Fz], [Tx, Ty, Tz])
                count += 1
                
    except FileNotFoundError:
        print(f"错误: 找不到文件 {csv_path}")
        return

    # 执行计算
    result = calib.solve()
    
    if result:
        mass, com, f_bias, t_bias, rmse = result
        
        print("\n" + "="*50)
        print("           重力补偿参数标定报告")
        print("="*50)
        print(f"输入数据量: {count} 组")
        print("-" * 30)
        print(f"1. 负载质量 (Mass):")
        print(f"   m = {mass:.5f} kg")
        print("-" * 30)
        print(f"2. 质心坐标 (CoM) [相对于传感器中心]:")
        print(f"   X = {com[0]*1000:8.4f} mm")
        print(f"   Y = {com[1]*1000:8.4f} mm")
        print(f"   Z = {com[2]*1000:8.4f} mm")
        print("-" * 30)
        print(f"3. 力零点偏置 (Force Bias):")
        print(f"   Fx = {f_bias[0]:8.4f} N")
        print(f"   Fy = {f_bias[1]:8.4f} N")
        print(f"   Fz = {f_bias[2]:8.4f} N")
        print("-" * 30)
        print(f"4. 力矩零点偏置 (Torque Bias):")
        print(f"   Tx = {t_bias[0]:8.4f} Nm")
        print(f"   Ty = {t_bias[1]:8.4f} Nm")
        print(f"   Tz = {t_bias[2]:8.4f} Nm")
        print("="*50)
        print(f"拟合均方根误差 (RMSE Force): {rmse:.5f} N")
        print("说明: RMSE越小，说明数据质量越高，模型拟合越好。")
        print("="*50)
        
        # 验证: 使用第一组数据进行回代验证
        print("\n[验证] 第一组数据补偿测试:")
        # 取第一组
        r0 = R.from_quat([
            float(calib.data_R[0][0,0]), # 这里简单取个对象，逻辑上应该重新读取
            0,0,0 # 占位
        ]) 
        # 重新取第一组原始数据
        R_ver = calib.data_R[0]
        F_ver = calib.data_F[0]
        T_ver = calib.data_T[0]
        
        # 计算理论重力
        g_dir = R_ver.T @ np.array([0.0, 0.0, -1.0])
        F_gravity = (mass * 9.81) * g_dir
        
        # 补偿后的力
        F_comp = F_ver - F_gravity - f_bias
        
        print(f"原始读数: {F_ver}")
        print(f"理论重力: {F_gravity}")
        print(f"补偿后力: {F_comp} (理论应接近 [0,0,0])")

if __name__ == "__main__":
    # 确保 data.csv 在当前目录下
    load_and_calculate("ft_calibration_data.csv")

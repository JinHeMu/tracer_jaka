import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from dh5_interfaces.srv import SetAxisValue, Initialize, SetValues
from dh5_interfaces.msg import AxisInfos
from .DH5 import DH5ModbusAPI


class DH5Controller(Node):
    def __init__(self):
        super().__init__('dh5_controller_node')

        self.api = DH5ModbusAPI(port='/dev/ttyUSB0')
        self.api.open_connection()

        self.pub_joint_state = self.create_publisher(AxisInfos, 'dh5/AxisInfos', 10)

        self.create_service(SetValues, 'dh5/set_position', self.set_position_cb)
        self.create_service(SetValues, 'dh5/set_speed', self.set_speed_cb)
        self.create_service(SetValues, 'dh5/set_force', self.set_force_cb)

        self.create_service(SetAxisValue, 'dh5/set_axis_position', self.set_axis_position_cb)
        self.create_service(SetAxisValue, 'dh5/set_axis_speed', self.set_axis_speed_cb)
        self.create_service(SetAxisValue, 'dh5/set_axis_force', self.set_axis_force_cb)
        self.create_service(Initialize, 'dh5/initialize', self.initialize_cb)

        self.create_service(Trigger, 'dh5/clear_cur_fault', self.clear_cur_fault)
        self.create_service(Trigger, 'dh5/clear_history_faults', self.clear_history_faults)
        self.create_service(Trigger, 'dh5/restart_system', self.restart_system)


        self.create_service(Trigger, 'dh5/get_faults', self.get_history_faults_cb)

        self.timer = self.create_timer(0.5, self.publish_joint_states)
        self.get_logger().info("DH5 Controller Node started.")

    def set_position_cb(self, request, response):
        result = self.api.set_position(request.value)
        response.success = (result == 0)
        response.message = 'OK' if response.success else str(result)
        return response

    def set_speed_cb(self, request, response):
        result = self.api.set_speed(request.value)
        response.success = (result == 0)
        response.message = 'OK' if response.success else str(result)
        return response

    def set_force_cb(self, request, response):
        result = self.api.set_force(request.value)
        response.success = (result == 0)
        response.message = 'OK' if response.success else str(result)
        return response

    def set_axis_position_cb(self, request, response):
        result = self.api.set_axis_position(request.axis, request.value)
        response.success = (result == 0)
        response.message = 'OK' if response.success else str(result)
        return response

    def set_axis_speed_cb(self, request, response):
        result = self.api.set_axis_speed(request.axis, request.value)
        response.success = (result == 0)
        response.message = 'OK' if response.success else str(result)
        return response

    def set_axis_force_cb(self, request, response):
        result = self.api.set_axis_force(request.axis, request.value)
        response.success = (result == 0)
        response.message = 'OK' if response.success else str(result)
        return response

    def initialize_cb(self, request, response):
        result = self.api.initialize(request.mode)
        response.success = (result == 0)
        response.message = 'OK' if response.success else str(result)
        return response

    def clear_cur_fault(self,request, response):
        result = self.api.reset_faults()
        response.success = (result == 0)
        response.message = 'OK' if response.success else str(result)
        return response

    def clear_history_faults(self, request, response):
        result = self.api.reset_history_faults()
        response.success = (result == 0)
        response.message = 'OK' if response.success else str(result)
        return response

    def restart_system(self, request, response):
        result = self.api.restart_system()
        response.success = (result == 0)
        response.message = 'OK' if response.success else str(result)
        return response

    def get_cur_faults_cb(self, response):
        faults = self.api.get_cur_faults()
        response.success = isinstance(faults, list)
        response.message = str(faults)
        return response

    def get_history_faults_cb(self, request, response):
        faults = self.api.get_history_faults()
        response.success = isinstance(faults, list)
        response.message = str(faults)
        return response


    def to_short(self, x):
        x = x & 0xFFFF  # 限制在 16 位
        return x

    def publish_joint_states(self):
        joint_names = [
            '01:Thumb_Yaw',  # 01
            '02:Index',  # 02
            '03:Middle',  # 03
            '04:Ring',  # 04
            '05:Pinky',  # 05
            '06:Thumb_Pitch'  # 06
        ]

        msg = AxisInfos()
        msg.name = joint_names
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.position = []
        msg.velocity = []
        msg.currents = []  # 当前电流
        msg.cur_faults = []

        # 位置
        pos_val_list = self.api.get_position_fd()
        if (type(pos_val_list) == list):
            for pos_val in pos_val_list:
                pos_int = self.to_short(pos_val)
                msg.position.append((pos_int))

        # 速度
        vel_val_list = self.api.get_speed_fd()
        if (type(vel_val_list) == list):
            for vel_val in vel_val_list:
                vel_int = self.to_short((vel_val))
                msg.velocity.append((vel_int))

        # 电流
        cur_val_list = self.api.get_current_fd()
        if (type(cur_val_list) == list):
            for cur_val in cur_val_list:
                cur_int = self.to_short(cur_val)
                msg.currents.append((cur_int))

        # 获取当前错误
        msg.cur_faults.append(self.api.get_cur_faults()[0])
        
        self.pub_joint_state.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DH5Controller()
    rclpy.spin(node)
    rclpy.shutdown()

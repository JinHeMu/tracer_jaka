import time

import serial
import struct

class DH5ModbusAPI:
    SUCCESS = 0
    ERROR_CONNECTION_FAILED = 1
    ERROR_INVALID_RESPONSE = 2
    ERROR_CRC_CHECK_FAILED = 3
    ERROR_INVALID_COMMAND = 4
    def __init__(self, port='COM6', modbus_id=1, baud_rate=115200, stop_bits=1, parity='N'):
        self.port = port
        self.modbus_id = modbus_id
        self.baud_rate = baud_rate
        self.stop_bits = stop_bits
        self.parity = parity
        self.serial_connection = None
    def open_connection(self):
        try:
            self.serial_connection = serial.Serial(
                port=self.port,
                baudrate=self.baud_rate,
                stopbits=self.stop_bits,
                parity=self.parity,
                timeout=0.05
            )
            if self.serial_connection.is_open:
                print("Serial connection opened successfully")
                return self.SUCCESS
        except Exception as e:
            return f"Failed to open serial connection: {str(e)}"

    def close_connection(self):
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.close()
            return self.SUCCESS


    def send_modbus_command(self, function_code, register_address, data=None, data_length=None):
        if not self.serial_connection or not self.serial_connection.is_open:
            return self.ERROR_CONNECTION_FAILED

        try:
            if function_code == 0x03:  # Read Holding Registers
                message = self._build_request(function_code, register_address, data_length=data_length or 1)
            elif function_code == 0x06:  # Write Single Register
                message = self._build_request(function_code, register_address, value=data)
            elif function_code == 0x10:  # Write Multiple Registers
                message = self._build_request(function_code, register_address, values=data, data_length=data_length)
            else:
                return self.ERROR_INVALID_COMMAND

            self.serial_connection.write(message)
            response = self.serial_connection.read(256)
            return self._parse_response(response, function_code)
        except Exception as e:
            return f"Error: {str(e)}"

    def _build_request(self, function_code, register_address, data_length=1, value=None, values=None):
        request = bytearray()
        request.append(self.modbus_id)
        request.append(function_code)
        request += struct.pack('>H', register_address)

        if function_code == 0x03:  # Read Holding Registers
            request += struct.pack('>H', data_length)
        elif function_code == 0x06:  # Write Single Register
            request += struct.pack('>H', value)
        elif function_code == 0x10:  # Write Multiple Registers
            register_count = data_length if data_length else len(values)
            request += struct.pack('>H', register_count)  # Number of registers
            request.append(register_count * 2)  # Byte count (register_count * 2 bytes per register)
            for val in values:
                request += struct.pack('>H', val)

        crc = self._calculate_crc(request)
        request += struct.pack('<H', crc)
        return request

    def _calculate_crc(self, data):
        crc = 0xFFFF
        for pos in data:
            crc ^= pos
            for _ in range(8):
                if crc & 0x0001:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        return crc

    def _parse_response(self, response, function_code):
        if len(response) < 5:
            # raise ValueError("Incomplete response received")
            return self.ERROR_INVALID_RESPONSE

        device_id, func_code, *payload, crc_low, crc_high = response
        if func_code != function_code:
            # raise ValueError(f"Unexpected function code: {func_code}")
            return self.ERROR_INVALID_RESPONSE

        crc_received = (crc_high << 8) | crc_low
        crc_calculated = self._calculate_crc(response[:-2])
        if crc_received != crc_calculated:
            return self.ERROR_CRC_CHECK_FAILED

        if function_code == 0x03:
            data = payload[1:]  # Skip byte count
            return [struct.unpack('>H', bytes(data[i:i + 2]))[0] for i in range(0, len(data), 2)]
        elif function_code in [0x06, 0x10]:
            return self.SUCCESS

    # -------------------- Configuration Functions --------------------
    def set_config(self, modbus_id=None, baud_rate=None, stop_bits=None, parity=None):
        if modbus_id:
            self.modbus_id = modbus_id
        if baud_rate:
            self.baud_rate = baud_rate
        if stop_bits:
            self.stop_bits = stop_bits
        if parity:
            self.parity = parity

    # -------------------- API Methods --------------------

    def set_uart_config(self, modbus_id=None, baud_rate=None, stop_bits=None, parity=None):
        uart_registers = [modbus_id, baud_rate, stop_bits, parity]
        self.send_modbus_command(function_code=0x10, register_address=0x0302, data=uart_registers,
                                 data_length=len(uart_registers))

    def set_save_param(self, flag = 1):
        self.send_modbus_command(function_code=0x06, register_address=0x0300, data=flag)

    def initialize(self, mode):
        """
        Initialize all 6 axes with a specific mode.
        Modes:
          - 0b01: Close
          - 0b10: Open
          - 0b11: Find total stroke
        """
        if mode not in [0b01, 0b10, 0b11]:
            return self.ERROR_INVALID_COMMAND

        data = 0
        for axis in range(6):
            data |= (mode << (axis * 2))  # Set all axes to the given mode
        return self.send_modbus_command(function_code=0x06, register_address=0x0100, data=data)

    def initialize_axis(self, axis, mode):
        """
        Initialize a specific axis with a specific mode.
        Modes:
          - 0b01: Close
          - 0b10: Open
          - 0b11: Find total stroke
        """
        if axis < 1 or axis > 6:
            return self.ERROR_INVALID_COMMAND
        if mode not in [0b01, 0b10, 0b11]:
            return self.ERROR_INVALID_COMMAND


        init_status = 0
        # Set 2 bits for the specified axis
        init_status &= ~(0b11 << ((axis - 1) * 2))  # Clear the bits for the axis
        init_status |= (mode << ((axis - 1) * 2))  # Set the mode bits for the axis
        return self.send_modbus_command(function_code=0x06, register_address=0x0100, data=init_status)

    def check_initialization(self):
        """
        Check the initialization status of all 6 axes.
        Returns:
            A dictionary where each key represents an axis (axis_F1, axis_F2, etc.) and the value is the initialization status:
              - "not initialized": Axis has no initialization (00)
              - "initialized": Axis initialization successful (01)
              - "initializing": Axis is currently initializing (10)
        """
        response = self.send_modbus_command(function_code=0x03, register_address=0x0200, data_length=1)
        if isinstance(response, list) and len(response) > 0:
            init_status = response[0]
            status = {}
            for axis in range(6):
                axis_status = (init_status >> (axis * 2)) & 0b11  # Extract 2 bits for each axis
                if axis_status == 0b01:
                    status[f"axis_F{axis + 1}"] = "initialized"
                elif axis_status == 0b10:
                    status[f"axis_F{axis + 1}"] = "initializing"
                else:
                    status[f"axis_F{axis + 1}"] = "not initialized"
            return status
        return self.ERROR_INVALID_RESPONSE

    def set_position(self, position: list):
        if len(position) != 6:
            return self.ERROR_INVALID_COMMAND
        register_address = 0x0101
        return self.send_modbus_command(function_code=0x10, register_address=register_address, data=position,
                                        data_length=6)
    def set_speed(self, speed: list):
        if len(speed) != 6:
            return self.ERROR_INVALID_COMMAND
        i = 0
        while i < len(speed):
            if speed[i] < 1 or speed[i] > 100:
                return self.ERROR_INVALID_COMMAND
            i += 1
        register_address = 0x010D
        return self.send_modbus_command(function_code=0x10, register_address=register_address, data=speed,
                                        data_length=6)
    def set_force(self, force: list):
        if len(force) != 6:
            return self.ERROR_INVALID_COMMAND

        i = 0
        while i < len(force):
            if force[i] < 20 or force[i] > 100:
                return self.ERROR_INVALID_COMMAND
            i += 1
        register_address = 0x0107
        return self.send_modbus_command(function_code=0x10, register_address=register_address, data=force,
                                        data_length=6)
    def get_seted_position(self):
        register_address = 0x0101
        return self.send_modbus_command(function_code=0x03, register_address=register_address,data_length=6)
    def get_seted_speed(self):
        register_address = 0x010D
        return self.send_modbus_command(function_code=0x03, register_address=register_address,data_length=6)

    def get_seted_force(self):
        register_address = 0x0107
        return self.send_modbus_command(function_code=0x03, register_address=register_address,data_length=6)

    def get_position_fd(self):
        register_address = 0x0207
        return self.send_modbus_command(function_code=0x03, register_address=register_address, data_length=6)

    def get_speed_fd(self):
        register_address = 0x020D
        return self.send_modbus_command(function_code=0x03, register_address=register_address, data_length=6)

    def get_current_fd(self):
        register_address = 0x0213
        return self.send_modbus_command(function_code=0x03, register_address=register_address, data_length=6)

    def set_axis_position(self, axis, position):
        if axis < 1 or axis > 6:
            return self.ERROR_INVALID_COMMAND
        register_address = 0x0101 + (axis - 1)
        return self.send_modbus_command(function_code=0x06, register_address=register_address, data=position)

    def set_axis_speed(self, axis, speed):
        if axis < 1 or axis > 6:
            return self.ERROR_INVALID_COMMAND
        register_address = 0x010D + (axis - 1)
        return self.send_modbus_command(function_code=0x06, register_address=register_address, data=speed)

    def set_axis_force(self, axis, force):
        if axis < 1 or axis > 6:
            return self.ERROR_INVALID_COMMAND
        register_address = 0x0107 + (axis - 1) * 0x10
        return self.send_modbus_command(function_code=0x06, register_address=register_address, data=force)

    def get_axis_position(self, axis):
        if axis < 1 or axis > 6:
            return self.ERROR_INVALID_COMMAND
        register_address = 0x0207 + (axis - 1)
        return self.send_modbus_command(function_code=0x03, register_address=register_address)

    def get_axis_speed(self, axis):
        if axis < 1 or axis > 6:
            return self.ERROR_INVALID_COMMAND
        register_address = 0x020D + (axis - 1)
        return self.send_modbus_command(function_code=0x03, register_address=register_address)

    def get_axis_current(self, axis):
        if axis < 1 or axis > 6:
            return self.ERROR_INVALID_COMMAND
        register_address = 0x0213 + (axis - 1)
        return self.send_modbus_command(function_code=0x03, register_address=register_address)

    def get_cur_faults(self):
        return self.send_modbus_command(function_code=0x03, register_address=0x021F, data_length=1)

    def get_history_faults(self):
        return self.send_modbus_command(function_code=0x03, register_address=0x0B00, data_length=0x3F)

    def reset_faults(self):
        return self.send_modbus_command(function_code=0x06, register_address=0x0501, data=1)

    def reset_history_faults(self):
        return self.send_modbus_command(function_code=0x06, register_address=0x0B3F, data=1)

    def restart_system(self):
        return self.send_modbus_command(function_code=0x06, register_address=0x0503, data=1)



# Example usage
if __name__ == '__main__':
    api = DH5ModbusAPI(port='COM14', baud_rate=115200)
    api.open_connection()

    # 示例
    # 初始化：
    # api.initialize(mode=0b10)
    # 设置6个轴的位置
    # _position_list = [100, 100, 100, 100, 100, 100]
    # print(api.set_position(_position_list))

    # time.sleep(2)
    # 设置6个轴的位置
    # _position_list = [400, 1000, 1000, 1000, 1000, 400]
    # api.set_position(_position_list)


    # 设置6个轴的速度指令
    # _speed_list = [10, 10, 10, 10, 10, 10]
    # api.set_speed(_speed_list)
    # # 获取当前的设置值
    # print(api.get_seted_speed())

    # 设置6个轴的速度指令
    # _force_list = [30, 30, 30, 30, 30, 30]
    # api.set_force(_force_list)
    # time.sleep(0.1)
    # # 获取当前的设置值
    # print(api.get_seted_force())

    # 获取历史故障码
    # print(api.get_history_faults())

    # # 获取当前错误
    # print(api.get_cur_faults())
    # # 清除当前错误
    # print(api.reset_faults())
    # # 获取当前错误
    # print(api.get_cur_faults())

    # # 获取已经设置的 位置
    # print(api.get_seted_position())
    # # 获取已经设置的 速度
    # print(api.get_seted_speed())
    # # 获取已经设置的 力
    # print(api.get_seted_force())

    # # 获取当前电流反馈值
    # print(api.get_current_fd())
    #
    # # 获取当前位置反馈值
    # print(api.get_position_fd())
    #
    # # 获取当前速度反馈值
    # print(api.get_speed_fd())

    # 重启系统
    # print(api.restart_system())

    print("end of test")

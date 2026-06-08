def calculate_checksum(data):
    return sum(data) & 0xFF


class UartProtocol:
    """Builds and sends RT1021 UART packets."""

    def __init__(self, uart):
        self.uart = uart

    def build_target_packet(self, color_id, cx, cy, w, h, distance):
        data = bytearray(14)
        data[0] = 0xAA
        data[1] = 0x55
        data[2] = color_id
        data[3] = cx & 0xFF
        data[4] = (cx >> 8) & 0xFF
        data[5] = cy & 0xFF
        data[6] = (cy >> 8) & 0xFF
        data[7] = w & 0xFF
        data[8] = (w >> 8) & 0xFF
        data[9] = h & 0xFF
        data[10] = (h >> 8) & 0xFF
        if distance < 0:
            distance = 0
        data[11] = distance & 0xFF
        data[12] = (distance >> 8) & 0xFF
        data[13] = calculate_checksum(data[2:13])
        return data

    def send_target_data(self, color_id, cx, cy, w, h, distance):
        data = self.build_target_packet(color_id, cx, cy, w, h, distance)
        self.uart.write(data)
        return data

    def send_no_target(self):
        data = bytearray(14)
        data[0] = 0xAA
        data[1] = 0x55
        data[13] = 0
        self.uart.write(data)
        return data

    def send_world_data(self, color_id, wx_mm, wy_mm, pw, yellow_flag=False, pos_flag=0x00,
                        obstacle_flag=0x00, angle_flag=0x00, angle_cdeg=0):
        data = bytearray(16)
        data[0] = 0xAA
        data[1] = 0x55
        data[2] = color_id & 0xFF
        data[3] = wx_mm & 0xFF
        data[4] = (wx_mm >> 8) & 0xFF
        data[5] = wy_mm & 0xFF
        data[6] = (wy_mm >> 8) & 0xFF
        data[7] = pw & 0xFF
        data[8] = (pw >> 8) & 0xFF
        data[9] = 0x01 if yellow_flag else 0x00
        data[10] = pos_flag & 0xFF
        data[11] = obstacle_flag & 0xFF
        data[12] = angle_flag & 0xFF
        data[13] = angle_cdeg & 0xFF
        data[14] = (angle_cdeg >> 8) & 0xFF
        data[15] = sum(data[2:15]) & 0xFF
        self.uart.write(data)
        return data

    def send_world_no_target(self, yellow_flag=False, pos_flag=0x00, obstacle_flag=0x00,
                             angle_flag=0x00, angle_cdeg=0):
        data = bytearray(16)
        data[0] = 0xAA
        data[1] = 0x55
        data[9] = 0x01 if yellow_flag else 0x00
        data[10] = pos_flag & 0xFF
        data[11] = obstacle_flag & 0xFF
        data[12] = angle_flag & 0xFF
        data[13] = angle_cdeg & 0xFF
        data[14] = (angle_cdeg >> 8) & 0xFF
        data[15] = sum(data[2:15]) & 0xFF
        self.uart.write(data)
        return data

import serial

class dh_device(object):
    def __init__(self):
        # 每个设备实例拥有独立的串口连接
        self.serialPort = serial.Serial()
        self.serialPort.bytesize = 8
        self.serialPort.parity = 'N'
        self.serialPort.stopbits = 1
        self.serialPort.set_output_flow_control = 'N'
        self.serialPort.set_input_flow_control = 'N'

    def connect_device(self, portname, Baudrate):
        """连接设备"""
        try:
            if self.serialPort.isOpen():
                self.serialPort.close()
                
            self.serialPort.port = portname
            self.serialPort.baudrate = Baudrate
            self.serialPort.open()
            
            if self.serialPort.isOpen():
                print(f'Serial {portname} Open Success')
                return 0
            else:
                print(f'Serial {portname} Open Error')
                return -1
        except Exception as e:
            print(f'Serial {portname} Open Exception: {str(e)}')
            return -1

    def disconnect_device(self):
        """断开设备连接"""
        if self.serialPort.isOpen():
            self.serialPort.close()
            print('Serial Port Closed')

    def device_write(self, write_data):
        """写入数据"""
        try:
            if self.serialPort.isOpen():
                write_length = self.serialPort.write(write_data)
                return write_length if write_length == len(write_data) else 0
            return -1
        except Exception as e:
            print(f'Write error: {str(e)}')
            return -1

    def device_read(self, wlen):
        """读取数据"""
        try:
            if self.serialPort.isOpen():
                return self.serialPort.read(wlen)
            return -1
        except Exception as e:
            print(f'Read error: {str(e)}')
            return -1
#!/usr/bin/env python3
import serial
import sys

def read_serial_data(port="/dev/ttyACM0", baudrate=115200, timeout=2):
    """
    读取串口数据并以16进制和10进制显示
    """
    try:
        # 打开串口
        ser = serial.Serial(port, baudrate, timeout=timeout)
        print(f"成功打开串口: {port}, 波特率: {baudrate}")
        print("按 Ctrl+C 停止读取\n")
        
        # 计数器
        packet_count = 0
        total_bytes = 0
        
        while True:
            if ser.in_waiting > 0:
                # 读取可用数据
                data = ser.read(ser.in_waiting)
                total_bytes += len(data)
                packet_count += 1
                
                # 显示16进制
                hex_str = " ".join(f"{byte:02x}" for byte in data)
                print(f"[包 {packet_count}] 16进制: {hex_str}")
                
                # 显示10进制
                dec_str = " ".join(f"{byte:3d}" for byte in data)
                print(f"[包 {packet_count}] 10进制: {dec_str}")
                
                # 显示原始ASCII（如果可打印）
                ascii_str = "".join(chr(byte) if 32 <= byte < 127 else "." for byte in data)
                print(f"[包 {packet_count}] ASCII : {ascii_str}")
                print(f"长度: {len(data)} 字节, 累计: {total_bytes} 字节\n")
    
    except serial.SerialException as e:
        print(f"串口错误: {e}")
        sys.exit(1)
    except KeyboardInterrupt:
        print(f"\n已停止。总共接收 {total_bytes} 字节，{packet_count} 个包")
        ser.close()

if __name__ == "__main__":
    port = "/dev/ttyACM0"
    if len(sys.argv) > 1:
        port = sys.argv[1]
    
    read_serial_data(port)
